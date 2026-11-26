"""The assistant's hands and eyes on the robot: every tool goes through here, and from here only
through the robot's own services and actions, the same ones the app uses. So the mode manager
decides every mode change, Nav2 plans every trip, the cleaning node runs every clean, and the
safety gate has the last word on the wheels. Nothing here publishes a speed.

Long actions (a trip, a look around) wait here, report what they are doing through `doing`, and
end early when the robot leaves the mode they need (the user pressed Stop in the app).
"""

import math
import threading
import time
from typing import Callable, List, Optional, Sequence, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose, Spin
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from vacuum_interfaces.msg import CleaningJob, CleaningStatus, RobotSettings, RobotState, Rooms
from vacuum_interfaces.srv import (ChangeState, FindObjects, FindRooms, ListCleaningHistory, RoomReport, SetRobotSettings, SetRooms,
                                   StartCleaning)

from vacuum_mode_manager import map_files

from . import describe
from .memory import Memory
from .tools import RobotError

CALL_TIMEOUT = 15.0  # s for a quick service (history, settings, rooms)
MODE_TIMEOUT = 150.0  # s for a mode change (Nav2 starting takes 10 to 30 s)
TRIP_TIMEOUT = 300.0  # s for one trip across the home
NEAR = 0.45  # m from the near side of a thing to the robot's centre when it goes up to it
SURE = 0.35  # the object finder's score from which a find is remembered (a hint of where to look later)
MAYBE = 0.22  # from this score a find is worth checking (the finder scored a real sofa 0.32)
VIEWPOINTS = 3  # other places in the room to look from when a thing is not seen from here
VIEW_GRID = 1.3  # m between the places tried
CLOSE_IN = 1.4  # m: a thing farther than this is approached in two steps, looking again on the way
LOOK_STEPS = 6  # pictures taken turning on the spot to look all around
_CAN_DRIVE_FROM = ("idle", "docked", "manual", "navigation")


def inside(polygon: Sequence[Tuple[float, float]], x: float, y: float) -> bool:
    """Is (x, y) inside the polygon (even-odd rule)?"""
    hit = False
    for (x0, y0), (x1, y1) in zip(polygon, list(polygon[1:]) + list(polygon[:1])):
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            hit = not hit
    return hit


# The camera's pictures are about 1 MB each. Sent best-effort, a picture is lost whenever one of
# its pieces is: only 59 % of them arrived. Reliable, every one arrives, about 16 ms after it
# was taken. Only the newest is kept.
PICTURE_QOS = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE)


class RosLink:
    def __init__(self, node, doing: Callable[[str], None] = lambda _text: None, maps_dir: str = "",
                 verify: Optional[Callable[[bytes, str, str], bool]] = None):
        """`verify(picture, name, which box)`: is the thing in that box of the marked picture really a
        `name`? (The object finder proposes, often wrongly; the language model checks.) None: the
        finder's score decides."""
        self.node = node
        self.doing = doing
        self.verify = verify
        self.maps_dir = maps_dir
        self._memory: Optional[Memory] = None
        self._memory_map = None
        self.robot_state: Optional[RobotState] = None
        self.cleaning: Optional[CleaningStatus] = None
        self.settings: Optional[RobotSettings] = None
        self.rooms: Optional[Rooms] = None
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        data = MutuallyExclusiveCallbackGroup()
        node.create_subscription(RobotState, "robot_state", lambda m: setattr(self, "robot_state", m), latched, callback_group=data)
        node.create_subscription(CleaningStatus, "cleaning/status", lambda m: setattr(self, "cleaning", m), latched, callback_group=data)
        node.create_subscription(RobotSettings, "robot_settings", lambda m: setattr(self, "settings", m), latched, callback_group=data)
        node.create_subscription(Rooms, "rooms", lambda m: setattr(self, "rooms", m), latched, callback_group=data)
        self._group = ReentrantCallbackGroup()
        c = lambda kind, name: node.create_client(kind, name, callback_group=self._group)  # noqa: E731
        self._change_state = c(ChangeState, "change_state")
        self._history = c(ListCleaningHistory, "cleaning/history")
        self._room_report = c(RoomReport, "cleaning/room_report")
        self._start = c(StartCleaning, "cleaning/start")
        self._pause = c(Trigger, "cleaning/pause")
        self._resume = c(Trigger, "cleaning/resume")
        self._set_settings = c(SetRobotSettings, "set_robot_settings")
        self._find_rooms = c(FindRooms, "rooms/find")
        self._set_rooms = c(SetRooms, "rooms/set")
        self._find_objects = c(FindObjects, "perception/find")
        self._navigate = ActionClient(node, NavigateToPose, "navigate_to_pose", callback_group=self._group)
        self._route = ActionClient(node, ComputePathToPose, "compute_path_to_pose", callback_group=self._group)
        self._spin = ActionClient(node, Spin, "spin", callback_group=self._group)
        self.tf = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self.tf, node)

    # ------------------------------------------------------------------ plumbing
    def _wait(self, future, timeout: float):
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            return None
        return future.result()

    def _call(self, client, request, timeout: float = CALL_TIMEOUT, what: str = ""):
        if not client.wait_for_service(timeout_sec=2.0):
            raise RobotError(f"{what or client.srv_name} is not running")
        result = self._wait(client.call_async(request), timeout)
        if result is None:
            raise RobotError(f"{what or client.srv_name} did not answer in time")
        return result

    @property
    def mode(self) -> str:
        return self.robot_state.state if self.robot_state is not None else ""

    @property
    def map_name(self) -> str:
        s = self.robot_state
        return "" if s is None or s.making_map else s.current_map

    @property
    def memory(self) -> Memory:
        """What has been seen on the map in use (kept next to the map)."""
        if self._memory is None or self._memory_map != self.map_name:
            path = map_files.things_path(self.maps_dir, self.map_name) if self.maps_dir and self.map_name else ""
            self._memory, self._memory_map = Memory(path), self.map_name
        return self._memory

    def remember(self, detections) -> None:
        if not self.map_name:
            return
        added = False
        for d in detections:
            if d.located and d.score >= SURE:
                self.memory.add(d.label, d.x, d.y, float(d.score))
                added = True
        if added:
            try:
                self.memory.save()
            except Exception as exc:
                self.node.get_logger().warn(f"could not save what was seen: {exc!r}")

    def pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            t = self.tf.lookup_transform("map", "base_link", Time())
        except TransformException:
            return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def room_list(self):
        return list(self.rooms.rooms) if self.rooms is not None and self.rooms.map_name == self.map_name else []

    def room_at(self, x: float, y: float) -> str:
        for r in self.room_list():
            if inside([(p.x, p.y) for p in r.outline.points], x, y):
                return r.name
        return ""

    def room_named(self, name: str):
        wanted = " ".join(name.split()).lower()
        for r in self.room_list():
            if " ".join(r.name.split()).lower() == wanted:
                return r
        return None

    def change_mode(self, mode: str) -> Tuple[bool, str]:
        response = self._call(self._change_state, ChangeState.Request(new_state=mode), MODE_TIMEOUT, "the mode manager")
        return bool(response.success), response.message

    # ------------------------------------------------------------------ reading
    def status(self) -> dict:
        out = describe.status(self.robot_state, self.cleaning, self.settings)
        pose = self.pose()
        if pose is not None and not (self.robot_state is not None and self.robot_state.making_map):
            room = self.room_at(pose[0], pose[1])
            out["in_room"] = room or ("not in a named room" if self.room_list() else "no rooms named yet")
        return out

    def history(self, count: int) -> dict:
        result = self._call(self._history, ListCleaningHistory.Request(max_records=count), what="the cleaning node")
        return describe.history(result.records, result.totals, self.map_name or None)

    def rooms_report(self) -> dict:
        if not self.map_name:
            raise RobotError("there is no saved map in use")
        if not self.room_list():
            return {"rooms": [], "note": "no rooms are named on this map yet: they can be found and named (learn_rooms)"}
        result = self._call(self._room_report, RoomReport.Request(), what="the cleaning node")
        if not result.success:
            raise RobotError(result.message)
        areas = {r.name: r.area for r in self.room_list()}
        rooms = []
        for r in result.rooms:
            rooms.append({"name": r.name, "floor_m2": round(areas.get(r.name, r.floor_area), 1),
                          "last_cleaned": r.last_cleaned, "share_of_room_cleaned_then_percent": round(r.last_share),
                          "last_passed_through": r.last_touched, "cleans_in_last_7_days": int(r.cleans_last_week)})
        out = {"rooms": rooms}
        pose = self.pose()
        if pose is not None:
            out["robot_is_in"] = self.room_at(pose[0], pose[1]) or "none of them"
        return out

    # ------------------------------------------------------------------ eyes
    def picture(self, timeout: float = 3.0) -> Optional[bytes]:
        """A fresh picture from the front camera, as JPEG."""
        import cv2
        import numpy as np

        got = []
        since = self.node.get_clock().now()
        sub = self.node.create_subscription(Image, "camera/image_raw", got.append, PICTURE_QOS, callback_group=self._group)
        try:
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                fresh = [m for m in list(got) if Time.from_msg(m.header.stamp) >= since]
                if fresh:
                    m = fresh[-1]
                    if m.encoding not in ("rgb8", "bgr8"):
                        raise RobotError(f"the camera sends {m.encoding} pictures")
                    a = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(m.height, m.step)[:, :m.width * 3]
                    a = a.reshape(m.height, m.width, 3)
                    bgr = a[:, :, ::-1] if m.encoding == "rgb8" else a
                    ok, data = cv2.imencode(".jpg", np.ascontiguousarray(bgr), [cv2.IMWRITE_JPEG_QUALITY, 80])
                    return data.tobytes() if ok else None
                time.sleep(0.03)
        finally:
            self.node.destroy_subscription(sub)
        return None

    def where(self) -> dict:
        """Where the robot is and which way it faces, in words the model can use."""
        pose = self.pose()
        if pose is None:
            return {"place": "unknown"}
        out = {"x": round(pose[0], 2), "y": round(pose[1], 2), "heading_degrees": round(math.degrees(pose[2]))}
        room = self.room_at(pose[0], pose[1])
        if room:
            out["room"] = room
        return out

    def _found(self, d) -> dict:
        return {"what": d.label, "sure_percent": round(100 * d.score), "x": round(d.x, 2), "y": round(d.y, 2),
                "distance_m": round(d.distance, 1), "direction": _direction(d.bearing), "room": self.room_at(d.x, d.y)}

    def look_for(self, name: str) -> Tuple[Optional[dict], bytes]:
        """`name` in what the camera sees now, checked: (where it is, or None; the marked picture)."""
        result = self._call(self._find_objects, FindObjects.Request(names=[name], min_score=MAYBE), 60.0, "the object finder")
        if not result.success:
            raise RobotError(result.message)
        picture = bytes(bytearray(result.picture_jpeg))
        self.remember(result.detections)
        for d in [d for d in result.detections if d.located][:2]:  # the two likeliest
            box = "the box from ({:.0f}, {:.0f}) to ({:.0f}, {:.0f}) in pixels, labelled '{} {:.2f}'".format(*d.box, d.label, d.score)
            sure = self.verify(picture, name, box) if self.verify is not None else d.score >= SURE
            if sure:
                self.memory.add(name, d.x, d.y, max(float(d.score), 0.9))  # checked: worth more than a guess
                try:
                    self.memory.save()
                except Exception:
                    pass
                return self._found(d), picture
        return None, picture

    def find(self, name: str) -> Tuple[Optional[dict], bytes, str]:
        """Look for `name` in what the camera sees now: (where it is or None, the picture, a note)."""
        found, picture = self.look_for(name)
        return found, picture, "" if found else "not seen in front"

    def remembered(self, name: str) -> Optional[dict]:
        """Where `name` was seen before on this map, if it was."""
        t = self.memory.best(name, SURE)
        if t is None:
            return None
        return {"what": t.name, "x": round(t.x, 2), "y": round(t.y, 2), "sure_percent": round(100 * min(t.score, 1.0)),
                "room": self.room_at(t.x, t.y), "seen": t.seen}

    def find_around(self, name: str) -> Tuple[Optional[dict], bytes]:
        """Look for `name` in front, then turning on the spot in steps all the way round."""
        found, picture = self.look_for(name)
        if found is not None:
            return found, picture
        self.ready_to_drive()
        step = 2.0 * math.pi / LOOK_STEPS
        for i in range(1, LOOK_STEPS):
            self.doing(f"looking around for the {name} ({i} of {LOOK_STEPS - 1})")
            self.turn(step)
            found, picture = self.look_for(name)
            if found is not None:
                return found, picture
        return None, picture

    def viewpoints(self, limit: int = VIEWPOINTS) -> List[Tuple[float, float]]:
        """Other places in the room the robot is in to look from: spread out, each far from here and
        from the others, each one the robot can drive to."""
        pose = self.pose()
        if pose is None:
            return []
        room = self.room_at(pose[0], pose[1])
        r = self.room_named(room) if room else None
        if r is None:
            return []
        outline = [(p.x, p.y) for p in r.outline.points]
        xs, ys = [p[0] for p in outline], [p[1] for p in outline]
        grid = []
        x = min(xs) + VIEW_GRID / 2.0
        while x < max(xs):
            y = min(ys) + VIEW_GRID / 2.0
            while y < max(ys):
                if inside(outline, x, y):
                    grid.append((x, y))
                y += VIEW_GRID
            x += VIEW_GRID
        chosen, here = [], [(pose[0], pose[1])]
        while grid and len(chosen) < limit:
            best = max(grid, key=lambda p: min(math.dist(p, q) for q in here + chosen))
            grid.remove(best)
            if min(math.dist(best, q) for q in here + chosen) < 1.0:
                break
            if self.route_exists(best[0], best[1], 0.0):
                chosen.append(best)
        return chosen

    def search(self, name: str) -> Tuple[Optional[dict], bytes]:
        """Look all round from here, then from other places in this room."""
        found, picture = self.find_around(name)
        if found is not None:
            return found, picture
        for i, (x, y) in enumerate(self.viewpoints(), start=1):
            outcome = self.drive_to(x, y, 0.0, f"moving to look for the {name} from another place ({i})")
            if outcome != "arrived":
                if self.mode != "navigation":
                    raise RobotError(outcome)
                continue
            found, picture = self.find_around(name)
            if found is not None:
                return found, picture
        return None, picture

    # ------------------------------------------------------------------ driving
    def ready_to_drive(self) -> None:
        """Navigation mode, asked for if needed; RobotError with the reason if it cannot be had."""
        mode = self.mode
        if mode == "navigation" and not self.robot_state.transitioning:
            return
        if mode == "cleaning":
            raise RobotError("I'm cleaning: stop or finish the clean first")
        if mode in ("mapping", "exploring") or (self.robot_state is not None and self.robot_state.making_map):
            raise RobotError("I'm making a map: save it first")
        if mode == "failsafe":
            raise RobotError("I'm stopped because something went wrong; reset me in the app first")
        if mode not in _CAN_DRIVE_FROM:
            raise RobotError(f"I can't drive from {mode or 'this'} mode")
        if not self.map_name:
            raise RobotError("there is no saved map to drive on")
        self.doing("getting ready to drive")
        ok, message = self.change_mode("navigation")
        if not ok:
            raise RobotError(f"I could not get ready to drive: {message}")
        end = time.monotonic() + 30.0
        while time.monotonic() < end:
            if self.mode == "navigation" and not self.robot_state.transitioning and self._navigate.server_is_ready():
                return
            time.sleep(0.1)
        raise RobotError("navigation did not start")

    def _goal(self, x: float, y: float, yaw: float) -> PoseStamped:
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x, goal.pose.position.y = float(x), float(y)
        goal.pose.orientation.z, goal.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        return goal

    def route_exists(self, x: float, y: float, yaw: float) -> bool:
        if not self._route.wait_for_server(timeout_sec=3.0):
            return True  # cannot ask: let navigation try
        handle = self._wait(self._route.send_goal_async(ComputePathToPose.Goal(goal=self._goal(x, y, yaw), use_start=False)), 5.0)
        if handle is None or not handle.accepted:
            return False
        outcome = self._wait(handle.get_result_async(), 10.0)
        return outcome is not None and outcome.status == GoalStatus.STATUS_SUCCEEDED and len(outcome.result.path.poses) > 0

    def drive_to(self, x: float, y: float, yaw: float, saying: str) -> str:
        """Drive to (x, y) facing `yaw` and wait: "arrived", or why not (in words)."""
        self.ready_to_drive()
        if not self.route_exists(x, y, yaw):
            return "no way there: it is blocked, or in a no-go zone"
        self.doing(saying)
        handle = self._wait(self._navigate.send_goal_async(NavigateToPose.Goal(pose=self._goal(x, y, yaw)), self._feedback(saying)), 10.0)
        if handle is None or not handle.accepted:
            return "navigation refused the trip"
        result = handle.get_result_async()
        end = time.monotonic() + TRIP_TIMEOUT
        while time.monotonic() < end and not result.done():
            if self.leaving_navigation():  # the user pressed Stop, or something went wrong
                handle.cancel_goal_async()
                return f"stopped on the way (the robot is now {self.mode or 'not answering'})"
            time.sleep(0.1)
        if not result.done():
            handle.cancel_goal_async()
            return "it took too long, so I stopped"
        status = result.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            return "arrived"
        end = time.monotonic() + 1.0  # a Stop ends the trip a moment before the mode change is published
        while time.monotonic() < end:
            if self.leaving_navigation():
                return "stopped on the way: someone pressed Stop, or the robot changed mode"
            time.sleep(0.05)
        if status == GoalStatus.STATUS_CANCELED:
            return "the trip was called off"
        return "I could not get there: the way was blocked"

    def leaving_navigation(self) -> bool:
        s = self.robot_state
        return s is None or s.state != "navigation" or bool(s.transitioning)

    def _feedback(self, saying: str):
        last = [0.0]

        def on(msg):
            now = time.monotonic()
            if now - last[0] >= 1.0:
                last[0] = now
                self.doing(f"{saying}, {msg.feedback.distance_remaining:.1f} m to go")
        return on

    def turn(self, angle: float) -> None:
        """Turn on the spot (radians, left positive) and wait."""
        if not self._spin.wait_for_server(timeout_sec=3.0):
            raise RobotError("turning on the spot is not available")
        goal = Spin.Goal(target_yaw=float(angle))
        goal.time_allowance.sec = 20
        handle = self._wait(self._spin.send_goal_async(goal), 5.0)
        if handle is None or not handle.accepted:
            raise RobotError("the robot would not turn")
        if self._wait(handle.get_result_async(), 25.0) is None:
            handle.cancel_goal_async()
            raise RobotError("turning took too long")
        if self.mode != "navigation":
            raise RobotError(f"stopped (the robot is now {self.mode})")

    def go_to_room(self, name: str) -> dict:
        room = self.room_named(name)
        if room is None:
            known = [r.name for r in self.room_list()]
            raise RobotError(f"there is no room called '{name}'" + (f"; the rooms are {', '.join(known)}" if known
                                                                     else "; no rooms are named yet"))
        pose = self.pose()
        yaw = math.atan2(room.y - pose[1], room.x - pose[0]) if pose is not None else 0.0
        outcome = self.drive_to(room.x, room.y, yaw, f"going to the {room.name.lower()}")
        return {"room": room.name, "outcome": outcome, "x": round(room.x, 2), "y": round(room.y, 2)}

    def approach_points(self, x: float, y: float, gap: float = NEAR) -> List[Tuple[float, float, float]]:
        """Places to stop near a thing at (x, y), best first, each facing it: on the line from the
        robot, then a little farther, then around it."""
        pose = self.pose() or (0.0, 0.0, 0.0)
        toward = math.atan2(y - pose[1], x - pose[0])
        out = []
        for gap in (gap, gap + 0.15, gap + 0.35):
            for turn in (0.0, 0.5, -0.5, 1.0, -1.0):
                a = toward + math.pi + turn  # from the thing back toward the robot, turned
                out.append((x + gap * math.cos(a), y + gap * math.sin(a), a + math.pi))
        return out

    def go_near(self, name: str) -> Tuple[dict, bytes]:
        """In view, go to it. Else, if it was seen before on this map, go there and look again. Else
        search: all round from here, then from other places in the room."""
        self.ready_to_drive()
        self.doing(f"looking for the {name}")
        found, picture = self.look_for(name)
        if found is None:
            before = self.remembered(name)
            if before is not None:
                for gx, gy, gyaw in self.approach_points(before["x"], before["y"], gap=1.0):
                    if self.route_exists(gx, gy, gyaw):
                        outcome = self.drive_to(gx, gy, gyaw, f"going to where I saw a {name}")
                        if outcome != "arrived" and self.mode != "navigation":
                            return {"found": False, "outcome": outcome}, picture
                        break
                found, picture = self.find_around(name)
        if found is None:
            found, picture = self.search(name)
        if found is None:
            return {"found": False, "outcome": f"I looked all round this room and could not find a {name}"}, picture
        # From afar the depth camera may see only part of it (at the edge of the picture, or behind
        # legs), so first stop about a metre short, look again, and close in on that closer sighting.
        if found.get("distance_m") is not None and found["distance_m"] > CLOSE_IN:
            self.doing(f"going closer to the {name}")
            first = self._drive_near(found, NEAR + 0.6, f"going toward the {name}")
            if first is None:
                return {"found": True, "thing": found, "outcome": "I found it but there is no way to get near it"}, picture
            if first != "arrived":
                return {"found": True, "thing": found, "outcome": first}, picture
            closer, closer_picture = self.look_for(name)
            if closer is not None:
                found, picture = closer, closer_picture
        self.doing(f"working out a way to the {name}")
        outcome = self._drive_near(found, NEAR, f"going to the {name}")
        if outcome is None:
            return {"found": True, "thing": found, "outcome": "I found it but there is no way to get near it"}, picture
        after = self.picture() or picture
        return {"found": True, "thing": found, "outcome": outcome}, after

    def _drive_near(self, thing: dict, gap: float, saying: str) -> Optional[str]:
        """Drive to the first reachable place `gap` from the thing, facing it; None if none is reachable."""
        for gx, gy, gyaw in self.approach_points(thing["x"], thing["y"], gap=gap):
            if self.route_exists(gx, gy, gyaw):
                return self.drive_to(gx, gy, gyaw, saying)
        return None

    # ------------------------------------------------------------------ cleaning and modes
    def start_cleaning(self, target: str, passes: int) -> dict:
        job = CleaningJob(passes=max(1, min(2, int(passes))), return_to_dock=CleaningJob.OPTION_DEFAULT)
        if target.strip().lower() in ("everything", "everywhere", "all", "the whole home", "whole home", "home", "house"):
            job.what = CleaningJob.EVERYTHING
        else:
            room = self.room_named(target)
            if room is None:
                known = [r.name for r in self.room_list()]
                raise RobotError(f"there is no room called '{target}'" + (f"; the rooms are {', '.join(known)}" if known
                                                                           else "; no rooms are named yet"))
            job.what, job.room = CleaningJob.ROOM, room.name
        response = self._call(self._start, StartCleaning.Request(job=job), 30.0, "the cleaning node")
        if not response.accepted:
            return {"started": False, "why": response.message}
        e = response.estimate
        return {"started": True, "what": job.room or "everything", "rows": int(e.rows), "minutes": round(e.time / 60.0),
                "floor_m2": round(e.planned_area, 1), "note": response.message, "then": "back to the dock"}

    def pause(self, resume: bool) -> dict:
        response = self._call(self._resume if resume else self._pause, Trigger.Request(), what="the cleaning node")
        return {"done": bool(response.success), "message": response.message}

    def stop(self) -> dict:
        ok, message = self.change_mode("idle")
        return {"done": ok, "message": message}

    def dock(self) -> dict:
        ok, message = self.change_mode("docking")
        return {"done": ok, "message": message, "note": "on the way to the dock" if ok else ""}

    def change_settings(self, changes: dict) -> dict:
        s = self.settings
        if s is None:
            raise RobotError("the settings are not known yet")
        wanted = RobotSettings(cleaning_speed=s.cleaning_speed, manual_speed=s.manual_speed, row_spacing=s.row_spacing,
                               row_angle_auto=s.row_angle_auto, row_angle=s.row_angle)
        if "cleaning_speed" in changes:
            wanted.cleaning_speed = float(changes["cleaning_speed"])
        if "joystick_speed" in changes:
            wanted.manual_speed = float(changes["joystick_speed"])
        if "row_spacing_cm" in changes:
            wanted.row_spacing = float(changes["row_spacing_cm"]) / 100.0
        if "row_direction" in changes:
            text = str(changes["row_direction"]).strip().lower()
            if text == "auto":
                wanted.row_angle_auto = True
            else:
                try:
                    angle = float(text.rstrip("°").replace("degrees", "").strip()) % 180.0
                except ValueError:
                    raise RobotError("the row direction is 'auto' or an angle in degrees") from None
                wanted.row_angle_auto, wanted.row_angle = False, round(angle / 5.0) * 5.0 % 180.0
        response = self._call(self._set_settings, SetRobotSettings.Request(settings=wanted), what="the settings node")
        return {"done": bool(response.success), "message": response.message}

    # ------------------------------------------------------------------ rooms
    def find_rooms(self):
        result = self._call(self._find_rooms, FindRooms.Request(), 30.0, "the rooms node")
        if not result.success:
            raise RobotError(result.message)
        return list(result.rooms)

    def save_rooms(self, rooms) -> str:
        result = self._call(self._set_rooms, SetRooms.Request(map_name=self.map_name, rooms=list(rooms)), what="the rooms node")
        if not result.success:
            raise RobotError(result.message)
        return result.message

    def look_in_room(self, room, views: int = 4, things: Sequence[str] = ()) -> Tuple[str, List[bytes], dict]:
        """Drive to a room's spot facing its middle, take `views` pictures turning on the spot, and
        look for `things` in each: (outcome, pictures, {thing: best score seen inside the room})."""
        outline = [(p.x, p.y) for p in room.outline.points]
        mx = sum(p[0] for p in outline) / len(outline)
        my = sum(p[1] for p in outline) / len(outline)
        outcome = self.drive_to(room.x, room.y, math.atan2(my - room.y, mx - room.x), f"going to {room.name}")
        if outcome != "arrived":
            return outcome, [], {}
        pictures, seen = [], {}
        for i in range(views):
            if i:
                self.turn(2.0 * math.pi / views)
            picture = self.picture()
            if picture:
                pictures.append(picture)
            if things:
                detections = self.detect(things)
                self.remember(detections)
                for d in detections:
                    if d.located and inside(outline, d.x, d.y):
                        seen[d.label] = max(seen.get(d.label, 0.0), round(float(d.score), 2))
        return outcome, pictures, seen

    def detect(self, things: Sequence[str], min_score: float = SURE):
        """What the object finder sees now of `things` (its Detection messages); none if it is not running."""
        try:
            result = self._call(self._find_objects, FindObjects.Request(names=list(things), min_score=min_score), 60.0, "the object finder")
        except RobotError:
            return []
        return list(result.detections) if result.success else []

    def look_around(self, views: int = 4) -> List[bytes]:
        """Pictures turning on the spot, the first one straight ahead."""
        self.ready_to_drive()
        pictures = []
        for i in range(views):
            if i:
                self.doing(f"looking around ({i + 1} of {views})")
                self.turn(2.0 * math.pi / views)
            picture = self.picture()
            if picture:
                pictures.append(picture)
        return pictures


def nearest_first(points: Sequence[Tuple[float, float]], start: Optional[Tuple[float, float, float]]) -> List[int]:
    """An order to visit `points` in: always the nearest one left, from `start` (or the first)."""
    left = list(range(len(points)))
    here = (start[0], start[1]) if start is not None else (points[0] if points else (0.0, 0.0))
    order = []
    while left:
        i = min(left, key=lambda k: (math.dist(here, points[k]), k))
        order.append(i)
        left.remove(i)
        here = points[i]
    return order


def _direction(bearing: float) -> str:
    d = math.degrees(bearing)
    if abs(d) <= 15:
        return "straight ahead"
    side = "left" if d > 0 else "right"
    if abs(d) <= 60:
        return f"ahead to the {side}"
    if abs(d) <= 120:
        return f"to the {side}"
    return f"behind, to the {side}"

"""The exploration node: the robot maps a house by itself.

It owns the explorer (frontier_exploration_ros2, used unchanged): a third-party program that picks
the next place to look at and sends the robot there through Nav2. This node starts it as a child
program, one per exploration (and one more for the second look), watches it, ends it, judges when
exploring is really finished, brings the robot back to where it started and reports progress.
The course of an exploration is in runner.py; starting and ending the program in process.py.

It never drives the robot itself apart from the one trip back, and never changes the mode except
by asking the mode manager: mapping (from idle or docked: a new map), then exploring, and mapping
again at the end, so the map is always kept. A Stop (a request for idle) is carried out by the
mode manager as a change to mapping; this node then sees the robot leave exploring and ends the
exploration as stopped.

Interfaces:
  /exploration/start      vacuum_interfaces/srv/StartExploration   start one; answered at once
  /exploration/status     vacuum_interfaces/msg/ExplorationStatus  latched, on change and 5 times a second
  /exploration/frontiers  geometry_msgs/PoseArray                  the places still to look at; latched

The mode manager watches /exploration/status in exploring mode: silent for 1 s, or no job, is
failsafe. The status timer therefore runs on its own thread and never waits on anything.
"""

import math
import os
import threading
import time
import traceback
from typing import Optional, Tuple

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Pose, PoseArray
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Empty
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from frontier_exploration_ros2.srv import ControlExploration
from vacuum_interfaces.msg import ExplorationStatus, RobotState
from vacuum_interfaces.srv import ChangeState, StartExploration

from . import process
from . import runner as rn

EXPLORER_PACKAGE = "frontier_exploration_ros2"
EXPLORER_PROGRAM = "frontier_explorer"
# Every explorer this node starts has this in its command line; a starting node kills leftovers.
EXPLORER_MARKER = f"lib/{EXPLORER_PACKAGE}/{EXPLORER_PROGRAM}"
CHANGE_MODE_TIMEOUT = 250.0  # s: the mode manager's own worst case, plus margin
BUSY_RETRY = 10.0  # s to keep asking while the mode manager answers "busy"
COMPLETE_GRACE = 3.0  # s after an explorer starts in which a "finished" cannot be its own
PROBE_TIMEOUT = 3.0  # s after which an unanswered state request is given up and asked again
_ACTIVE_GOAL = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING)


class _Explorer:
    """One running explorer program, as the runner sees it (runner.Explorer)."""

    def __init__(self, child: process.Child):
        self.child = child
        self.started = time.monotonic()
        self.answered: Optional[float] = None  # monotonic time of its last answer to a state request
        self.said_finished = False
        self.how_it_ended = ""

    def exit_code(self) -> Optional[int]:
        return self.child.exit_code()

    def last_answer(self) -> Optional[float]:
        return self.answered

    def finished(self) -> bool:
        return self.said_finished

    def end(self) -> str:
        self.how_it_ended = self.child.end()
        return self.how_it_ended


class _Job:
    def __init__(self, job_id: int, runner: rn.ExplorationRunner, time_limit: float):
        self.id = job_id
        self.runner = runner
        self.time_limit = time_limit
        self.started = time.time()
        self.seen_exploring = False  # the robot has been seen in exploring mode during this job
        self.distance = 0.0
        self.goals_failed = 0
        self.done = False


class ExplorationNode(Node):
    def __init__(self):
        super().__init__("exploration_node")
        defaults = rn.Settings()
        self.declare_parameter("time_limit", defaults.time_limit)  # s of robot time
        self.declare_parameter("quiet_limit", defaults.quiet_limit)
        self.declare_parameter("answer_timeout", defaults.answer_timeout)
        self.declare_parameter("startup_timeout", defaults.startup_timeout)
        self.declare_parameter("nav_ready_timeout", 60.0)  # s for Nav2 and slam_toolbox to be up in exploring
        self.declare_parameter("return_timeout", 240.0)  # s (wall) for the trip back
        self.declare_parameter("status_rate", 5.0)
        self.declare_parameter("explorer_params", "")  # the explorer's settings file (exploration.yaml)
        self.declare_parameter("explorer_log", os.path.expanduser("~/.ros/log/frontier_explorer.log"))

        self._explorer_params = self.get_parameter("explorer_params").value
        if not os.path.isfile(self._explorer_params):
            raise RuntimeError(f"parameter explorer_params must name the explorer's settings file (got '{self._explorer_params}')")
        self._explorer_program = os.path.join(get_package_prefix(EXPLORER_PACKAGE), "lib", EXPLORER_PACKAGE, EXPLORER_PROGRAM)
        if not os.access(self._explorer_program, os.X_OK):
            raise RuntimeError(f"the explorer program is missing: {self._explorer_program}")
        self._explorer_log = self.get_parameter("explorer_log").value
        leftovers = process.kill_leftovers(EXPLORER_MARKER)
        if leftovers:
            self.get_logger().warn(f"killed explorer program(s) left over from before: {leftovers}")
        self._launcher = process.Launcher()  # its thread lives as long as this node: see process.py

        self._lock = threading.RLock()
        self._job: Optional[_Job] = None  # the exploration running, else None
        self._last: Optional[_Job] = None  # the most recent one, running or ended
        self._job_counter = 0
        self._explorer: Optional[_Explorer] = None
        self._robot_state: Optional[RobotState] = None
        self._mapped_area = 0.0
        self._places_left = 0
        self._last_odom: Optional[Tuple[float, float]] = None
        self._costmap_at = {"local": 0.0, "global": 0.0}  # monotonic time of the last costmap of each
        self._map_odom_at = 0.0  # monotonic time of the last map -> odom transform
        self._goals_seen = set()  # navigate_to_pose goal ids seen under way, not this node's
        self._goals_failed = set()
        self._goals_active = set()
        self._own_goals = set()
        self._probe_sent = 0.0  # monotonic time of the state request waiting for an answer; 0: none

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        clients = ReentrantCallbackGroup()
        data = MutuallyExclusiveCallbackGroup()

        self._status_pub = self.create_publisher(ExplorationStatus, "exploration/status", latched)
        self._frontiers_pub = self.create_publisher(PoseArray, "exploration/frontiers", latched)

        self.create_subscription(RobotState, "robot_state", self._on_robot_state, latched, callback_group=data)
        self.create_subscription(OccupancyGrid, "map", self._on_map, latched, callback_group=data)
        self.create_subscription(Odometry, "odom", self._on_odom, qos_profile_sensor_data, callback_group=data)
        self.create_subscription(TFMessage, "tf", self._on_tf, 100, callback_group=data)
        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(GoalStatusArray, "navigate_to_pose/_action/status", self._on_nav_status, status_qos,
                                 callback_group=data)
        # Not latched: only costmaps published since exploring mode was entered say Nav2 is up.
        self.create_subscription(OccupancyGrid, "local_costmap/costmap", lambda _m: self._costmap("local"), 1,
                                 callback_group=data)
        self.create_subscription(OccupancyGrid, "global_costmap/costmap", lambda _m: self._costmap("global"), 1,
                                 callback_group=data)
        # The explorer's "finished" is latched; this listens without asking for old copies, so a
        # message from an earlier explorer can never count for the one running now.
        self.create_subscription(Empty, "exploration_complete", self._on_complete,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE), callback_group=data)
        self.create_subscription(MarkerArray, "explore/frontiers", self._on_frontiers, 10, callback_group=data)

        self._change_state = self.create_client(ChangeState, "change_state", callback_group=clients)
        self._control = self.create_client(ControlExploration, "control_exploration", callback_group=clients)
        self._cancel_nav = self.create_client(CancelGoal, "navigate_to_pose/_action/cancel_goal", callback_group=clients)
        self._navigate = ActionClient(self, NavigateToPose, "navigate_to_pose", callback_group=clients)
        self.tf = Buffer()
        self._tf_listener = TransformListener(self.tf, self)

        self.create_service(StartExploration, "exploration/start", self._on_start, callback_group=ReentrantCallbackGroup())

        steady = Clock(clock_type=ClockType.STEADY_TIME)
        rate = float(self.get_parameter("status_rate").value)
        self.create_timer(1.0 / rate, self._publish_status, callback_group=MutuallyExclusiveCallbackGroup(), clock=steady)
        self.create_timer(1.0, self._probe_explorer, callback_group=MutuallyExclusiveCallbackGroup(), clock=steady)
        self._publish_status()
        self._publish_frontiers([])
        self.get_logger().info(f"exploration node started; the explorer's output goes to {self._explorer_log}")

    # ================================================================== data in
    def _on_robot_state(self, msg: RobotState) -> None:
        self._robot_state = msg
        with self._lock:
            job = self._job
        if job is None:
            return
        runner = job.runner
        if msg.state == rn.EXPLORING_MODE:
            job.seen_exploring = True
            return
        # A copy from before the switch may still arrive: only a change seen after exploring mode
        # was seen ends the job. While it is ending, the change is the job's own.
        if not job.seen_exploring or runner.state in (rn.ENDING,) + (rn.FINISHED, rn.STOPPED, rn.FAILED):
            return
        if msg.state == "failsafe":
            runner.stop(f"the robot went to failsafe ({msg.reason})", failed=True)
        else:
            runner.stop(f"the robot left exploring mode (now {msg.state}: {msg.reason})")

    def _on_map(self, msg: OccupancyGrid) -> None:
        cells = np.asarray(msg.data, dtype=np.int16)
        free = int(np.count_nonzero((cells >= 0) & (cells < 50)))
        self._mapped_area = free * msg.info.resolution ** 2

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        with self._lock:
            job = self._job
            if job is not None and self._last_odom is not None and job.runner.state in (rn.EXPLORING, rn.RETURNING):
                job.distance += math.hypot(p.x - self._last_odom[0], p.y - self._last_odom[1])
            self._last_odom = (p.x, p.y)

    def _on_tf(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            if t.header.frame_id == "map" and t.child_frame_id == "odom":
                self._map_odom_at = time.monotonic()

    def _costmap(self, which: str) -> None:
        self._costmap_at[which] = time.monotonic()

    def _on_nav_status(self, msg: GoalStatusArray) -> None:
        with self._lock:
            active = set()
            for s in msg.status_list:
                goal = bytes(bytearray(s.goal_info.goal_id.uuid))
                if goal in self._own_goals:
                    continue
                if s.status in _ACTIVE_GOAL:
                    active.add(goal)
                    self._goals_seen.add(goal)
                elif s.status == GoalStatus.STATUS_ABORTED and goal in self._goals_seen and goal not in self._goals_failed:
                    self._goals_failed.add(goal)
                    if self._job is not None:
                        self._job.goals_failed += 1
            self._goals_active = active

    def _on_complete(self, _msg: Empty) -> None:
        with self._lock:
            explorer = self._explorer
        if explorer is None:
            return
        # An explorer cannot finish this early: it measures the map's rate over four maps (2 s
        # apart) before its first search. A message this early was the explorer before it, still
        # on its way; counting it would end the second look before it had looked.
        if time.monotonic() - explorer.started < COMPLETE_GRACE:
            self.get_logger().info("ignored a 'finished' that arrived just after a new explorer started")
            return
        explorer.said_finished = True
        self.get_logger().info("the explorer says it has finished")

    def _on_frontiers(self, msg: MarkerArray) -> None:
        points = []
        for m in msg.markers:
            if m.action == Marker.ADD and m.type == Marker.POINTS:
                points.extend((p.x, p.y) for p in m.points)
        with self._lock:
            running = self._job is not None and self._explorer is not None
        if running:
            self._places_left = len(points)
            self._publish_frontiers(points)

    # ================================================================== the explorer
    def _probe_explorer(self) -> None:
        """Once a second: ask the running explorer for its state (action 0 changes nothing). Its
        answer is the sign of life the runner's watchdog uses."""
        with self._lock:
            explorer = self._explorer
        now = time.monotonic()
        if explorer is None or explorer.exit_code() is not None or now - self._probe_sent < PROBE_TIMEOUT:
            return
        if not self._control.service_is_ready():
            return
        self._probe_sent = now

        def answered(future, explorer=explorer):
            self._probe_sent = 0.0
            try:
                if future.result() is not None:
                    explorer.answered = time.monotonic()
            except Exception:
                pass

        self._control.call_async(ControlExploration.Request(action=0)).add_done_callback(answered)

    def _ask_explorer_to_stop(self) -> bool:
        """Ask the running explorer to stop and quit: it cancels its own goal first."""
        if not self._control.service_is_ready():
            return False
        done = threading.Event()
        future = self._control.call_async(ControlExploration.Request(action=ControlExploration.Request.ACTION_STOP,
                                                                     quit_after_stop=True))
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(2.0):
            return False
        try:
            return bool(future.result().accepted)
        except Exception:
            return False

    def start_explorer(self) -> _Explorer:
        args = [self._explorer_program, "--ros-args", "-r", f"__node:={EXPLORER_PROGRAM}",
                "--params-file", self._explorer_params, "-p", "use_sim_time:=true"]
        popen = self._launcher.start(args, self._explorer_log)
        explorer = _Explorer(process.Child(popen, self._ask_explorer_to_stop))
        with self._lock:
            self._explorer = explorer
            self._places_left = 0
        self.get_logger().info(f"explorer started (pid {popen.pid})")
        return explorer

    def explorer_ended(self, explorer: _Explorer) -> None:
        with self._lock:
            if self._explorer is explorer:
                self._explorer = None
        self.get_logger().info(f"explorer (pid {explorer.child.pid}) {explorer.how_it_ended}")

    # ================================================================== requests
    def _on_start(self, request, response):
        state = self._robot_state
        with self._lock:
            busy = self._job is not None
        if busy:
            why = "an exploration is already running: stop it first"
        elif state is None:
            why = "the robot's mode is not known yet (is the mode manager running?)"
        elif state.transitioning:
            why = "the robot is changing modes: try again in a moment"
        elif state.state not in rn.START_MODES:
            why = f"stop what the robot is doing first (it is in {state.state} mode)"
        else:
            why = ""
        if why:
            response.accepted, response.message = False, why
            self.get_logger().info(f"exploration refused: {why}")
            return response
        limit = float(request.time_limit) if request.time_limit > 0 else float(self.get_parameter("time_limit").value)
        settings = rn.Settings(
            time_limit=limit,
            quiet_limit=float(self.get_parameter("quiet_limit").value),
            answer_timeout=float(self.get_parameter("answer_timeout").value),
            startup_timeout=float(self.get_parameter("startup_timeout").value),
        )
        with self._lock:
            if self._job is not None:
                response.accepted, response.message = False, "an exploration is already running: stop it first"
                return response
            self._job_counter += 1
            job = _Job(self._job_counter, rn.ExplorationRunner(_RosWorld(self), settings), limit)
            self._job = self._last = job
        new_map = state.state in rn.NEW_MAP_MODES
        response.accepted, response.job_id = True, job.id
        response.message = "exploring: a new map begins" if new_map else "exploring: the map being made is continued"
        self.get_logger().info(f"exploration {job.id} accepted ({response.message}; time limit {limit / 60:.0f} min)")
        self._publish_status()
        threading.Thread(target=self._run_job, args=(job,), daemon=True).start()
        return response

    def _run_job(self, job: _Job) -> None:
        log = self.get_logger()
        try:
            outcome, message = job.runner.run()
        except Exception as exc:  # a fault here must still end the exploration and leave exploring
            log.error(f"exploration {job.id} failed in the exploration node: {exc!r}\n{traceback.format_exc()}")
            with self._lock:
                explorer = self._explorer
            if explorer is not None:
                explorer.end()
            state = self._robot_state
            if state is not None and state.state == rn.EXPLORING_MODE:
                self.ask_mode(rn.MAPPING_MODE)
            job.runner._set(rn.FAILED, f"failed: the exploration node failed: {exc}")
            outcome, message = rn.FAILED, str(exc)
        with self._lock:
            job.done = True
            if self._job is job:
                self._job = None
            self._places_left = 0
        self._publish_frontiers([])
        # Two calls: rclpy refuses to log one line of code at two different levels.
        if outcome == rn.FAILED:
            log.warn(f"exploration {job.id}: {message}")
        else:
            log.info(f"exploration {job.id}: {message}")
        self._publish_status()

    def ask_mode(self, mode: str) -> Tuple[bool, str]:
        """Ask the mode manager for a mode; returns (done, its message). It answers once the change
        has finished. "Busy" is asked again for a few seconds; a reply lost on a brand-new client
        (Humble) is asked again once."""
        if not self._change_state.wait_for_service(timeout_sec=5.0):
            return False, "the mode manager is not running (no /change_state service)"
        busy_until = time.monotonic() + BUSY_RETRY
        lost = 0
        while True:
            done = threading.Event()
            future = self._change_state.call_async(ChangeState.Request(new_state=mode))
            future.add_done_callback(lambda _f: done.set())
            if not done.wait(CHANGE_MODE_TIMEOUT if lost else 10.0):
                s = self._robot_state
                if s is not None and s.transitioning and not lost and done.wait(CHANGE_MODE_TIMEOUT):
                    response = future.result()
                    return bool(response.success), response.message
                lost += 1
                if lost >= 2:
                    return False, "no answer from the mode manager"
                continue
            response = future.result()
            if not response.success and response.message.startswith("busy") and time.monotonic() < busy_until:
                time.sleep(0.5)
                continue
            return bool(response.success), response.message

    # ================================================================== status out
    def _status(self, job: Optional[_Job]) -> ExplorationStatus:
        msg = ExplorationStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.mapped_area = float(self._mapped_area)
        if job is None:
            msg.state = ExplorationStatus.NONE
            return msg
        r = job.runner
        msg.state, msg.message = r.state, r.message
        msg.job_id = job.id
        msg.started = float(job.started)
        msg.places_left = self._places_left if not job.done else 0
        msg.exploring_time = float(r.exploring_time)
        msg.time_limit = float(job.time_limit)
        msg.distance = float(job.distance)
        msg.goals_sent = max(0, r.goals_sent)
        msg.goals_failed = job.goals_failed
        return msg

    def _publish_status(self) -> None:
        with self._lock:
            job = self._job or self._last
        self._status_pub.publish(self._status(job))

    def _publish_frontiers(self, points) -> None:
        msg = PoseArray()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in points:
            pose = Pose()
            pose.position.x, pose.position.y = float(x), float(y)
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        self._frontiers_pub.publish(msg)

    def shut_down(self) -> None:
        with self._lock:
            job, explorer = self._job, self._explorer
        if job is not None:
            job.runner.stop("the exploration node is shutting down")
        if explorer is not None:
            explorer.end()


class _RosWorld:
    """The runner's world (runner.World) on the real robot."""

    def __init__(self, node: ExplorationNode):
        self.node = node

    def mode(self) -> Tuple[str, bool]:
        s = self.node._robot_state
        return (s.state, s.transitioning) if s is not None else ("", False)

    def ask_mode(self, mode: str) -> Tuple[bool, str]:
        return self.node.ask_mode(mode)

    def wait_ready(self, abort: threading.Event) -> str:
        """Exploring mode settled, then slam_toolbox's own transform and both of Nav2's costmaps
        arriving after that, and Nav2's action answering. Before then the robot's place on the map
        may still be AMCL's last word, and the explorer would wait on a costmap."""
        node = self.node
        end = time.monotonic() + float(node.get_parameter("nav_ready_timeout").value)
        since = None
        while time.monotonic() < end:
            if abort.is_set():
                return "stopped while waiting for navigation to start"
            s = node._robot_state
            if since is None:
                if s is not None and s.state == rn.EXPLORING_MODE and not s.transitioning:
                    since = time.monotonic()
            elif (node._map_odom_at > since and node._costmap_at["local"] > since and node._costmap_at["global"] > since
                  and node._navigate.server_is_ready()):
                time.sleep(1.0)  # one more costmap update: the obstacle layer fills from the next scans
                return ""
            time.sleep(0.1)
        missing = []
        if since is None:
            missing.append("exploring mode")
        else:
            if node._map_odom_at <= since:
                missing.append("slam_toolbox's map -> odom transform")
            for which in ("local", "global"):
                if node._costmap_at[which] <= since:
                    missing.append(f"the {which} costmap")
            if not node._navigate.server_is_ready():
                missing.append("Nav2's navigate_to_pose")
        return "navigation did not start in exploring mode (no " + ", ".join(missing) + ")"

    def pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            t = self.node.tf.lookup_transform("map", "base_link", Time())
        except TransformException:
            return None
        tr, q = t.transform.translation, t.transform.rotation
        return tr.x, tr.y, math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def now(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    def wall(self) -> float:
        return time.monotonic()

    def start_explorer(self):
        return _Handle(self.node, self.node.start_explorer())

    def goals_seen(self) -> int:
        with self.node._lock:
            return len(self.node._goals_seen)

    def goal_active(self) -> bool:
        with self.node._lock:
            return bool(self.node._goals_active)

    def navigate(self, x: float, y: float, yaw: float, abort: threading.Event) -> bool:
        node = self.node
        client = node._navigate
        if not client.wait_for_server(timeout_sec=5.0):
            return False
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"  # stamp zero: Nav2 uses the newest transform
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        answered = threading.Event()
        sent = client.send_goal_async(goal)
        sent.add_done_callback(lambda _f: answered.set())
        if not answered.wait(10.0):
            return False
        handle = sent.result()
        if handle is None or not handle.accepted:
            return False
        with node._lock:
            node._own_goals.add(bytes(bytearray(handle.goal_id.uuid)))
        finished = threading.Event()
        result = handle.get_result_async()
        result.add_done_callback(lambda _f: finished.set())
        end = time.monotonic() + float(node.get_parameter("return_timeout").value)
        while not finished.wait(0.1):
            if abort.is_set() or time.monotonic() > end:
                cancelled = threading.Event()
                handle.cancel_goal_async().add_done_callback(lambda _f: cancelled.set())
                cancelled.wait(2.0)
                return False
        wrapped = result.result()
        return wrapped is not None and wrapped.status == GoalStatus.STATUS_SUCCEEDED

    def cancel_goals(self) -> None:
        client = self.node._cancel_nav
        if not client.service_is_ready():
            return
        done = threading.Event()
        client.call_async(CancelGoal.Request()).add_done_callback(lambda _f: done.set())
        done.wait(2.0)

    def sleep(self, seconds: float) -> None:
        time.sleep(seconds)


class _Handle:
    """runner.Explorer for one explorer program; tells the node when it has been ended."""

    def __init__(self, node: ExplorationNode, explorer: _Explorer):
        self.node = node
        self.explorer = explorer

    def exit_code(self):
        return self.explorer.exit_code()

    def last_answer(self):
        return self.explorer.last_answer()

    def finished(self):
        return self.explorer.finished()

    def end(self):
        how = self.explorer.end()
        self.node.explorer_ended(self.explorer)
        return how


def main():
    rclpy.init()
    node = ExplorationNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shut_down()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

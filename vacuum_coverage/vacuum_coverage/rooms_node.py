"""The rooms node: keeps the rooms of each saved map.

    /rooms         vacuum_interfaces/msg/Rooms         latched: the rooms of the map in use
    /rooms/find    vacuum_interfaces/srv/FindRooms     split the map in use into rooms (a proposal)
    /rooms/set     vacuum_interfaces/srv/SetRooms      save the rooms of the map in use

The rooms live next to the map (<maps dir>/<map>_rooms.yaml, see rooms.py). The cleaning node reads
them for a room clean; the assistant uses them to go to a room, clean it, and say how clean it is.
"""

import os
import threading
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Point32, Polygon
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from vacuum_interfaces.msg import Room as RoomMsg
from vacuum_interfaces.msg import Rooms, RobotState
from vacuum_interfaces.srv import FindRooms, SetRooms
from vacuum_mode_manager import map_files

from . import mapdata, rooms
from .rooms import Room

CHECK_PERIOD = 2.0  # s between looks at the rooms file of the map in use


def to_msg(room: Room) -> RoomMsg:
    return RoomMsg(name=room.name, outline=Polygon(points=[Point32(x=float(x), y=float(y), z=0.0) for x, y in room.outline]),
                   x=float(room.x), y=float(room.y), area=float(room.area))


def from_msg(msg) -> Room:
    return Room(name=" ".join(msg.name.split()), outline=[(float(p.x), float(p.y)) for p in msg.outline.points],
                x=float(msg.x), y=float(msg.y), area=float(msg.area))


class RoomsNode(Node):
    def __init__(self):
        super().__init__("rooms_node")
        self.declare_parameter("maps_dir", "")
        self._maps_dir = self.get_parameter("maps_dir").value
        if not self._maps_dir:
            raise RuntimeError("parameter maps_dir is required")
        self._lock = threading.Lock()
        self._map = ""  # the saved map in use ("" while a map is being made, or none)
        self._sent = None  # (map, file signature) last published
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(Rooms, "rooms", latched)
        self.create_subscription(RobotState, "robot_state", self._on_state, latched, callback_group=MutuallyExclusiveCallbackGroup())
        services = ReentrantCallbackGroup()
        self.create_service(FindRooms, "rooms/find", self._on_find, callback_group=services)
        self.create_service(SetRooms, "rooms/set", self._on_set, callback_group=services)
        self.create_timer(CHECK_PERIOD, self._publish, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self._publish()

    def _path(self, name: str) -> str:
        return map_files.rooms_path(self._maps_dir, name)

    def _on_state(self, msg: RobotState) -> None:
        name = "" if msg.making_map else msg.current_map
        if name != self._map:
            self._map = name
            self._publish()

    def _publish(self, force: bool = False) -> None:
        with self._lock:
            name = self._map
            path = self._path(name) if name else ""
            try:
                signature = (name, os.stat(path).st_mtime_ns) if path and os.path.exists(path) else (name, None)
            except OSError:
                signature = (name, None)
            if signature == self._sent and not force:
                return
            try:
                found = rooms.read(path) if path else []
            except Exception as exc:
                self.get_logger().error(f"cannot read the rooms of '{name}' ({path}): {exc}")
                found = []
            self._pub.publish(Rooms(map_name=name, rooms=[to_msg(r) for r in found]))
            self._sent = signature
            self.get_logger().info(f"rooms of {repr(name) if name else 'no saved map'}: {', '.join(r.name for r in found) or 'none'}")

    def _load_map(self, name: str) -> mapdata.MapData:
        yaml_path = os.path.join(self._maps_dir, name + ".yaml")
        return mapdata.load(yaml_path, map_files.read_zones(map_files.zones_path(self._maps_dir, name)))

    def _on_find(self, _request, response):
        name = self._map
        if not name:
            response.success, response.message = False, "there is no saved map in use: save a map first"
            return response
        try:
            m = self._load_map(name)
            found = rooms.find(m.grid, walls=m.cells == 100)
            try:
                saved = rooms.read(self._path(name))
            except Exception:
                saved = []
            found = rooms.keep_names(found, saved, m.grid)
        except Exception as exc:
            self.get_logger().error(f"finding the rooms of '{name}' failed: {exc!r}")
            response.success, response.message = False, f"could not find the rooms: {exc}"
            return response
        response.success = True
        response.message = f"found {len(found)} room{'s' if len(found) != 1 else ''}"
        response.rooms = [to_msg(r) for r in found]
        self.get_logger().info(f"rooms found on '{name}': {', '.join(f'{r.name} {r.area:.1f} m2' for r in found)}")
        return response

    def _on_set(self, request, response):
        name = request.map_name.strip()
        if not self._map:
            response.success, response.message = False, "there is no saved map in use"
            return response
        if name != self._map:
            response.success, response.message = False, f"those rooms are for the map '{name}', and the robot uses '{self._map}'"
            return response
        new = [from_msg(r) for r in request.rooms]
        why = rooms.problem(new)
        if why:
            response.success, response.message = False, why
            return response
        try:
            rooms.write(self._path(name), new)
        except Exception as exc:
            response.success, response.message = False, f"could not save the rooms: {exc}"
            return response
        self._publish(force=True)
        response.success = True
        response.message = f"saved the rooms of '{name}': {', '.join(r.name for r in new) or 'none'}"
        return response


def main():
    rclpy.init()
    node = RoomsNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

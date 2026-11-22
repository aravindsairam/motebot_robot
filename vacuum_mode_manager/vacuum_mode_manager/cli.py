"""Command-line helpers for the mode manager.

    ros2 run vacuum_mode_manager mode              show the current mode
    ros2 run vacuum_mode_manager mode mapping      request a mode
    ros2 run vacuum_mode_manager save_map house    save the map being built (mapping mode only)
    ros2 run vacuum_mode_manager save_map house --replace    ... overwriting a saved map of that name
    ros2 run vacuum_mode_manager zones             list the no-go zones of the map in use
    ros2 run vacuum_mode_manager zones add NAME X0 Y0 X1 Y1    add a rectangle (two opposite corners, map metres)
    ros2 run vacuum_mode_manager zones delete NAME             delete a zone
    ros2 run vacuum_mode_manager settings          show the settings (speeds, row spacing, row direction)
    ros2 run vacuum_mode_manager settings cleaning_speed=0.25 row_angle=45    change some of them
"""

import argparse
import dataclasses
import sys

import rclpy
from geometry_msgs.msg import Point32, Polygon
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from vacuum_interfaces.msg import NoGoZone, RobotState
from vacuum_interfaces.msg import RobotSettings as RobotSettingsMsg
from vacuum_interfaces.srv import ChangeState, SaveMap, SetNoGoZones, SetRobotSettings

from . import map_files, robot_settings, zone_rules


def _call(node, service_type, name, request, timeout):
    client = node.create_client(service_type, name)
    if not client.wait_for_service(timeout_sec=5.0):
        print(f"No {name} service: is the robot stack running?")
        return None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        print(f"No answer from {name} within {timeout:.0f} s.")
        return None
    return future.result()


def _robot_state(node):
    """The mode manager's latest /robot_state, or None (with a note) if none arrives within 5 s."""
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    box = []
    node.create_subscription(RobotState, "robot_state", box.append, latched)
    end = node.get_clock().now().nanoseconds + int(5e9)
    while not box and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not box:
        print("No /robot_state received: is the mode manager running?")
        return None
    return box[-1]


def _show_state(node) -> int:
    s = _robot_state(node)
    if s is None:
        return 1
    print(f"mode:        {s.state}" + ("  (changing)" if s.transitioning else ""))
    print(f"reason:      {s.reason}")
    print(f"map in use:  {s.current_map or 'none'}")
    print(f"can go to:   {', '.join(s.allowed_transitions) or 'nothing right now'}")
    if s.faults:
        print("faults:")
        for f in s.faults:
            print(f"  - {f}")
    return 0


def mode_main():
    rclpy.init()
    node = Node("mode_cli")
    try:
        if len(sys.argv) < 2:
            code = _show_state(node)
        else:
            # Longer than the slowest change (about 100 s when nodes fail) plus the up to 120 s a
            # stop request waits for a change already in progress.
            response = _call(node, ChangeState, "change_state", ChangeState.Request(new_state=sys.argv[1]), 250.0)
            if response is None:
                code = 1
            else:
                print(("OK: " if response.success else "REFUSED: ") + response.message)
                code = 0 if response.success else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


def save_map_main():
    rclpy.init()
    node = Node("save_map_cli")
    try:
        args = [a for a in sys.argv[1:] if not a.startswith("--")]
        name = args[0] if args else ""
        replace = "--replace" in sys.argv[1:]
        response = _call(node, SaveMap, "save_map", SaveMap.Request(name=name, replace=replace), 40.0)
        if response is None:
            code = 1
        else:
            print(("OK: " if response.success else "FAILED: ") + response.message)
            if response.name_taken:
                print("add --replace to overwrite it")
            if response.success:
                print(f"file: {response.path}")
            code = 0 if response.success else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


def zones_main():
    parser = argparse.ArgumentParser(prog="zones", description="The no-go zones of the map in use.")
    actions = parser.add_subparsers(dest="action")
    add = actions.add_parser("add", help="add a rectangle, given by two opposite corners in map metres")
    add.add_argument("name")
    for corner in ("x0", "y0", "x1", "y1"):
        add.add_argument(corner, type=float)
    delete = actions.add_parser("delete", help="delete a zone")
    delete.add_argument("name")
    args = parser.parse_args()

    rclpy.init()
    node = Node("zones_cli")
    try:
        code = _zones(node, args)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


def _zones(node, args) -> int:
    state = _robot_state(node)
    if state is None:
        return 1
    map_name = "" if state.state == "mapping" else state.current_map
    if not map_name:
        print("No saved map in use: no-go zones belong to a saved map.")
        return 1
    answer = _call(node, GetParameters, "mode_manager/get_parameters", GetParameters.Request(names=["maps_dir"]), 5.0)
    if answer is None or not answer.values or not answer.values[0].string_value:
        print("Could not ask the mode manager where the maps are.")
        return 1
    path = map_files.zones_path(answer.values[0].string_value, map_name)
    try:
        zones = map_files.read_zones(path)
    except Exception as exc:
        print(f"Cannot read {path}: {exc}")
        return 1

    if args.action is None:
        if not zones:
            print(f"The map '{map_name}' has no no-go zones.")
        else:
            print(f"No-go zones of the map '{map_name}' ({path}):")
            for zone in zones:
                corners = " ".join(f"({float(x):.2f}, {float(y):.2f})" for x, y in zone["polygon"])
                print(f"  {zone['name']}: {corners}")
        return 0
    names = [str(z["name"]) for z in zones]
    if args.action == "add":
        if args.name in names:
            print(f"There is already a zone called '{args.name}'; delete it first to replace it.")
            return 1
        zones.append({"name": args.name, "polygon": zone_rules.rectangle(args.x0, args.y0, args.x1, args.y1)})
    else:
        if args.name not in names:
            print(f"The map '{map_name}' has no zone called '{args.name}'. Its zones: {', '.join(names) or 'none'}")
            return 1
        zones = [z for z in zones if str(z["name"]) != args.name]

    request = SetNoGoZones.Request(map_name=map_name, zones=[_zone_msg(z) for z in zones])
    response = _call(node, SetNoGoZones, "set_no_go_zones", request, 30.0)
    if response is None:
        return 1
    print(("OK: " if response.success else "REFUSED: ") + response.message)
    return 0 if response.success else 1


def _zone_msg(zone: dict) -> NoGoZone:
    points = [Point32(x=float(x), y=float(y), z=0.0) for x, y in zone["polygon"]]
    return NoGoZone(name=str(zone["name"]), polygon=Polygon(points=points))


def settings_main():
    parser = argparse.ArgumentParser(prog="settings", description="The settings a user may change from the app.")
    parser.add_argument("changes", nargs="*", metavar="NAME=VALUE",
                        help="cleaning_speed, manual_speed (m/s), row_spacing (m), row_angle (degrees, or auto)")
    args = parser.parse_args()
    rclpy.init()
    node = Node("settings_cli")
    try:
        sys.exit(_settings(node, args.changes))
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _settings(node, changes) -> int:
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    got = []
    node.create_subscription(RobotSettingsMsg, "robot_settings", got.append, latched)
    for _ in range(50):
        if got:
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    if not got:
        print("The settings node is not running (nothing on /robot_settings).")
        return 1
    current = robot_settings.tidy(robot_settings.RobotSettings(
        cleaning_speed=got[-1].cleaning_speed, manual_speed=got[-1].manual_speed, row_spacing=got[-1].row_spacing,
        row_angle_auto=got[-1].row_angle_auto, row_angle=got[-1].row_angle))
    if not changes:
        print(robot_settings.describe(current))
        return 0
    values = {}
    for change in changes:
        name, _, value = change.partition("=")
        if name == "row_angle" and value.strip().lower() == "auto":
            values["row_angle_auto"] = True
            continue
        if name not in ("cleaning_speed", "manual_speed", "row_spacing", "row_angle") or not value:
            print(f"cannot read '{change}': use cleaning_speed=, manual_speed=, row_spacing= or row_angle= (a number, or auto)")
            return 2
        try:
            values[name] = float(value)
        except ValueError:
            print(f"'{value}' is not a number")
            return 2
        if name == "row_angle":
            values["row_angle_auto"] = False
    wanted = dataclasses.replace(current, **values)
    request = SetRobotSettings.Request()
    request.settings = RobotSettingsMsg(cleaning_speed=wanted.cleaning_speed, manual_speed=wanted.manual_speed,
                                        row_spacing=wanted.row_spacing, row_angle_auto=wanted.row_angle_auto,
                                        row_angle=wanted.row_angle)
    response = _call(node, SetRobotSettings, "set_robot_settings", request, 10.0)
    if response is None:
        return 1
    print(response.message)
    return 0 if response.success else 1

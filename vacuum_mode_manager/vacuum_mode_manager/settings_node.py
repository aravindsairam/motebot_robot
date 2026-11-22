"""The settings node: keeps the settings a user may change from the app.

    /robot_settings       vacuum_interfaces/msg/RobotSettings   latched: the settings in force
    /set_robot_settings   vacuum_interfaces/srv/SetRobotSettings  change them (the complete set)

They are kept in the file named by the parameter `settings_file` and read again at start, so they
outlast a restart. Who follows them: the cleaning node (cleaning speed, row spacing, row direction)
and the gateway (the joystick's top speed). This node only keeps and checks them.
"""

import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from vacuum_interfaces.msg import RobotSettings as RobotSettingsMsg
from vacuum_interfaces.srv import SetRobotSettings

from . import robot_settings
from .robot_settings import RobotSettings


def to_msg(s: RobotSettings) -> RobotSettingsMsg:
    return RobotSettingsMsg(cleaning_speed=s.cleaning_speed, manual_speed=s.manual_speed, row_spacing=s.row_spacing,
                            row_angle_auto=s.row_angle_auto, row_angle=s.row_angle)


def from_msg(msg) -> RobotSettings:
    return robot_settings.tidy(RobotSettings(cleaning_speed=msg.cleaning_speed, manual_speed=msg.manual_speed,
                                             row_spacing=msg.row_spacing, row_angle_auto=msg.row_angle_auto,
                                             row_angle=msg.row_angle))


class SettingsNode(Node):
    def __init__(self):
        super().__init__("settings_node")
        self.declare_parameter("settings_file", "")
        self._path = self.get_parameter("settings_file").value
        if not self._path:
            raise RuntimeError("parameter settings_file is required")
        self._lock = threading.Lock()
        self._settings, problem = robot_settings.load(self._path)
        if problem:
            self.get_logger().error(problem)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(RobotSettingsMsg, "robot_settings", latched)
        self.create_service(SetRobotSettings, "set_robot_settings", self._on_set)
        self._pub.publish(to_msg(self._settings))
        self.get_logger().info(f"settings: {robot_settings.describe(self._settings)} (file {self._path})")

    def _on_set(self, request, response):
        wanted = from_msg(request.settings)
        why = robot_settings.check(wanted)
        with self._lock:
            if not why:
                try:
                    robot_settings.save(self._path, wanted)
                except Exception as exc:
                    why = f"could not save the settings: {exc}"
            if why:
                response.success, response.message = False, why
                self.get_logger().warn(f"settings not changed: {why}")
            else:
                changed = robot_settings.changes(self._settings, wanted)
                self._settings = wanted
                self._pub.publish(to_msg(wanted))
                response.success, response.message = True, f"saved: {changed}" if changed else "nothing changed"
                self.get_logger().info(f"settings {response.message}")
            response.settings = to_msg(self._settings)
        return response


def main():
    rclpy.init()
    node = SettingsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

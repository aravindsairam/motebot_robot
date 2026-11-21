"""The exploration node starts, publishes "none", and answers a start it must refuse. With ROS, no
robot: no mode manager runs, so the start is refused for not knowing the robot's mode."""

import os
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402

try:
    from vacuum_interfaces.msg import ExplorationStatus
    from vacuum_interfaces.srv import StartExploration
    from vacuum_exploration.exploration_node import ExplorationNode
except ImportError as exc:  # the workspace is not built or not sourced
    pytest.skip(f"needs the built workspace: {exc}", allow_module_level=True)


@pytest.fixture
def ros(tmp_path):
    params = tmp_path / "exploration.yaml"
    params.write_text("frontier_explorer:\n  ros__parameters:\n    robot_base_frame: base_link\n")
    domain = str(200 + os.getpid() % 30)  # away from a running stack
    rclpy.init(args=["--ros-args", "-p", f"explorer_params:={params}", "-p", f"explorer_log:={tmp_path / 'explorer.log'}"],
               domain_id=int(domain))
    yield
    rclpy.shutdown()


def test_the_node_starts_reports_none_and_refuses_without_a_robot(ros):
    node = ExplorationNode()
    probe = rclpy.create_node("probe")
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    seen = []
    probe.create_subscription(ExplorationStatus, "exploration/status", seen.append, latched)
    client = probe.create_client(StartExploration, "exploration/start")
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(probe)
    try:
        end = time.monotonic() + 10.0
        while not seen and time.monotonic() < end:
            executor.spin_once(timeout_sec=0.1)
        assert seen and seen[-1].state == ExplorationStatus.NONE
        assert client.wait_for_service(timeout_sec=5.0)
        future = client.call_async(StartExploration.Request())
        end = time.monotonic() + 10.0
        while not future.done() and time.monotonic() < end:
            executor.spin_once(timeout_sec=0.1)
        answer = future.result()
        assert answer is not None and not answer.accepted and "mode is not known" in answer.message
    finally:
        node.shut_down()
        executor.shutdown()
        probe.destroy_node()
        node.destroy_node()

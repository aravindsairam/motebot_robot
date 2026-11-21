"""ros2 run vacuum_exploration explore [--minutes N]

Starts an exploration and follows it until it ends, one line per change. Ctrl+C stops following,
not the exploration; stop that with:  ros2 run vacuum_mode_manager mode mapping
(the robot then stands still in mapping mode, the map kept; save it with
 ros2 run vacuum_mode_manager save_map NAME).
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from vacuum_interfaces.msg import ExplorationStatus
from vacuum_interfaces.srv import StartExploration

_FINAL = (ExplorationStatus.FINISHED, ExplorationStatus.STOPPED, ExplorationStatus.FAILED)


def _line(s: ExplorationStatus) -> str:
    minutes, seconds = divmod(int(s.exploring_time), 60)
    return (f"[{s.state}] {s.message} | {s.places_left} places left, {s.mapped_area:.1f} m² mapped, "
            f"{minutes}:{seconds:02d}, {s.distance:.1f} m, goals {s.goals_sent} ({s.goals_failed} failed)")


def main(argv=None):
    parser = argparse.ArgumentParser(prog="explore", description="Let the robot map the house by itself.")
    parser.add_argument("--minutes", type=float, default=0.0,
                        help="time limit in minutes of robot time (default: the exploration node's, 20)")
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    rclpy.init()
    node = Node("explore_command")
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    box = {"status": None}
    node.create_subscription(ExplorationStatus, "exploration/status", lambda m: box.__setitem__("status", m), latched)
    client = node.create_client(StartExploration, "exploration/start")
    code = 1
    try:
        if not client.wait_for_service(timeout_sec=10.0):
            print("the exploration node is not running (no /exploration/start)")
            return 1
        answer = None
        for _ in range(2):  # a brand-new client can lose its first reply on Humble
            future = client.call_async(StartExploration.Request(time_limit=float(args.minutes) * 60.0))
            rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
            if future.done():
                answer = future.result()
                break
        if answer is None:
            print("no answer from the exploration node")
            return 1
        if not answer.accepted:
            print(f"refused: {answer.message}")
            return 1
        print(f"exploration {answer.job_id} started: {answer.message}")
        print("Ctrl+C stops following, not the exploration; stop it with: ros2 run vacuum_mode_manager mode mapping")
        last = ""
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            s = box["status"]
            if s is None or s.job_id != answer.job_id:
                continue
            text = f"[{s.state}] {s.message}"
            if text != last:
                print(_line(s), flush=True)
                last = text
            if s.state in _FINAL:
                code = 0 if s.state == ExplorationStatus.FINISHED else 1
                if s.state != ExplorationStatus.FAILED:
                    print("the robot is in mapping mode with the map kept; save it with: "
                          "ros2 run vacuum_mode_manager save_map NAME")
                break
    except KeyboardInterrupt:
        print("\nstopped following; the exploration goes on")
        code = 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())

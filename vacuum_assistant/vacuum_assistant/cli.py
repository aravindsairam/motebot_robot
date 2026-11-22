"""Ask the assistant from a terminal.

    ros2 run vacuum_assistant ask "what are you doing?"
    ros2 run vacuum_assistant ask                       a chat: one question per line, an empty line ends it
    ros2 run vacuum_assistant ask --new "..."           start a new conversation first
"""

import argparse
import sys
import uuid

import rclpy
from rclpy.node import Node

from vacuum_interfaces.srv import AskAssistant

TIMEOUT = 900.0  # s: a trip, or a tour of every room, takes minutes


def main():
    parser = argparse.ArgumentParser(prog="ask", description="Ask the robot's assistant.")
    parser.add_argument("question", nargs="*")
    parser.add_argument("--new", action="store_true", help="start a new conversation")
    args = parser.parse_args()
    rclpy.init()
    node = Node("ask_cli")
    client = node.create_client(AskAssistant, "assistant/ask")
    conversation = f"terminal-{uuid.uuid4().hex[:8]}" if args.new else "terminal"
    try:
        if not client.wait_for_service(timeout_sec=5.0):
            print("The assistant is not running (no /assistant/ask service).")
            sys.exit(1)
        questions = [" ".join(args.question)] if args.question else None
        while True:
            if questions is not None:
                if not questions:
                    break
                text = questions.pop(0)
            else:
                try:
                    text = input("you> ").strip()
                except EOFError:
                    break
                if not text:
                    break
            future = client.call_async(AskAssistant.Request(conversation=conversation, text=text))
            rclpy.spin_until_future_complete(node, future, timeout_sec=TIMEOUT)
            if not future.done() or future.result() is None:
                print(f"No answer within {TIMEOUT:.0f} s.")
                sys.exit(1)
            result = future.result()
            print(f"robot> {result.reply}")
            if result.tools_used:
                print(f"       (looked at: {', '.join(result.tools_used)})")
            if questions is not None and not result.success:
                sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

"""The assistant node: the robot's chat. It answers questions and does what it is asked, in plain words.

    /assistant/ask         vacuum_interfaces/srv/AskAssistant   a question; answered when done (seconds, or
                                                                minutes when the robot drives somewhere)
    /assistant/name_rooms  vacuum_interfaces/srv/NameRooms      name rooms by driving to each and looking
    /assistant/status      vacuum_interfaces/msg/AssistantStatus  what it is doing, for the app

It acts only through the robot's own services and actions (link.py), the same ones the app uses: the
mode manager, Nav2, the cleaning node and the safety gate keep the last word, and the app's Stop
works at any time. Without an API key or internet it says so, and the robot works as before.

The API key is read from the environment (ANTHROPIC_API_KEY) or from the file named by the
parameter `api_key_file`.
"""

import os
import threading
import time
from typing import Dict, Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from vacuum_interfaces.msg import AssistantStatus
from vacuum_interfaces.srv import AskAssistant, NameRooms

from . import brain, link, namer, robot_facts, tools

DEFAULT_KEY_FILE = "~/.config/motebot/anthropic_api_key"
CONVERSATION_IDLE = 1800.0  # s: a conversation not used for this long is forgotten
NO_KEY = "I have no API key yet, so I cannot chat. Put one in {path} on the robot's computer and ask again."


class AssistantNode(Node):
    def __init__(self):
        super().__init__("assistant_node")
        self.declare_parameter("model", brain.MODEL)
        self.declare_parameter("effort", "low")  # short chat answers; higher costs more and takes longer
        self.declare_parameter("api_key_file", DEFAULT_KEY_FILE)
        self.declare_parameter("refusal_fallback", True)
        self.declare_parameter("maps_dir", "")  # where what the robot has seen is kept, next to each map

        self._status = AssistantStatus()
        self._status_lock = threading.Lock()
        self._status_pub = self.create_publisher(AssistantStatus, "assistant/status", 10)
        self.robot = link.RosLink(self, doing=self._doing, maps_dir=self.param("maps_dir"), verify=self._verify)
        self._conversations: Dict[str, brain.Conversation] = {}
        self._locks: Dict[str, threading.Lock] = {}
        self._table_lock = threading.Lock()
        self._job_lock = threading.Lock()  # one robot-driving job (naming rooms) at a time
        self._client = None
        self._brain: Optional[brain.Brain] = None
        self._make_brain()
        services = ReentrantCallbackGroup()
        self.create_service(AskAssistant, "assistant/ask", self._on_ask, callback_group=services)
        self.create_service(NameRooms, "assistant/name_rooms", self._on_name_rooms, callback_group=services)
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(60.0, self._forget_old, clock=steady)
        self.create_timer(0.5, self._publish_status, clock=steady)

    def param(self, name):
        return self.get_parameter(name).value

    # ------------------------------------------------------------------ status
    def _set_status(self, busy: bool, conversation: str = "", doing: str = "") -> None:
        with self._status_lock:
            changed = (busy, conversation, doing) != (self._status.busy, self._status.conversation, self._status.doing)
            self._status = AssistantStatus(busy=busy, conversation=conversation, doing=doing)
        if changed:
            self._publish_status()

    def _doing(self, text: str) -> None:
        with self._status_lock:
            busy, conversation = self._status.busy, self._status.conversation
        if busy:
            self._set_status(True, conversation, text)

    def _publish_status(self) -> None:
        with self._status_lock:
            msg = self._status
        self._status_pub.publish(msg)

    # ------------------------------------------------------------------ the model
    def _api_key(self) -> str:
        key = os.environ.get("ANTHROPIC_API_KEY", "").strip()
        if key:
            return key
        try:
            with open(os.path.expanduser(self.param("api_key_file"))) as f:
                return f.read().strip()
        except OSError:
            return ""

    def _make_brain(self) -> None:
        key = self._api_key()
        if not key:
            self.get_logger().warn(f"no API key (ANTHROPIC_API_KEY, or the file {self.param('api_key_file')}): "
                                   "questions will be answered with how to add one")
            return
        import anthropic

        self._client = anthropic.Anthropic(api_key=key, timeout=120.0, max_retries=2)
        box = tools.ToolBox(self.robot, learn_rooms=self._learn_rooms)
        self._brain = brain.Brain(self._client, robot_facts.SYSTEM, tools.TOOLS, box.run, model=self.param("model"),
                                  effort=self.param("effort"), fallback=bool(self.param("refusal_fallback")),
                                  log=lambda text: self.get_logger().warn(text), on_tool=self._on_tool)
        self.get_logger().info(f"assistant ready: model {self.param('model')}, effort {self.param('effort')}")

    def _verify(self, picture: bytes, name: str, box: str) -> bool:
        """The object finder's guess, checked by the language model (without a key: not checked)."""
        if self._client is None:
            return False
        try:
            answer = namer.is_it(self._client, self.param("model"), picture, name, box)
        except Exception as exc:
            self.get_logger().warn(f"checking a find failed: {exc!r}")
            return False
        self.get_logger().info(f"is {box} a {name}? {'yes' if answer else 'no'}")
        return answer

    def _on_tool(self, name: str) -> None:
        self._doing(tools.TOOL_WORDS.get(name, "working on it"))

    # ------------------------------------------------------------------ questions
    def _on_ask(self, request, response):
        text = request.text.strip()
        if not text:
            response.success, response.reply = False, "Ask me something."
            return response
        if self._brain is None:
            try:
                self._make_brain()  # the key may have been added since
            except Exception as exc:
                self.get_logger().error(f"could not start the language model client: {exc!r}")
        if self._brain is None:
            response.success, response.reply = False, NO_KEY.format(path=self.param("api_key_file"))
            return response
        name = request.conversation or "default"
        with self._table_lock:
            conversation = self._conversations.setdefault(name, brain.Conversation())
            lock = self._locks.setdefault(name, threading.Lock())
        if not lock.acquire(blocking=False):  # one question at a time in a conversation
            response.success, response.reply = False, "I'm still answering your last question. Ask again when that answer is in."
            return response
        started = time.monotonic()
        self._set_status(True, name, "thinking")
        try:
            answer = self._brain.ask(conversation, text)
        except Exception as exc:  # never leave the asker without an answer
            self.get_logger().error(f"answering failed: {exc!r}")
            answer = brain.Answer(False, "Something went wrong while I was thinking. Try again.")
        finally:
            lock.release()
            self._set_status(False)
        took = time.monotonic() - started
        self.get_logger().info(f"[{name}] {text!r} -> {answer.text!r} (tools {answer.tools_used}, {took:.1f} s, "
                               f"{answer.input_tokens} tokens in, {answer.output_tokens} out)")
        response.success, response.reply, response.tools_used = answer.ok, answer.text, answer.tools_used
        response.picture_jpeg = list(answer.picture_jpeg)
        response.mark = [float(answer.mark[0]), float(answer.mark[1])] if answer.mark is not None else []
        return response

    def _forget_old(self):
        now = time.monotonic()
        with self._table_lock:
            for name in [n for n, c in self._conversations.items() if now - c.last_used > CONVERSATION_IDLE]:
                lock = self._locks.get(name)
                if lock is not None and lock.locked():
                    continue
                del self._conversations[name]
                self._locks.pop(name, None)

    # ------------------------------------------------------------------ naming rooms
    def _name(self, rooms) -> tuple:
        """Visit each room, look, then name them all together. (rooms named, a summary). A room it
        could not reach, or the model did not name, keeps its name."""
        if self._client is None:
            raise tools.RobotError(NO_KEY.format(path=self.param("api_key_file")))
        order = link.nearest_first([(r.x, r.y) for r in rooms], self.robot.pose())
        outlines = [[(p.x, p.y) for p in r.outline.points] for r in rooms]
        touching = namer.neighbours(outlines)
        evidence = [None] * len(rooms)
        for step, i in enumerate(order, start=1):  # visited nearest first, not zigzagging across the home
            room = rooms[i]
            self._doing(f"looking at room {step} of {len(rooms)}")
            outcome, pictures, seen = self.robot.look_in_room(room, things=namer.THINGS)
            if self.robot.mode != "navigation":
                raise tools.RobotError(f"stopped while naming the rooms (the robot is now {self.robot.mode})")
            evidence[i] = namer.Evidence(area=room.area, pictures=pictures, things=seen,
                                         opens_onto=[j + 1 for j in touching[i]], reached=outcome == "arrived")
        self._doing("working out which room is which")
        named = namer.name_rooms(self._client, self.param("model"), evidence)
        final = namer.unique([n for n, _ in named])
        notes = []
        for room, name, (_, why) in zip(rooms, final, named):
            notes.append(f"{name or room.name}: {why}" if why else f"{room.name}: not named")
            room.name = name or room.name
        return list(rooms), "; ".join(notes)

    def _on_name_rooms(self, request, response):
        if not self._job_lock.acquire(blocking=False):
            response.success, response.message = False, "already naming the rooms"
            return response
        self._set_status(True, "", "getting ready to name the rooms")
        try:
            self.robot.ready_to_drive()
            rooms, notes = self._name(list(request.rooms))
            response.success, response.rooms, response.message = True, rooms, notes
        except tools.RobotError as exc:
            response.success, response.message = False, str(exc)
        except Exception as exc:
            self.get_logger().error(f"naming the rooms failed: {exc!r}")
            response.success, response.message = False, f"naming the rooms failed: {exc}"
        finally:
            self._job_lock.release()
            self._set_status(False)
        self.get_logger().info(f"name rooms: {response.message}")
        return response

    def _learn_rooms(self) -> dict:
        """For the chat: find the rooms, name them by looking, save them."""
        if not self._job_lock.acquire(blocking=False):
            raise tools.RobotError("already naming the rooms")
        try:
            self.robot.ready_to_drive()
            found = self.robot.find_rooms()
            rooms, notes = self._name(found)
            message = self.robot.save_rooms(rooms)
            return {"saved": True, "rooms": [{"name": r.name, "area_m2": round(r.area, 1)} for r in rooms], "seen": notes,
                    "message": message}
        finally:
            self._job_lock.release()


def main():
    rclpy.init()
    node = AssistantNode()
    # Each question holds a thread while it is answered (minutes when the robot drives), and the
    # robot's answers to its tools need free ones: enough for several phones asking at once.
    executor = MultiThreadedExecutor(num_threads=12)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

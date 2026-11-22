"""What the assistant may do with the robot: the tools the language model can call, and what each
gives back to it. No ROS imports: the robot is anything with the methods RosLink has (link.py),
which lets the tests use a stand-in.

Every tool goes through the robot's own services (see link.py), so the mode manager, Nav2, the
cleaning node and the safety gate keep the last word. A tool that cannot do its job says why in
plain words; the model passes that on.
"""

import json
import time
from dataclasses import dataclass, field
from datetime import datetime
from typing import List, Optional, Tuple

MAX_HISTORY = 20


class RobotError(Exception):
    """The robot could not do it (a node is not running, a mode forbids it, ...); the text says why."""


@dataclass
class ToolOutput:
    text: str
    is_error: bool = False
    pictures: List[bytes] = field(default_factory=list)  # JPEG pictures the model sees, and the user too
    mark: Optional[Tuple[float, float]] = None  # a place on the map that goes with the answer


def _schema(properties=None, required=None):
    properties = properties or {}
    return {"type": "object", "properties": properties, "required": list(required if required is not None else properties),
            "additionalProperties": False}


TOOLS = [
    {"name": "get_status", "strict": True, "input_schema": _schema(),
     "description": "What the robot is doing right now: its mode, any problem, the map it uses, the room it is in, the "
                    "clean under way (if any) with its progress, and the user's settings. Also the time now."},
    {"name": "get_cleaning_history", "strict": True,
     "input_schema": _schema({"count": {"type": "integer", "description": f"how many cleans, 1 to {MAX_HISTORY}"}}),
     "description": "The most recent cleans, newest first: when, what was cleaned, how it ended, floor area, share of the "
                    "planned floor, minutes, rows skipped, bumps; and the totals over every clean."},
    {"name": "get_rooms", "strict": True, "input_schema": _schema(),
     "description": "The rooms of the home and how clean each is: when a clean last covered most of it (and how much), "
                    "when a clean last passed through it, cleans in the last week, its floor area. Also the room the "
                    "robot is in. Use it for 'is the kitchen clean?' and to know the rooms' names."},
    {"name": "look", "strict": True,
     "input_schema": _schema({"around": {"type": "boolean", "description": "false: one picture straight ahead. true: four "
                                         "pictures turning on the spot (about 15 s; the robot must be free to drive)"}}),
     "description": "Take pictures with the front camera and look at them (the user sees the last one). For 'what do you "
                    "see', or to check something near the robot. The camera is low, 7 cm above the floor."},
    {"name": "find_object", "strict": True,
     "input_schema": _schema({"name": {"type": "string", "description": "the thing, in a few words: 'sofa', 'red chair'"},
                              "look_around": {"type": "boolean", "description": "turn on the spot to look all round if it "
                                              "is not in front (about 30 s; the robot must be free to drive)"}}),
     "description": "Look for a thing with the object finder and say where it is: distance, direction, room, and its place "
                    "on the map. Shows the picture with the thing marked."},
    {"name": "go_to_room", "strict": True,
     "input_schema": _schema({"room": {"type": "string", "description": "the room's name, as get_rooms gives it"}}),
     "description": "Drive to a room (its most open spot) and wait until there, up to a few minutes. Not during a clean "
                    "(stop it first, if the user wants that) or while making a map."},
    {"name": "go_near", "strict": True,
     "input_schema": _schema({"name": {"type": "string", "description": "the thing: 'sofa', 'dining table'"}}),
     "description": "Find a thing (looking all round if it is not in front) and drive up to it, stopping about half a "
                    "metre away, facing it. Waits until there. Only things the camera can see from where the robot is: "
                    "for something in another room, go to that room first."},
    {"name": "start_cleaning", "strict": True,
     "input_schema": _schema({"target": {"type": "string", "description": "'everything', or a room's name"},
                              "passes": {"type": "integer", "description": "1, or 2 for a second pass across the first"}}),
     "description": "Start a clean of the whole home or of one room. Answers at once with the plan (rows, minutes); the "
                    "robot cleans on its own and goes back to its dock when finished."},
    {"name": "pause_cleaning", "strict": True,
     "input_schema": _schema({"resume": {"type": "boolean", "description": "true to go on with a paused clean"}}),
     "description": "Pause the clean under way, or go on with a paused one."},
    {"name": "stop", "strict": True, "input_schema": _schema(),
     "description": "Stop whatever the robot is doing (a clean, a trip) and stand still."},
    {"name": "go_to_dock", "strict": True, "input_schema": _schema(),
     "description": "Send the robot back to its charging dock. Ends a clean that is running."},
    {"name": "change_settings",
     "input_schema": {"type": "object", "additionalProperties": False, "properties": {
         "cleaning_speed": {"type": "number", "description": "m/s while cleaning, 0.10 to 0.30"},
         "joystick_speed": {"type": "number", "description": "m/s at full joystick, 0.10 to 0.30"},
         "row_spacing_cm": {"type": "number", "description": "cm between cleaning rows, 10 to 30 in steps of 5"},
         "row_direction": {"type": "string", "description": "'auto', or the rows' angle in degrees from the map's x axis"}}},
     "description": "Change some of the user's settings; leave out the ones to keep. Cleaning speed changes at once; "
                    "row spacing and direction from the next clean."},
    {"name": "learn_rooms", "strict": True, "input_schema": _schema(),
     "description": "Find the rooms of the map and name them by driving to each and looking (a few minutes). The names "
                    "are saved; the user can rename them in the app (Rooms on the map). Only when the user asks for it."},
]

TOOL_WORDS = {  # what the robot is doing while a tool runs, for the app
    "get_status": "checking", "get_cleaning_history": "looking at past cleans", "get_rooms": "checking the rooms",
    "look": "looking", "find_object": "looking for it", "go_to_room": "on my way", "go_near": "looking for it",
    "start_cleaning": "starting the clean", "pause_cleaning": "pausing", "stop": "stopping", "go_to_dock": "heading to the dock",
    "change_settings": "changing the settings", "learn_rooms": "learning the rooms",
}


def when(seconds: float, now: Optional[float] = None) -> str:
    """A moment as people say it: "Sunday 22 Nov, 14:05 (3 hours ago)"."""
    if not seconds:
        return "never"
    now = time.time() if now is None else now
    ago = max(0.0, now - seconds)
    if ago < 90:
        rough = "just now"
    elif ago < 90 * 60:
        rough = f"{ago / 60:.0f} minutes ago"
    elif ago < 36 * 3600:
        rough = f"{ago / 3600:.0f} hours ago"
    else:
        rough = f"{ago / 86400:.0f} days ago"
    return f"{datetime.fromtimestamp(seconds).strftime('%A %d %b, %H:%M')} ({rough})"


class ToolBox:
    def __init__(self, robot, clock=time.time, learn_rooms=None):
        """`learn_rooms()`: finds, names and saves the rooms (the assistant node's own job), giving a dict."""
        self.robot = robot
        self.clock = clock
        self.learn_rooms = learn_rooms

    def run(self, name: str, args: dict) -> ToolOutput:
        try:
            return self._run(name, args)
        except RobotError as exc:
            return ToolOutput(f"could not: {exc}", True)

    def _run(self, name: str, args: dict) -> ToolOutput:
        r = self.robot
        now = self.clock()
        if name == "get_status":
            data = r.status()
            data["time_now"] = datetime.fromtimestamp(now).strftime("%A %d %B %Y, %H:%M")
            return ToolOutput(_json(data))
        if name == "get_cleaning_history":
            count = args.get("count", 5)
            if not isinstance(count, int) or isinstance(count, bool):
                return ToolOutput("count must be a whole number", True)
            data = r.history(max(1, min(MAX_HISTORY, count)))
            for record in data.get("cleans", []):
                for key in ("started", "ended"):
                    if key in record:
                        record[key] = when(record[key], now)
            return ToolOutput(_json(data))
        if name == "get_rooms":
            data = r.rooms_report()
            for room in data.get("rooms", []):
                for key in ("last_cleaned", "last_passed_through"):
                    room[key] = when(room.get(key, 0.0), now)
            data["time_now"] = datetime.fromtimestamp(now).strftime("%A %d %B %Y, %H:%M")
            return ToolOutput(_json(data))
        if name == "look":
            if args.get("around"):
                pictures = r.look_around()
                note = {"robot": r.where(), "pictures": "straight ahead first, then each a quarter turn to the left"}
            else:
                picture = r.picture()
                pictures = [picture] if picture else []
                note = {"robot": r.where()}
            if not pictures:
                return ToolOutput("no picture came from the camera", True)
            return ToolOutput(_json(note), pictures=pictures)
        if name == "find_object":
            thing = _text(args, "name")
            if args.get("look_around"):
                found, picture = r.find_around(thing)
            else:
                found, picture, _ = r.find(thing)
            out = {"looked_for": thing, "found": found is not None, "robot": r.where()}
            if found is not None:
                out["thing"] = found
            before = r.remembered(thing)
            if before is not None:
                before["seen"] = when(before["seen"], now)
                out["seen_before_at"] = before
            mark = (found["x"], found["y"]) if found and found.get("x") is not None else None
            return ToolOutput(_json(out), pictures=[picture] if picture else [], mark=mark)
        if name == "go_to_room":
            out = r.go_to_room(_text(args, "room"))
            return ToolOutput(_json(out), mark=(out["x"], out["y"]))
        if name == "go_near":
            out, picture = r.go_near(_text(args, "name"))
            thing = out.get("thing") or {}
            mark = (thing["x"], thing["y"]) if thing.get("x") is not None else None
            return ToolOutput(_json(out), pictures=[picture] if picture else [], mark=mark)
        if name == "start_cleaning":
            passes = args.get("passes", 1)
            return ToolOutput(_json(r.start_cleaning(_text(args, "target"), passes if isinstance(passes, int) else 1)))
        if name == "pause_cleaning":
            return ToolOutput(_json(r.pause(bool(args.get("resume")))))
        if name == "stop":
            return ToolOutput(_json(r.stop()))
        if name == "go_to_dock":
            return ToolOutput(_json(r.dock()))
        if name == "change_settings":
            changes = {k: v for k, v in args.items() if k in ("cleaning_speed", "joystick_speed", "row_spacing_cm", "row_direction")}
            if not changes:
                return ToolOutput("say which settings to change", True)
            for k in ("cleaning_speed", "joystick_speed", "row_spacing_cm"):
                if k in changes and (not isinstance(changes[k], (int, float)) or isinstance(changes[k], bool)):
                    return ToolOutput(f"{k} must be a number", True)
            return ToolOutput(_json(r.change_settings(changes)))
        if name == "learn_rooms":
            if self.learn_rooms is None:
                return ToolOutput("learning the rooms is not available", True)
            return ToolOutput(_json(self.learn_rooms()))
        return ToolOutput(f"there is no tool named {name}", True)


def _text(args: dict, key: str) -> str:
    value = args.get(key)
    if not isinstance(value, str) or not value.strip():
        raise RobotError(f"{key} is missing")
    return " ".join(value.split())


def _json(data) -> str:
    return json.dumps(data, ensure_ascii=False, separators=(",", ":"))

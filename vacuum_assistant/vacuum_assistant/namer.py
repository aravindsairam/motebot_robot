"""Naming rooms from what the robot saw. No ROS imports.

The robot stands just inside each room's doorway, takes four pictures turning on the spot, and the
object finder looks in each for things that tell rooms apart (THINGS). Then the language model is
given everything at once: every room's pictures, the things seen in it, its floor area and which
rooms it opens onto. Seeing the whole home together it can tell the only room with a toilet is the
bathroom, or that the small room joining four others is the hallway, which a room on its own
seen from 7 cm above the floor often does not show.
"""

import base64
import json
import math
import re
from dataclasses import dataclass, field
from typing import Dict, List, Sequence, Tuple

THINGS = ("bed", "sofa", "armchair", "toilet", "bathtub", "sink", "fridge", "stove", "kitchen counter", "dining table",
          "chair", "desk", "wardrobe", "television", "bookshelf", "washing machine", "stool")

ASK = """\
A robot vacuum visited each room of a home. In each room it stood just inside the doorway and took \
four pictures turning on the spot (the first faces into the room). Its camera is 7 cm above the \
floor, so furniture is seen from below and tables show mostly their legs. An object finder also \
looked for typical things in every picture; its finds are often wrong on their own, so weigh them \
against the pictures.

Name every room as the people living there would: Kitchen, Living room, Bedroom, Bathroom, Hallway, \
Office, Dining room, Laundry, Toilet and the like. A small room that connects several others is \
usually a hallway. Use each name only where it fits; rooms of the same kind can share a name (they \
will be numbered). Give a short reason for each.

{rooms}"""

SCHEMA = {
    "type": "object",
    "properties": {"rooms": {"type": "array", "items": {
        "type": "object",
        "properties": {"room": {"type": "integer"}, "name": {"type": "string"}, "why": {"type": "string"}},
        "required": ["room", "name", "why"], "additionalProperties": False}}},
    "required": ["rooms"],
    "additionalProperties": False,
}


@dataclass
class Evidence:
    area: float  # m2
    pictures: List[bytes] = field(default_factory=list)
    things: Dict[str, float] = field(default_factory=dict)  # what the object finder saw: thing -> best score
    opens_onto: List[int] = field(default_factory=list)  # other rooms (1-based numbers) it touches
    reached: bool = True


def clean_name(name: str) -> str:
    name = re.sub(r"[^\w '\-]", "", name, flags=re.UNICODE)
    name = " ".join(name.split())[:30]
    return name[:1].upper() + name[1:] if name else ""


def unique(names: Sequence[str]) -> List[str]:
    """Rooms of the same kind numbered: Bedroom, Bedroom -> Bedroom 1, Bedroom 2. Empty names stay empty."""
    counts = {}
    for n in names:
        if n:
            counts[n.lower()] = counts.get(n.lower(), 0) + 1
    seen = {}
    out = []
    for n in names:
        if n and counts[n.lower()] > 1:
            seen[n.lower()] = seen.get(n.lower(), 0) + 1
            out.append(f"{n} {seen[n.lower()]}")
        else:
            out.append(n)
    return out


def neighbours(outlines: Sequence[Sequence[Tuple[float, float]]], gap: float = 0.35) -> List[List[int]]:
    """For each room, the others (0-based) whose outline comes within `gap` of its own."""
    def near(a, b):
        for p in a:
            for q0, q1 in zip(b, list(b[1:]) + list(b[:1])):
                if _segment_distance(p, q0, q1) <= gap:
                    return True
        return False
    out = []
    for i, a in enumerate(outlines):
        out.append([j for j, b in enumerate(outlines) if j != i and (near(a, b) or near(b, a))])
    return out


def _segment_distance(p, a, b) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length2))
    return math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy))


def name_rooms(client, model: str, rooms: Sequence[Evidence]) -> List[Tuple[str, str]]:
    """(name, why) for each room, in order; ("", "") for a room the model did not name."""
    content = []
    lines = []
    for i, room in enumerate(rooms, start=1):
        facts = f"Room {i}: {room.area:.1f} m2 of floor"
        if room.opens_onto:
            facts += ", opens onto " + ", ".join(f"room {j}" for j in room.opens_onto)
        if room.things:
            facts += "; the object finder saw " + ", ".join(f"{t} ({s:.0%} sure)" for t, s in sorted(room.things.items(), key=lambda kv: -kv[1]))
        if not room.reached:
            facts += "; the robot could not get in, so there are no pictures"
        lines.append(facts)
        if room.pictures:
            content.append({"type": "text", "text": f"Pictures of room {i}:"})
            content += [{"type": "image", "source": {"type": "base64", "media_type": "image/jpeg",
                                                     "data": base64.standard_b64encode(p).decode("ascii")}} for p in room.pictures]
    content.append({"type": "text", "text": ASK.format(rooms="\n".join(lines))})
    response = client.messages.create(model=model, max_tokens=4000, messages=[{"role": "user", "content": content}],
                                      output_config={"effort": "medium", "format": {"type": "json_schema", "schema": SCHEMA}})
    out = [("", "")] * len(rooms)
    if response.stop_reason == "refusal":
        return out
    text = "".join(b.text for b in response.content if getattr(b, "type", "") == "text")
    try:
        data = json.loads(text)
    except ValueError:
        return out
    out = list(out)
    for entry in data.get("rooms", []):
        i = entry.get("room")
        if isinstance(i, int) and 1 <= i <= len(rooms):
            out[i - 1] = (clean_name(str(entry.get("name", ""))), str(entry.get("why", ""))[:200])
    return out


CHECK = ("A robot vacuum's camera (7 cm above the floor) took this picture. An object finder drew boxes on it; look "
         "at {box}. Is the thing in that box a {name}? Furniture looks unusual from this low, so judge by its shape "
         "and size, but say no if the box is only legs of something else, floor, wall or a different thing.")
CHECK_SCHEMA = {"type": "object", "properties": {"answer": {"type": "string", "enum": ["yes", "no", "unsure"]}},
                "required": ["answer"], "additionalProperties": False}


def is_it(client, model: str, picture: bytes, name: str, box: str) -> bool:
    """Does the language model agree that the boxed thing in the picture is a `name`?"""
    content = [{"type": "image", "source": {"type": "base64", "media_type": "image/jpeg",
                                            "data": base64.standard_b64encode(picture).decode("ascii")}},
               {"type": "text", "text": CHECK.format(box=box, name=name)}]
    response = client.messages.create(model=model, max_tokens=1000, messages=[{"role": "user", "content": content}],
                                      output_config={"effort": "low", "format": {"type": "json_schema", "schema": CHECK_SCHEMA}})
    if response.stop_reason == "refusal":
        return False
    text = "".join(b.text for b in response.content if getattr(b, "type", "") == "text")
    try:
        return json.loads(text).get("answer") == "yes"
    except ValueError:
        return False

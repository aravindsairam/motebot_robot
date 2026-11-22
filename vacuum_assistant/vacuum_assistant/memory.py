"""What the robot has seen and where: things the object finder found, kept per map. No ROS imports.

So "go to the sofa" works when the sofa is in another part of the room, or was seen while the robot
learned the rooms. Each thing keeps its best sighting; a sighting of the same kind of thing within
SAME metres of a kept one is the same thing seen again.

The file, next to the map (map_files.things_path):
    things:
      - {name: sofa, x: 0.95, y: -0.4, score: 0.62, seen: 1795369550.7}
"""

import math
import os
import tempfile
import time
from dataclasses import asdict, dataclass
from typing import List, Optional

import yaml

SAME = 1.0  # m


@dataclass
class Sighting:
    name: str
    x: float
    y: float
    score: float
    seen: float  # seconds since 1970


class Memory:
    def __init__(self, path: str = ""):
        self.path = path
        self.things: List[Sighting] = []
        if path and os.path.exists(path):
            try:
                with open(path) as f:
                    data = yaml.safe_load(f) or {}
                self.things = [Sighting(str(d["name"]), float(d["x"]), float(d["y"]), float(d["score"]), float(d["seen"]))
                               for d in data.get("things") or []]
            except Exception:
                self.things = []  # a broken file only means starting with nothing remembered

    def add(self, name: str, x: float, y: float, score: float, when: Optional[float] = None) -> None:
        name = " ".join(name.lower().split())
        when = time.time() if when is None else when
        for t in self.things:
            if t.name == name and math.hypot(t.x - x, t.y - y) <= SAME:
                if score >= t.score:
                    t.x, t.y, t.score = x, y, score
                t.seen = when
                return
        self.things.append(Sighting(name, x, y, score, when))

    def best(self, name: str, min_score: float = 0.0) -> Optional[Sighting]:
        """The surest sighting of `name` (or of a thing whose name holds it: "chair" finds "dining chair")."""
        wanted = " ".join(name.lower().split())
        found = [t for t in self.things if t.score >= min_score and (t.name == wanted or wanted in t.name or t.name in wanted)]
        return max(found, key=lambda t: (t.name == wanted, t.score)) if found else None

    def save(self) -> None:
        if not self.path:
            return
        directory = os.path.dirname(os.path.abspath(self.path))
        fd, temporary = tempfile.mkstemp(prefix="." + os.path.basename(self.path) + ".", suffix=".tmp", dir=directory)
        try:
            with os.fdopen(fd, "w") as f:
                f.write("# Things the robot has seen on this map, and where (map metres).\n")
                yaml.safe_dump({"things": [{k: (round(v, 3) if isinstance(v, float) else v) for k, v in asdict(t).items()}
                                           for t in self.things]}, f, sort_keys=False)
            os.replace(temporary, self.path)
        except BaseException:
            if os.path.exists(temporary):
                os.unlink(temporary)
            raise

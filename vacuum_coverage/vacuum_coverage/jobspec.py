"""What a cleaning job asks for, as plain data. No ROS imports.

The cleaning node turns the CleaningJob message into this, checks it, and keeps it in saved plans
and records as plain YAML.
"""

import math
from dataclasses import asdict, dataclass, field, replace
from typing import List, Optional, Tuple

from .planner import Options, spot_area

EVERYTHING, AREA, SPOT, PLAN = "everything", "area", "spot", "plan"
WHATS = (EVERYTHING, AREA, SPOT, PLAN)


@dataclass(frozen=True)
class JobSpec:
    what: str = EVERYTHING
    area: List[Tuple[float, float]] = field(default_factory=list)  # AREA: polygon corners, map frame
    spot: Optional[Tuple[float, float]] = None  # SPOT: the point
    plan_name: str = ""  # PLAN: the saved plan; also kept when a saved plan is run
    direction: str = "auto"
    passes: int = 1
    edges: Optional[bool] = None  # None: the robot's setting
    return_to_dock: bool = True

    @property
    def target(self) -> str:
        """What is being cleaned, in a word or the plan's name."""
        if self.plan_name:
            return self.plan_name
        return self.what

    def check(self) -> Optional[str]:
        """Why this job cannot be done, or None."""
        if self.what not in WHATS:
            return f"unknown kind of job '{self.what}' (what to clean must be one of {', '.join(WHATS)})"
        if self.direction not in ("auto", "x", "y"):
            return f"unknown row direction '{self.direction}': use x, y or auto"
        if self.passes not in (1, 2):
            return "passes must be 1 or 2"
        if self.what == AREA:
            if len(self.area) < 3:
                return "an area needs at least 3 corners"
            if not all(math.isfinite(v) for p in self.area for v in p):
                return "an area corner is not a number"
            if _polygon_area(self.area) < 1e-6:
                return "the area has no floor in it: its corners do not enclose anything"
        if self.what == SPOT:
            if self.spot is None or not all(math.isfinite(v) for v in self.spot):
                return "the spot is not a number"
        if self.what == PLAN and not self.plan_name.strip():
            return "no plan name given"
        return None

    def options(self, edges_by_default: bool) -> Options:
        return Options(direction=self.direction, passes=self.passes, edges=edges_by_default if self.edges is None else self.edges)

    def area_polygon(self, spot_size: float) -> Optional[List[Tuple[float, float]]]:
        if self.what == AREA:
            return [tuple(p) for p in self.area]
        if self.what == SPOT:
            return spot_area(self.spot[0], self.spot[1], spot_size)
        return None

    def with_saved(self, saved: "JobSpec") -> "JobSpec":
        """Running a saved plan: its area and options, this request's docking, and the plan's name."""
        return replace(saved, plan_name=self.plan_name, return_to_dock=self.return_to_dock)

    def to_dict(self) -> dict:
        d = asdict(self)
        d["area"] = [[float(x), float(y)] for x, y in self.area]
        d["spot"] = list(self.spot) if self.spot is not None else None
        return d

    @staticmethod
    def from_dict(d: dict) -> "JobSpec":
        return JobSpec(
            what=str(d.get("what", EVERYTHING)),
            area=[(float(p[0]), float(p[1])) for p in d.get("area") or []],
            spot=tuple(float(v) for v in d["spot"]) if d.get("spot") is not None else None,
            plan_name=str(d.get("plan_name", "")),
            direction=str(d.get("direction", "auto")),
            passes=int(d.get("passes", 1)),
            edges=None if d.get("edges") is None else bool(d["edges"]),
            return_to_dock=bool(d.get("return_to_dock", True)),
        )


def _polygon_area(pts) -> float:
    return abs(sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(pts, list(pts[1:]) + list(pts[:1])))) / 2.0

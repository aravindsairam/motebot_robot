"""When the no-go zones may be changed, and what makes a list of zones acceptable. No ROS imports.

The mode manager asks these before it saves a new list of zones for a map (/set_no_go_zones).
A zone is {'name': str, 'polygon': [[x, y], ...]} in the map frame (metres), as map_files reads it.
Each function returns the reason for refusing, in a short plain sentence, or "" when all is well.
"""

import math
from typing import Iterable, List, Optional, Sequence, Tuple

from . import state_machine as sm

MAX_ZONES = 50
MIN_CORNERS, MAX_CORNERS = 3, 20
MAX_NAME = 40  # characters
MIN_WIDTH = 0.10  # m: a zone narrower than this, in any direction, is a slip of the finger
MIN_AREA = 0.001  # m² (10 cm²): less is a line, not a zone
ROBOT_CLEARANCE = 0.20  # m: a new zone may not cover the robot's centre or come this close to it
DOCK_CLEARANCE = 0.50  # m: nor come this close to the robot's place on the dock
_TOLERANCE = 1e-5  # m: more than a float32 corner (as in geometry_msgs/Polygon) can be off by

Point = Tuple[float, float]


def mode_problem(mode: str, changing: bool, map_in_use: str, map_name: str, making_map: bool = False) -> str:
    """Can the zones of `map_name` be changed now? `map_in_use` is "" when there is no saved map.
    `making_map`: a map is being made (RobotState.making_map), whatever the mode is called."""
    if changing:
        return "the robot is changing modes: try again in a moment"
    if making_map or mode in sm.MAP_MAKING:
        return "no-go zones belong to a saved map and cannot be changed while a map is being made"
    if mode == sm.CLEANING:
        return "stop the clean first: no-go zones cannot change during a clean"
    if not map_in_use:
        return "there is no saved map yet: make a map first"
    if map_name != map_in_use:
        return f"the robot uses the map '{map_in_use}', not '{map_name or 'none'}'"
    return ""


def map_extent(meta: dict) -> Tuple[float, float, float, float]:
    """(min x, min y, max x, max y) of a saved map, from map_files.read_map_meta."""
    (ox, oy), res = meta["origin"], meta["resolution"]
    return ox, oy, ox + meta["width"] * res, oy + meta["height"] * res


def check_zones(
    zones: Sequence[dict],
    extent: Tuple[float, float, float, float],
    robot: Optional[Point] = None,
    dock: Optional[Point] = None,
    saved: Iterable[dict] = (),
) -> str:
    """Why this list of zones cannot be saved for a map covering `extent`, or "" if it can.

    robot:  where the robot's centre is on the map, if known: a new zone must leave it room to move.
    dock:   where the robot stands when docked: a new zone must leave the way onto the dock open.
    saved:  the map's zones as they are now. A zone saved before and not moved is not held to the
            map's edges, the robot or the dock: otherwise one old zone could block every change.
            The robot cleans as close as 15 cm to a zone, a replaced map may be smaller than the
            old one, and the house's stairwell zone runs 7 cm past the edge of the saved map.
    """
    zones = list(zones)
    if len(zones) > MAX_ZONES:
        return f"at most {MAX_ZONES} no-go zones, and this list has {len(zones)}"
    unmoved = {_key(z["polygon"]) for z in saved if _key(z.get("polygon")) is not None}
    names = set()
    for number, zone in enumerate(zones, start=1):
        name = str(zone.get("name", ""))
        why = _name_problem(name, number, names)
        if why:
            return why
        names.add(name)
        corners, why = _corners(zone.get("polygon"), name)
        if why:
            return why
        why = _shape_problem(corners, name)
        if why:
            return why
        if _key(corners) in unmoved:
            continue
        if not _within(corners, extent):
            return f"zone '{name}' reaches outside the map"
        if robot is not None and distance_to_polygon(robot, corners) < ROBOT_CLEARANCE:
            return f"zone '{name}' covers the robot or comes within {ROBOT_CLEARANCE * 100:.0f} cm of it: move the robot away first"
        if dock is not None and distance_to_polygon(dock, corners) < DOCK_CLEARANCE:
            return f"zone '{name}' comes within {DOCK_CLEARANCE * 100:.0f} cm of the dock: the robot must be able to reach it"
    return ""


def _name_problem(name: str, number: int, taken: set) -> str:
    if not name.strip():
        return f"zone {number} has no name"
    if len(name) > MAX_NAME:
        return f"the zone name '{name[:MAX_NAME]}...' is too long: at most {MAX_NAME} characters"
    if not name.isprintable():
        return f"zone {number}'s name has a line break or another character that cannot be shown"
    if name in taken:
        return f"two zones are called '{name}'"
    return ""


def _corners(polygon, name: str) -> Tuple[List[Point], str]:
    try:
        corners = [(float(x), float(y)) for x, y in polygon]
    except (TypeError, ValueError):
        return [], f"zone '{name}' is not a list of [x, y] corners"
    if not MIN_CORNERS <= len(corners) <= MAX_CORNERS:
        return [], f"zone '{name}' has {len(corners)} corners: a zone has {MIN_CORNERS} to {MAX_CORNERS}"
    if not all(math.isfinite(v) for p in corners for v in p):
        return [], f"zone '{name}' has a corner that is not a number"
    return corners, ""


def _within(corners: List[Point], extent) -> bool:
    x0, y0, x1, y1 = extent
    return all(x0 - _TOLERANCE <= x <= x1 + _TOLERANCE and y0 - _TOLERANCE <= y <= y1 + _TOLERANCE for x, y in corners)


def _shape_problem(corners: List[Point], name: str) -> str:
    if narrowest_width(corners) < MIN_WIDTH - _TOLERANCE:
        return f"zone '{name}' is less than {MIN_WIDTH * 100:.0f} cm across"
    if crosses_itself(corners):
        return f"zone '{name}' crosses itself: draw its outline without crossing"
    if abs(area(corners)) < MIN_AREA:
        return f"zone '{name}' has almost no area: its corners are nearly in a line"
    return ""


def _key(polygon) -> Optional[tuple]:
    """A zone's corners rounded to 1 mm, to tell whether it has moved. None if it has no corners."""
    try:
        return tuple((round(float(x), 3), round(float(y), 3)) for x, y in polygon)
    except (TypeError, ValueError):
        return None


# ---------------------------------------------------------------- geometry
def area(corners: Sequence[Point]) -> float:
    """Signed area (shoelace): positive when the corners go anticlockwise."""
    n = len(corners)
    return 0.5 * sum(corners[i][0] * corners[(i + 1) % n][1] - corners[(i + 1) % n][0] * corners[i][1] for i in range(n))


def _cross(o: Point, a: Point, b: Point) -> float:
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def convex_hull(points: Sequence[Point]) -> List[Point]:
    """The convex hull, anticlockwise (Andrew's monotone chain)."""
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts
    lower: List[Point] = []
    upper: List[Point] = []
    for p in pts:
        while len(lower) >= 2 and _cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and _cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def narrowest_width(corners: Sequence[Point]) -> float:
    """How wide the zone is in its narrowest direction: the smallest distance between two parallel
    lines that hold it. For a rectangle, its shorter side, however it is turned."""
    hull = convex_hull(corners)
    if len(hull) < 3:
        return 0.0
    best = math.inf
    for i in range(len(hull)):
        a, b = hull[i], hull[(i + 1) % len(hull)]
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        if length > 0.0:
            best = min(best, max(abs(_cross(a, b, p)) / length for p in hull))
    return best


def _segments_cross(a: Point, b: Point, c: Point, d: Point) -> bool:
    """Do segments a-b and c-d cross at a point inside both?"""
    return _cross(a, b, c) * _cross(a, b, d) < 0 and _cross(c, d, a) * _cross(c, d, b) < 0


def crosses_itself(corners: Sequence[Point]) -> bool:
    """Does the outline cross itself (a figure of eight)? Such a zone would leave it unclear what
    is inside."""
    n = len(corners)
    edges = [(corners[i], corners[(i + 1) % n]) for i in range(n)]
    for i in range(n):
        for j in range(i + 2, n):
            if i == 0 and j == n - 1:
                continue  # the first and last edges share a corner
            if _segments_cross(*edges[i], *edges[j]):
                return True
    return False


def inside(point: Point, corners: Sequence[Point]) -> bool:
    """Is the point inside the polygon (even-odd rule)?"""
    x, y = point
    result = False
    n = len(corners)
    for i in range(n):
        (xa, ya), (xb, yb) = corners[i], corners[(i + 1) % n]
        if (ya > y) != (yb > y) and x < xa + (y - ya) * (xb - xa) / (yb - ya):
            result = not result
    return result


def distance_to_polygon(point: Point, corners: Sequence[Point]) -> float:
    """Distance from a point to a zone: 0 inside it, else to its nearest edge."""
    if inside(point, corners):
        return 0.0
    px, py = point
    best = math.inf
    n = len(corners)
    for i in range(n):
        (ax, ay), (bx, by) = corners[i], corners[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        length2 = dx * dx + dy * dy
        t = 0.0 if length2 == 0.0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length2))
        best = min(best, math.hypot(px - (ax + t * dx), py - (ay + t * dy)))
    return best


# ---------------------------------------------------------------- editing a list
def rectangle(x0: float, y0: float, x1: float, y1: float) -> List[List[float]]:
    """The four corners of the rectangle with opposite corners (x0, y0) and (x1, y1), anticlockwise."""
    left, right, bottom, top = min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1)
    return [[left, bottom], [right, bottom], [right, top], [left, top]]

"""Pictures of a plan and of a finished clean, drawn over the map. No ROS imports.

    plan picture    rows, connectors, transits and edges over the map, with the planned floor
                    tinted, left-out floor in orange and floor the robot cannot reach in purple
    record picture  the map with the cleaned floor tinted and the path driven drawn on it
"""

import io
from typing import Iterable, Optional, Sequence, Tuple

import numpy as np
from PIL import Image, ImageDraw

from .grid import Point
from .mapdata import MapData
from .planner import CONNECTOR, EDGE, ROW, TRANSIT, Plan

_FREE = (250, 250, 250)
_OCCUPIED = (40, 40, 40)
_UNKNOWN = (190, 190, 190)
_NOGO = (235, 170, 170)
_PLANNED = (205, 232, 205)
_CLEANED = (150, 205, 240)
_LEFT_OUT = (250, 190, 110)
_UNREACHABLE = (200, 170, 230)
_LINES = {ROW: (20, 90, 200), CONNECTOR: (120, 160, 230), TRANSIT: (150, 150, 150), EDGE: (20, 140, 60)}


def _base(m: MapData) -> np.ndarray:
    rgb = np.empty(m.cells.shape + (3,), dtype=np.uint8)
    rgb[:] = _UNKNOWN
    rgb[m.cells == 0] = _FREE
    rgb[m.cells == 100] = _OCCUPIED
    rgb[m.nogo & (m.cells != 100)] = _NOGO
    return rgb


def _to_image(rgb: np.ndarray, scale: int) -> Image.Image:
    img = Image.fromarray(np.flipud(rgb), mode="RGB")  # row 0 is the bottom of the map
    return img.resize((img.width * scale, img.height * scale), Image.NEAREST)


def _pixel(m: MapData, scale: int, p: Point) -> Tuple[float, float]:
    res = m.grid.resolution
    ox, oy = m.grid.origin
    h = m.cells.shape[0]
    return ((p[0] - ox) / res * scale, (h - (p[1] - oy) / res) * scale)


def plan_picture(m: MapData, plan: Plan, robot: Optional[Point] = None, scale: int = 4) -> Image.Image:
    rgb = _base(m)
    rgb[plan.planned] = _PLANNED
    rgb[plan.left_out] = _LEFT_OUT
    rgb[plan.unreachable] = _UNREACHABLE
    img = _to_image(rgb, scale)
    draw = ImageDraw.Draw(img)
    for p in plan.pieces:
        a, b = _pixel(m, scale, p.start), _pixel(m, scale, p.end)
        if p.kind == TRANSIT:
            _dashed(draw, a, b, _LINES[TRANSIT])
        else:
            draw.line([a, b], fill=_LINES[p.kind], width=2 if p.kind in (ROW, EDGE) else 1)
    if robot is not None:
        x, y = _pixel(m, scale, robot)
        draw.ellipse([x - 5, y - 5, x + 5, y + 5], outline=(200, 0, 0), width=2)
    return img


def record_picture(m: MapData, cleaned: np.ndarray, path: Sequence[Point], scale: int = 3) -> bytes:
    """PNG bytes: the map, the cleaned floor tinted, the path driven on top."""
    rgb = _base(m)
    rgb[cleaned & (m.cells == 0)] = _CLEANED
    img = _to_image(rgb, scale)
    if len(path) > 1:
        ImageDraw.Draw(img).line([_pixel(m, scale, p) for p in path], fill=(20, 60, 140), width=1)
    out = io.BytesIO()
    img.save(out, format="PNG", optimize=True)
    return out.getvalue()


def _dashed(draw: ImageDraw.ImageDraw, a, b, colour, dash: float = 6.0) -> None:
    length = float(np.hypot(b[0] - a[0], b[1] - a[1]))
    if length == 0:
        return
    n = max(1, int(length // dash))
    for i in range(0, n, 2):
        t0, t1 = i / n, min((i + 1) / n, 1.0)
        draw.line([(a[0] + (b[0] - a[0]) * t0, a[1] + (b[1] - a[1]) * t0), (a[0] + (b[0] - a[0]) * t1, a[1] + (b[1] - a[1]) * t1)],
                  fill=colour, width=1)


def summary(plan: Plan) -> Iterable[str]:
    """The plan in numbers, one line each, for the terminal."""
    e = plan.estimate
    yield f"rows:            {e.rows} (directions: {', '.join(plan.directions)})"
    if plan.part_order:
        yield f"parts:           {len(plan.part_order)} (rooms, or areas behind a narrow gap; cleaned one at a time)"
    if e.edges:
        yield f"edges:           {e.edges}"
    yield f"turns:           {e.turns}"
    yield f"connectors:      {e.connectors}"
    yield f"transits:        {e.transits}"
    yield f"distance:        {e.distance:.1f} m"
    yield f"planned floor:   {e.planned_area:.2f} m2"
    yield f"left out:        {e.left_out_area:.2f} m2"
    yield f"cannot reach:    {e.unreachable_area:.2f} m2"
    yield f"expected time:   {e.time / 60.0:.1f} min"
    yield f"planning time:   {e.planning_time:.2f} s"

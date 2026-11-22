"""Coverage planning: the back-and-forth rows that clean a floor. No ROS imports.

Given the map, where the robot is, and what to clean, the planner returns the pieces the robot
drives, in order:

    row        one straight cleaning pass
    connector  a short straight hop to the next row (about one row spacing), inside row-clear space
    transit    a longer trip to another part; Nav2 plans it around the furniture
    edge       a straight pass along a wall or a piece of furniture (optional, after a part's rows)

Every piece is straight: a path with a corner in it is never sent to the controller, which would
cut the corner. Each piece starts where the one before it ended; the first starts at the robot.

The method:
  1. The clearance of every cell: how far its centre is from the nearest obstacle (a wall, furniture,
     unknown space, a no-go zone). Row-clear cells keep `row_clearance`; the robot can travel through
     cells that keep `transit_clearance`, and only the part it can reach from where it stands is planned.
  2. The reachable floor: every floor cell the cleaning tool (a disc `cleaning_width` across, centred
     on the robot) could reach from some row-clear place the robot can get to.
  3. Parts. The space the robot can travel in is split where it narrows: a watershed on the clearance,
     flooded from the widest places down. Two basins stay apart where the passage between them is
     narrower than `_PART_NECK` times the widest place of the smaller one (a doorway, a gap between
     two pieces of furniture), and join otherwise (the two ends of one room). A part is a room, or a
     nook of one behind a narrow gap; each floor cell belongs to the part whose row-clear space is
     nearest.
  4. The parts are cleaned one at a time, in the order with the shortest trips between them (every
     order is tried when there are few), and each part is left near the way to the next.
  5. In each part, rows on lines along x or y ("auto": whichever needs fewer turns and transits in
     that part), or at an angle the user chose (the lines are then laid over the map turned by that
     angle); on each line, the unbroken runs of row-clear cells are the rows, fitted to the edges
     of each region and spread evenly at no more than `row_spacing`; then extra rows are added on
     in-between lines while one would clean at least `min_extra_area` that nothing else reaches.
     A row is left out if everything it would clean is cleaned anyway: by the other rows, by the
     part's edge pass, or in a part cleaned before.
  6. Rows are grouped into regions in which each row has exactly one neighbour on the next line, so a
     region is one clean zig-zag. The order of a part's regions, and which way each is driven, is
     the shortest found from several first orders (nearest first; sweeps across the part from each
     side), each improved while the trips get shorter: a run of regions moved elsewhere or driven
     backwards, each region driven the way that leads best on to the next.
  7. Optionally a second pass at right angles (to that part's first), and a pass along the walls
     and furniture (edges), both in the same part before the robot moves on to the next.

The planned floor is every floor cell the tool passes over on the rows, connectors and edges (the
brush stays on between rows). Reachable floor it misses is "left out": slivers at the rounded ends of
rows, and patches only a row shorter than `min_row_length`, or not worth a row, would reach.

The same inputs always give the same plan.
"""

import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .grid import (Clearance, Grid, Point, clearance_field, dilate, disc_offsets, nearest, polygon_mask, reachable, runs, small_blobs,
                   sample_segment, stamp_segment)

ROW, CONNECTOR, TRANSIT, EDGE = "row", "connector", "transit", "edge"
_EPS = 1e-9
_TURN_ANGLE = math.radians(30.0)  # a change of heading larger than this counts as a turn
# Splitting into parts. A passage narrower than _PART_NECK times the widest place (clearance, capped
# at _PART_CAP) of the smaller of the two sides it joins separates them: a 0.8 m doorway (clearance
# 0.4 m) separates rooms wider than about 1.35 m. A room wider than 2 x _PART_CAP counts as that wide.
_PART_NECK = 0.6
_PART_CAP = 1.25
_TRANSIT_COST = 0.3  # m: what a trip Nav2 plans costs beyond its length, when choosing the order
_EXACT_ORDER_PARTS = 8  # up to this many parts, every order of them is weighed
_MOVE_RUN = 3  # the longest run of regions moved elsewhere in one step when improving an order
_MANY_ITEMS = 40  # with more regions than this in a part, only the nearest-first order is improved (time)
# A row is left out as needless only if the others reach the floor within half a row spacing of it
# plus this much. At the default spacing (0.25) that is the whole cleaning width (0.30), as before;
# at a closer spacing the user chose, the overlap they asked for is kept instead of every other row
# being dropped (its two neighbours alone would reach all its floor).
_NEEDLESS_MARGIN = 0.025


class PlanError(Exception):
    """Nothing can be planned; the message says why, in plain words."""


@dataclass(frozen=True)
class Settings:
    """Lengths in metres, speeds in m/s, times in seconds (vacuum_bringup/config/coverage.yaml)."""

    cleaning_width: float = 0.30  # floor cleaned in one pass: a disc this wide, centred on the robot
    row_spacing: float = 0.25  # distance between rows
    row_clearance: float = 0.25  # robot centre to any obstacle, on rows and connectors
    edge_clearance: float = 0.20  # the same for the pass along the edges
    transit_clearance: float = 0.20  # decides which parts can be reached at all
    small_obstacle_area: float = 0.015  # m2: obstacles this small on the map (furniture legs) ...
    small_obstacle_margin: float = 0.05  # ... count this much bigger, for rows, edges and trips alike
    min_row_length: float = 0.20  # shorter rows are left out, unless one alone reaches a pocket worth min_extra_area
    min_edge_length: float = 0.30  # shorter straight stretches of edge are not driven
    max_connector_length: float = 0.50  # a longer hop between rows is a transit
    min_extra_area: float = 0.02  # m2: uncovered floor smaller than this is not worth an extra row
    spot_size: float = 1.5  # side of the square a spot clean covers
    area_margin: float = 0.05  # with an area, rows keep the robot's centre this far inside its edge
    cleaning_speed: float = 0.20  # on rows, connectors and edges
    transit_speed: float = 0.18  # on transits, on average
    transit_detour: float = 1.3  # a transit's length, estimated as this times the straight distance
    turn_time: float = 3.0  # per turn on the spot


@dataclass(frozen=True)
class Options:
    # "auto": in each part, x or y, whichever needs fewer turns and transits. "x", "y", or the angle
    # of the rows in degrees from the map's x axis (a number, or one as text: "45").
    direction: str = "auto"
    passes: int = 1  # 2: a second pass at right angles to the first
    edges: bool = False  # a pass along walls and furniture after the rows


@dataclass(frozen=True)
class Piece:
    kind: str
    start: Point
    end: Point
    number: int = 0  # rows: row number, edges: edge number (both from 1); 0 for the others
    pass_number: int = 0  # 1 or 2 for rows and the connectors between them; 0 otherwise
    region: int = 0  # rows of one region form one zig-zag

    @property
    def length(self) -> float:
        return math.dist(self.start, self.end)

    @property
    def heading(self) -> float:
        return math.atan2(self.end[1] - self.start[1], self.end[0] - self.start[0])


@dataclass
class Estimate:
    rows: int = 0
    edges: int = 0
    connectors: int = 0
    transits: int = 0
    turns: int = 0
    distance: float = 0.0  # m, transits estimated
    planned_area: float = 0.0  # m2 of floor the plan cleans
    left_out_area: float = 0.0  # m2 only a row shorter than min_row_length could clean
    unreachable_area: float = 0.0  # m2 of floor in parts the robot cannot reach from where it is
    time: float = 0.0  # s
    planning_time: float = 0.0  # s spent planning


@dataclass
class Plan:
    pieces: List[Piece]
    planned: np.ndarray  # bool per cell: the floor the tool passes over on rows, connectors and edges
    left_out: np.ndarray  # bool: reachable floor the plan misses
    unreachable: np.ndarray  # bool: floor the tool would reach from parts the robot cannot get to
    estimate: Estimate
    directions: List[str] = field(default_factory=list)  # row direction of each pass: "x", "y", "45°", or "mixed" (by part)
    reachable: Optional[np.ndarray] = None  # bool: the reachable floor, fixed before any row is laid
    parts: Optional[np.ndarray] = None  # int per cell: the part (room, 1, 2, ...) of each place the robot can get to; 0 elsewhere
    part_order: List[int] = field(default_factory=list)  # the parts in the order they are cleaned


def spot_area(x: float, y: float, size: float) -> List[Point]:
    """The square cleaned by a spot clean around (x, y)."""
    h = size / 2.0
    return [(x - h, y - h), (x + h, y - h), (x + h, y + h), (x - h, y + h)]


# ---------------------------------------------------------------- row directions
def parse_direction(direction) -> Optional[float]:
    """The angle of the rows in degrees from the map's x axis, 0 up to (not including) 180; None
    for "auto". Takes "auto", "x" (0), "y" (90) or a number, also as text. ValueError otherwise."""
    if isinstance(direction, str):
        text = direction.strip().lower()
        if text == "auto":
            return None
        if text in ("x", "y"):
            return 0.0 if text == "x" else 90.0
        try:
            direction = float(text)
        except ValueError:
            raise ValueError(f"unknown row direction '{direction}': use auto, x, y or an angle in degrees") from None
    angle = float(direction)
    if not math.isfinite(angle):
        raise ValueError("the row direction is not a number")
    angle = round(angle % 180.0, 6)
    return 0.0 if angle >= 180.0 else angle


def direction_name(angle: float) -> str:
    """How a row direction is written: "x", "y", or the angle ("45°")."""
    if angle == 0.0:
        return "x"
    if angle == 90.0:
        return "y"
    return f"{angle:g}°"


# ---------------------------------------------------------------- the frame of one direction
class _Frame:
    """Rows along x run over the grid's columns, one line per grid row; rows along y over its rows.
    Working in "lines and positions" lets one piece of code do both.

    Rows at any other angle run over a second grid of the same cell size, laid over the map turned
    by that angle: its lines run along the rows. A mask is seen through it by taking, for each of
    its cells, the map cell its centre falls in. `clear` then says where a row may really go."""

    def __init__(self, grid: Grid, angle: float):
        self.grid = grid
        self.angle = angle
        self.turned = angle not in (0.0, 90.0)
        self.clear: Optional[np.ndarray] = None  # turned frames: cells whose own centre is row-clear (see _Context._frame)
        if not self.turned:
            return
        res = grid.resolution
        h, w = grid.shape
        self._cos, self._sin = math.cos(math.radians(angle)), math.sin(math.radians(angle))
        # Along the rows (a) and across them (b): the map's corners give the size of the turned grid.
        corners = [(grid.origin[0] + i * w * res, grid.origin[1] + j * h * res) for i in (0, 1) for j in (0, 1)]
        along = [x * self._cos + y * self._sin for x, y in corners]
        across = [-x * self._sin + y * self._cos for x, y in corners]
        self._a0, self._b0 = min(along), min(across)
        self.shape = (int(math.ceil((max(across) - self._b0) / res - _EPS)), int(math.ceil((max(along) - self._a0) / res - _EPS)))
        lines, positions = np.mgrid[0:self.shape[0], 0:self.shape[1]]
        a = self._a0 + (positions + 0.5) * res
        b = self._b0 + (lines + 0.5) * res
        self._x = a * self._cos - b * self._sin
        self._y = a * self._sin + b * self._cos
        cols = np.floor((self._x - grid.origin[0]) / res).astype(np.int64)
        rows = np.floor((self._y - grid.origin[1]) / res).astype(np.int64)
        self._on = (rows >= 0) & (rows < h) & (cols >= 0) & (cols < w)
        self._rows, self._cols = rows[self._on], cols[self._on]

    def view(self, mask: np.ndarray) -> np.ndarray:
        if not self.turned:
            return mask if self.angle == 0.0 else mask.T
        out = np.zeros(self.shape, dtype=mask.dtype)
        out[self._on] = mask[self._rows, self._cols]
        return out

    def point(self, line: int, pos: int) -> Point:
        if not self.turned:
            return self.grid.centre(line, pos) if self.angle == 0.0 else self.grid.centre(pos, line)
        return (float(self._x[line, pos]), float(self._y[line, pos]))

    def clear_cells(self, clearance: Clearance, allowed: np.ndarray, need: float) -> np.ndarray:
        """Turned frames: the cells seen as `allowed` whose own centre keeps `need` from every
        obstacle. The centre of a turned cell is not the centre of the map cell it falls in, so the
        map cell's clearance alone could be up to 0.7 of a cell too generous."""
        out = self.view(allowed)
        lines, positions = np.nonzero(out)
        points = np.stack([self._x[lines, positions], self._y[lines, positions]], axis=1)
        for lo in range(0, len(points), 2048):  # in parts: each point is weighed against every cell near it
            near = clearance.of_points(points[lo:lo + 2048]) >= need - _EPS
            out[lines[lo:lo + 2048], positions[lo:lo + 2048]] = near
        return out


@dataclass(frozen=True)
class _Row:
    line: int
    a: int  # first position (cell) of the row
    b: int  # last position


# ---------------------------------------------------------------- planning
def make_plan(grid: Grid, robot: Point, settings: Settings, options: Options, area: Optional[Sequence[Point]] = None) -> Plan:
    started = time.perf_counter()
    s = settings
    res = grid.resolution
    angle = parse_direction(options.direction)  # ValueError for a direction that is none
    if options.passes not in (1, 2):
        raise ValueError("passes must be 1 or 2")

    # Furniture legs: on the map a leg is a cell or two, but the lidar sees a thin post bigger (its
    # noise spreads the hits), and Nav2's controller then refuses to pass between two legs with the
    # few centimetres a row there leaves. So small obstacles count `small_obstacle_margin` bigger
    # here; walls and big furniture, which the lidar sees as clean lines, do not.
    keep_clear = Grid(grid.floor, grid.resolution, grid.origin)
    if s.small_obstacle_margin > 0.0:
        legs = small_blobs(~grid.floor, int(s.small_obstacle_area / (res * res) + _EPS))
        if legs.any():
            keep_clear = Grid(grid.floor & ~dilate(legs, s.small_obstacle_margin, res), grid.resolution, grid.origin)
    clearance = Clearance(keep_clear, max(s.row_clearance, s.edge_clearance, s.transit_clearance) + 2.0 * res)
    clear = clearance.field
    travel = clear >= s.transit_clearance - _EPS
    start = nearest(travel, grid.cell_of(*robot))
    if start is None:
        raise PlanError("there is no floor the robot can reach")
    reach = reachable(travel, start)
    inside = polygon_mask(grid, area) if area else np.ones(grid.shape, dtype=bool)
    # Rows keep the centre a little inside an area's edge: at a row's end the real robot overshoots
    # by a few centimetres, and localization adds as much again.
    rows_inside = inside
    if area and s.area_margin > 0.0:
        rows_inside = inside & ~dilate(~inside, math.ceil(s.area_margin / res - _EPS) * res, res)
    row_all = (clear >= s.row_clearance - _EPS) & rows_inside
    row_ok = row_all & reach
    if not row_ok.any():
        if area:
            raise PlanError("there is no floor the robot can reach in that area")
        raise PlanError("there is no floor the robot can reach from where it is")
    half = s.cleaning_width / 2.0
    # With an area, only floor inside it counts (the tool may still sweep up to `half` beyond its edge).
    reachable_floor = dilate(row_ok, half, res) & grid.floor & inside
    unreachable = dilate(row_all & ~reach, half, res) & grid.floor & ~reachable_floor

    ctx = _Context(grid, s, clearance, row_ok, travel, reachable_floor)
    labels = _split(reach & inside, clearance_field(grid.floor, res, _PART_CAP), _PART_NECK)
    pieces, directions, order = ctx.plan(labels, robot, start, options, angle, reach, rows_inside)

    pieces = _number(pieces)
    planned = cleaned_by(grid, pieces, half) & grid.floor & inside
    left_out = reachable_floor & ~planned
    estimate = _estimate(pieces, robot, s, res, planned, left_out, unreachable)
    estimate.planning_time = time.perf_counter() - started
    return Plan(pieces=pieces, planned=planned, left_out=left_out, unreachable=unreachable, estimate=estimate,
                directions=directions, reachable=reachable_floor, parts=labels, part_order=order)


class _Context:
    def __init__(self, grid: Grid, s: Settings, clearance: Clearance, row_ok, travel, planned):
        self.grid = grid
        self.s = s
        self.res = grid.resolution
        self.clearance = clearance
        self.row_ok = row_ok
        self.travel = travel
        self.planned = planned
        self.spacing = max(1, int(round(s.row_spacing / self.res)))
        self.half = s.cleaning_width / 2.0
        self._frames: Dict[float, _Frame] = {}

    def _frame(self, angle: float) -> _Frame:
        frame = self._frames.get(angle)
        if frame is None:
            frame = _Frame(self.grid, angle)
            if frame.turned:
                frame.clear = frame.clear_cells(self.clearance, self.row_ok, self.s.row_clearance)
            self._frames[angle] = frame
        return frame

    # ---------------------------------------------------------- part by part
    def plan(self, labels: np.ndarray, robot: Point, start: Tuple[int, int], options: Options, angle: Optional[float],
             reach, rows_inside):
        """(pieces, row direction of each pass, the parts in the order cleaned). `angle`: the rows'
        direction in degrees, or None for x or y, whichever suits each part."""
        row_labels = np.where(self.row_ok, labels, 0)
        floor_labels = np.where(self.planned, _nearest_label(row_labels, self.half, self.res), 0)
        parts = _PartMap(self.grid, labels, self.row_ok, robot, start)
        edges: Dict[int, List[Tuple[Point, Point]]] = {}
        if options.edges:
            for seg in self._edge_segments(reach, rows_inside):
                edges.setdefault(parts.part_of_segment(seg), []).append(seg)
        has_rows = np.bincount(row_labels.ravel(), minlength=parts.n + 1) > 0
        order = parts.order([p for p in range(1, parts.n + 1) if has_rows[p] or p in edges])

        pieces: List[Piece] = []
        here = robot
        used: Dict[int, List[float]] = {1: [], 2: []}
        regions = {1: 0, 2: 0}
        elsewhere = np.zeros(self.grid.shape, dtype=bool)  # floor cleaned in the parts before this one
        for k, p in enumerate(order):
            done = len(pieces)
            target = parts.towards(p, order[k + 1]) if k + 1 < len(order) else None
            stages = (list(range(1, options.passes + 1)) if has_rows[p] else []) + ([0] if p in edges else [])
            ok, floor = row_labels == p, floor_labels == p
            # Floor cleaned anyway: by the parts before, and by this part's edges.
            covered = elsewhere | cleaned_by(self.grid, [Piece(EDGE, a, b) for a, b in edges.get(p, [])], self.half)
            first = None
            for stage in stages:
                aim = target if stage == stages[-1] else None
                if stage == 0:
                    new, here = self._edge_pieces(edges[p], here, aim)
                    pieces += new
                    continue
                if stage == 1:
                    choices = (0.0, 90.0) if angle is None else (angle,)
                else:
                    choices = ((first + 90.0) % 180.0,)
                # Leaving an edge (at edge_clearance), the hop to the first row may keep that clearance too.
                lead = self.s.edge_clearance if pieces and pieces[-1].kind == EDGE else self.s.row_clearance
                tries = [self._pass(d, ok, floor & ~covered, here, aim, stage, regions[stage], lead) for d in choices]
                new, end, direction, regions[stage] = min(tries, key=lambda t: self._score(t[0], t[1], aim, t[2]))
                if stage == 1:
                    first = direction
                used[stage].append(direction)
                pieces += new
                here = end
            elsewhere |= cleaned_by(self.grid, pieces[done:], self.half)
        directions = []
        for stage in range(1, options.passes + 1):
            kinds = sorted(set(used[stage]))
            directions.append(direction_name(kinds[0]) if len(kinds) == 1 else "mixed" if kinds else "")
        return pieces, directions, order

    def _score(self, pieces: List[Piece], end: Point, aim: Optional[Point], direction: float):
        """Which of two passes over a part is better: fewer turns and transits, then less time
        (the trip on to the next part included), then x before y."""
        s = self.s
        driven = sum(p.length for p in pieces if p.kind != TRANSIT)
        transit = sum(p.length for p in pieces if p.kind == TRANSIT) + (math.dist(end, aim) if aim is not None else 0.0)
        turns = _turns(pieces, end)
        seconds = driven / s.cleaning_speed + transit * s.transit_detour / s.transit_speed + turns * s.turn_time
        return (turns + _count(pieces, TRANSIT), round(seconds, 6), direction)

    # ---------------------------------------------------------- rows of one pass over one part
    def _pass(self, direction: float, ok: np.ndarray, floor: np.ndarray, here: Point, aim: Optional[Point],
              pass_number: int, region: int, lead: float):
        """(pieces, where the robot ends, direction, last region number) for one pass over the
        row-clear cells `ok` of one part, to clean `floor`. `lead`: the clearance the hop to the
        first row needs to be a connector."""
        frame = self._frame(direction)
        rows = self._rows(frame, ok, floor)
        chains = _chains(rows, self.spacing)
        ways = [[(frame.point(legs[0][0].line, legs[0][1]), frame.point(legs[-1][0].line, legs[-1][2]), legs)
                 for legs in _entries(chain)] for chain in chains]
        pieces: List[Piece] = []
        for ci, wi in self._order([[(a, b) for a, b, _ in w] for w in ways], here, aim, self.s.row_clearance):
            region += 1
            for i, (row, a, b) in enumerate(ways[ci][wi][2]):
                start, end = frame.point(row.line, a), frame.point(row.line, b)
                need = self.s.row_clearance if pieces else min(lead, self.s.row_clearance)
                pieces += self._hop(here, start, need, pass_number if i else 0, region if i else 0)
                pieces.append(Piece(ROW, start, end, pass_number=pass_number, region=region))
                here = end
        return pieces, here, direction, region

    def _row_cells(self, shape, rows: Sequence[_Row]) -> np.ndarray:
        mask = np.zeros(shape, dtype=bool)
        for r in rows:
            mask[r.line, r.a:r.b + 1] = True
        return mask

    def _rows(self, frame: _Frame, row_ok: np.ndarray, floor: np.ndarray):
        """The rows of one direction over the row-clear cells `row_ok` (positions in the frame's view),
        fitted to each region's edges, to clean `floor`.

        Every line of cells is looked at. Runs of row-clear cells on neighbouring lines that join one
        to one form a region (one zig-zag without lifting off: a region ends where furniture splits
        or joins the runs). In each region the first and last rows run along its two edges, and the
        rows between are spread evenly at no more than row_spacing. A fixed grid of lines instead
        can fall up to a whole spacing short of an edge, and misses places narrower than a spacing."""
        ok = frame.view(row_ok)
        if frame.clear is not None:
            ok = ok & frame.clear
        reach = frame.view(floor)
        min_steps = int(math.ceil(self.s.min_row_length / self.res - _EPS))
        lines = np.flatnonzero(ok.any(axis=1))
        every = [_Row(int(line), a, b) for line in lines for a, b in runs(ok[line])]
        rows: List[_Row] = []
        for region in _chains(every, 1):
            first, last = region[0].line, region[-1].line
            span = last - first
            gaps = int(math.ceil(span / self.spacing - _EPS)) if span > 0 else 0
            picked = sorted({first + int(round(i * span / gaps)) for i in range(gaps + 1)}) if gaps else [first]
            by_line = {r.line: r for r in region}
            rows += [by_line[line] for line in picked if by_line[line].b - by_line[line].a >= min_steps]
        covered = dilate(self._row_cells(ok.shape, rows), self.half, self.res)
        uncovered = reach & ~covered
        # Extra rows where the regions' own rows leave a patch worth one (a diagonal edge, a corner).
        while uncovered.any():
            pick = self._best_extra_row(ok, uncovered, min_steps)
            if pick is None:
                break
            row, cells = pick
            rows.append(row)
            uncovered &= ~cells
        return self._drop_needless(ok.shape, sorted(set(rows), key=lambda r: (r.line, r.a)), reach)

    def _drop_needless(self, shape, rows: List[_Row], reach: np.ndarray) -> List[_Row]:
        """Leave out rows that clean nothing (of `reach`) that the other rows do not clean: where
        two regions' edge rows lie side by side, or where the edge pass cleans along a wall. One at
        a time, shortest first, as each one left out can make another one needed. Here a row cleans
        only out to half a row spacing (and _NEEDLESS_MARGIN) from its line, so close rows stay."""
        radius = min(self.half, self.s.row_spacing / 2.0 + _NEEDLESS_MARGIN)
        bands = [np.flatnonzero((self._band(shape, r, radius) & reach).ravel()) for r in rows]
        count = np.zeros(shape[0] * shape[1], dtype=np.int32)
        for b in bands:
            count[b] += 1
        keep = [True] * len(rows)
        for i in sorted(range(len(rows)), key=lambda i: (rows[i].b - rows[i].a, i)):
            if not (count[bands[i]] == 1).any():
                keep[i] = False
                count[bands[i]] -= 1
        return [r for r, k in zip(rows, keep) if k]

    def _band(self, shape, row: _Row, radius: Optional[float] = None) -> np.ndarray:
        """Cells (frame view) within `radius` (half a cleaning width) of the row."""
        radius = self.half if radius is None else radius
        k = int(math.floor(radius / self.res + _EPS))
        out = np.zeros(shape, dtype=bool)
        l0, l1 = max(row.line - k, 0), min(row.line + k, shape[0] - 1)
        p0, p1 = max(row.a - k, 0), min(row.b + k, shape[1] - 1)
        ll, pp = np.mgrid[l0:l1 + 1, p0:p1 + 1]
        dl = (ll - row.line) * self.res
        dp = np.maximum(np.maximum(row.a - pp, pp - row.b), 0) * self.res
        out[ll, pp] = dl * dl + dp * dp <= radius * radius + 1e-12
        return out

    def _best_extra_row(self, ok: np.ndarray, uncovered: np.ndarray, min_steps: int):
        helpers = ok & dilate(uncovered, self.half, self.res)  # places from which the tool reaches uncovered floor
        split = max(1, int(round(self.s.cleaning_width / self.res)))
        worth = int(math.ceil(self.s.min_extra_area / (self.res * self.res) - _EPS))
        # How much uncovered floor a row would clean, from running sums along each line: the tool's
        # disc reaches `reach[i]` positions either side of the row on the line i lines away.
        k = int(math.floor(self.half / self.res + _EPS))
        reach = [int(math.floor(math.sqrt(max(self.half ** 2 - (i * self.res) ** 2, 0.0)) / self.res + _EPS)) for i in range(k + 1)]
        n_lines, n_pos = ok.shape
        sums = np.zeros((n_lines + 2 * k, n_pos + 2 * k + 2), dtype=np.int64)
        sums[k:k + n_lines, k + 1:k + 1 + n_pos] = np.cumsum(uncovered, axis=1)
        sums[k:k + n_lines, k + 1 + n_pos:] = sums[k:k + n_lines, k + n_pos:k + n_pos + 1]

        def gain(line: int, c0: int, c1: int) -> int:
            total = 0
            for di in range(-k, k + 1):
                w = reach[abs(di)]
                total += int(sums[line + di + k, min(c1 + w, n_pos - 1) + k + 1] - sums[line + di + k, max(c0 - w, 0) + k])
            return total

        best = None
        for line in np.flatnonzero(helpers.any(axis=1)):
            for a, b in runs(ok[line]):
                hs = np.flatnonzero(helpers[line, a:b + 1]) + a
                if len(hs) == 0:
                    continue
                # A pocket where only a row shorter than min_row_length fits gets that short row,
                # if it cleans a worthwhile patch (`worth`) that nothing else reaches.
                if b == a:
                    continue  # a single cell is a point, not a row: it cannot be driven as a line
                for c0, c1 in (_clusters(hs, split) if b - a >= min_steps else [(a, b)]):
                    c0, c1 = _stretch(c0, c1, a, b, min_steps)
                    g = gain(int(line), c0, c1)
                    key = (g, -(c1 - c0), -line, -c0)  # most floor; then the shortest; then lowest
                    if g >= max(worth, 1) and (best is None or key > best[0]):
                        best = (key, _Row(int(line), int(c0), int(c1)))
        if best is None:
            return None
        row = best[1]
        return row, self._band(ok.shape, row)

    # ---------------------------------------------------------- visiting order
    def _order(self, items: List[List[Tuple[Point, Point]]], here: Point, aim: Optional[Point], clearance: float):
        """The order to drive `items` in, as [(item, way)]: each item is driven once, one of its ways
        (start, end). Several first orders (nearest first from `here`; and sweeps across the part from
        each side), each improved while the trips get shorter (see _improve, _best_ways); the
        shortest wins. With `aim`, the trip from the last item's end to it counts too."""
        ids: Dict[Point, int] = {}
        for p in [here] + [q for item in items for way in item for q in way] + ([aim] if aim is not None else []):
            ids.setdefault(p, len(ids))
        cost = self._costs(list(ids), clearance)
        nowhere = len(ids)  # cost 0 to and from anywhere: the end when there is no aim
        start, goal = ids[here], ids[aim] if aim is not None else nowhere
        ways = [[(ids[a], ids[b]) for a, b in item] for item in items]
        backwards = [[next((j for j, (a2, b2) in enumerate(item) if a2 == b and b2 == a), -1) for a, b in item] for item in ways]

        firsts = [_nearest_first(ways, cost, start)]
        if len(items) <= _MANY_ITEMS:
            middles = [np.mean([q for way in item for q in way], axis=0) for item in items]
            for axis in (0, 1):
                for sign in (1.0, -1.0):
                    firsts.append([(i, 0) for i in sorted(range(len(items)), key=lambda i: (sign * middles[i][axis], i))])
        best = None
        for seq in firsts:
            while True:
                seq = _best_ways(seq, ways, cost, start, goal)
                was = _length(seq, ways, cost, start, goal)
                seq = _improve(seq, ways, backwards, cost, start, goal)
                if _length(seq, ways, cost, start, goal) > was - 1e-9:
                    break
            length = _length(seq, ways, cost, start, goal)
            if best is None or length < best[0] - 1e-9:
                best = (length, seq)
        return best[1]

    def _costs(self, points: List[Point], clearance: float) -> List[List[float]]:
        """Rough cost of the trip between every two of the points (and a last, extra point that costs
        nothing to get to or from): its length, longer if something is in the way, plus _TRANSIT_COST
        if it cannot be a connector."""
        m = len(points)
        out = np.zeros((m + 1, m + 1))
        if m > 1:
            pts = np.asarray(points, dtype=float)
            ii, jj = np.triu_indices(m, 1)
            for lo in range(0, len(ii), 4096):
                i, j = ii[lo:lo + 4096], jj[lo:lo + 4096]
                c = self._trip_costs(pts[i], pts[j], clearance)
                out[i, j] = c
                out[j, i] = c
        return out.tolist()

    def _trip_costs(self, a: np.ndarray, b: np.ndarray, clearance: float) -> np.ndarray:
        res = self.res
        d = np.hypot(b[:, 0] - a[:, 0], b[:, 1] - a[:, 1])
        n = np.maximum(2, np.ceil(d / res).astype(np.int64) + 1)  # points along each trip, about one per cell
        first = np.cumsum(n) - n
        which = np.repeat(np.arange(len(d)), n)
        t = (np.arange(int(n.sum())) - first[which]) / (n[which] - 1)
        xs = a[which, 0] + t * (b[which, 0] - a[which, 0])
        ys = a[which, 1] + t * (b[which, 1] - a[which, 1])
        cols = np.floor((xs - self.grid.origin[0]) / res).astype(np.int64)
        rows = np.floor((ys - self.grid.origin[1]) / res).astype(np.int64)
        h, w = self.grid.shape
        on = (rows >= 0) & (rows < h) & (cols >= 0) & (cols < w)
        rows, cols = np.clip(rows, 0, h - 1), np.clip(cols, 0, w - 1)
        free = np.logical_and.reduceat(on & self.travel[rows, cols], first)
        clear = np.logical_and.reduceat(on & (self.clearance.field[rows, cols] >= clearance - _EPS), first)
        out = np.where(free, d, d * 1.5 + 1.0)
        out += np.where(clear & (d <= self.s.max_connector_length + _EPS), 0.0, _TRANSIT_COST)
        out[d < _EPS] = 0.0
        return out

    def _hop(self, here: Point, to: Point, clearance: float, pass_number: int, region: int) -> List[Piece]:
        """The piece that takes the robot from `here` to `to`: a connector if short and clear, else a transit."""
        if math.dist(here, to) < _EPS:
            return []
        if math.dist(here, to) <= self.s.max_connector_length + _EPS and self.clearance.segment_ok(here, to, clearance):
            return [Piece(CONNECTOR, here, to, pass_number=pass_number, region=region)]
        return [Piece(TRANSIT, here, to)]

    # ---------------------------------------------------------- edges
    def _edge_segments(self, reach: np.ndarray, inside: np.ndarray) -> List[Tuple[Point, Point]]:
        """Straight stretches along walls and furniture, at edge_clearance."""
        s, res = self.s, self.res
        clear = self.clearance.field
        edge_ok = (clear >= s.edge_clearance - _EPS) & reach & inside
        hugging = clear < s.edge_clearance + 1.5 * res  # next to something real, not just the area's border
        min_steps = int(math.ceil(s.min_edge_length / res - _EPS))
        segments: List[Tuple[Point, Point]] = []
        for direction in (0.0, 90.0):  # edges follow walls: along the map's own axes whatever the rows do
            frame = self._frame(direction)
            ok = frame.view(edge_ok)
            hug = frame.view(hugging)
            above = np.zeros_like(ok)
            above[:-1] = ok[1:]
            below = np.zeros_like(ok)
            below[1:] = ok[:-1]
            boundary = ok & hug & (~above | ~below)  # an obstacle just across this line
            for line in np.flatnonzero(boundary.any(axis=1)):
                for a, b in _merge_runs(runs(boundary[line]), ok[line], max_gap=2):
                    if b - a >= min_steps:
                        segments.append((frame.point(int(line), a), frame.point(int(line), b)))
        segments.sort()
        return segments

    def _edge_pieces(self, segments: List[Tuple[Point, Point]], here: Point, aim: Optional[Point]):
        """Straight passes along the given stretches of edge, and where the robot ends."""
        pieces: List[Piece] = []
        for i, j in self._order([[(a, b), (b, a)] for a, b in segments], here, aim, self.s.edge_clearance):
            a, b = segments[i] if j == 0 else segments[i][::-1]
            pieces += self._hop(here, a, self.s.edge_clearance, 0, 0)
            pieces.append(Piece(EDGE, a, b))
            here = b
        return pieces, here


# ---------------------------------------------------------------- parts
def _split(space: np.ndarray, height: np.ndarray, neck: float) -> np.ndarray:
    """Split the cells of `space` (side-by-side neighbours) into parts: a watershed on `height`,
    flooded from the highest cells down. Where two basins meet, they join if the meeting place is at
    least `neck` times as high as the lower of their two peaks, and stay apart otherwise. Returns the
    part of every cell: 1, 2, ... (highest peak first); 0 outside `space`."""
    h, w = space.shape
    flat = height.ravel()
    cells = np.flatnonzero(space.ravel())
    order = cells[np.lexsort((cells, -flat[cells]))].tolist()  # highest first; equal heights by position
    heights = flat.tolist()
    label = [0] * (h * w)
    parent: List[int] = [0]
    peak: List[float] = [0.0]

    def find(a: int) -> int:
        root = a
        while parent[root] != root:
            root = parent[root]
        while parent[a] != root:
            parent[a], a = root, parent[a]
        return root

    for idx in order:
        r, c = divmod(idx, w)
        roots: List[int] = []
        uphill, top = 0, -1.0  # the basin of the highest neighbour already flooded: the way uphill
        for nb, inside in ((idx - w, r > 0), (idx + w, r < h - 1), (idx - 1, c > 0), (idx + 1, c < w - 1)):
            if inside and label[nb]:
                q = find(label[nb])
                if q not in roots:
                    roots.append(q)
                if heights[nb] > top:
                    uphill, top = q, heights[nb]
        if not roots:
            parent.append(len(parent))
            peak.append(heights[idx])
            label[idx] = len(parent) - 1
            continue
        roots.sort(key=lambda q: (-peak[q], q))
        for q in roots[1:]:
            if heights[idx] >= neck * peak[q] - _EPS:  # peak[q] is the lower of the two
                parent[q] = roots[0]
        label[idx] = find(uphill)
    out = np.fromiter((find(v) if v else 0 for v in label), dtype=np.int64, count=h * w).reshape(h, w)
    used = np.unique(out[out > 0])
    renumber = np.zeros(len(parent), dtype=np.int64)
    renumber[used] = np.arange(1, len(used) + 1)
    return renumber[out]


def _nearest_label(labels: np.ndarray, radius: float, res: float) -> np.ndarray:
    """For every cell, the label of the nearest labelled cell within `radius` (centre to centre); 0 if none."""
    h, w = labels.shape
    k = int(math.floor(radius / res + _EPS))
    padded = np.pad(labels, k)
    out = np.zeros_like(labels)
    for di, dj in sorted(disc_offsets(radius, res), key=lambda o: (o[0] * o[0] + o[1] * o[1], o)):
        shifted = padded[k + di:k + di + h, k + dj:k + dj + w]
        take = (out == 0) & (shifted > 0)
        out[take] = shifted[take]
    return out


class _PartMap:
    """Where each part is, the doorways between them, and the order to clean them in."""

    def __init__(self, grid: Grid, labels: np.ndarray, row_ok: np.ndarray, robot: Point, start: Tuple[int, int]):
        self.grid = grid
        self.labels = labels
        self.n = n = int(labels.max())
        res = grid.resolution
        ox, oy = grid.origin
        # A place in each part: its row-clear cell nearest the middle of its row-clear cells.
        self.centre: List[Point] = [robot] * (n + 1)
        rows, cols = np.nonzero(labels)
        lab = labels[rows, cols]
        prefer = row_ok[rows, cols]
        for p in range(1, n + 1):
            sel = lab == p
            if (sel & prefer).any():
                sel &= prefer
            r, c = rows[sel], cols[sel]
            i = int(np.argmin((r - r.mean()) ** 2 + (c - c.mean()) ** 2))
            self.centre[p] = grid.centre(int(r[i]), int(c[i]))
        # Doorways: the middle of the line along which two parts touch.
        sums: Dict[Tuple[int, int], List[float]] = {}
        for vertical in (True, False):
            a = labels[:-1, :] if vertical else labels[:, :-1]
            b = labels[1:, :] if vertical else labels[:, 1:]
            rr, cc = np.nonzero((a > 0) & (b > 0) & (a != b))
            la, lb = a[rr, cc], b[rr, cc]
            ys = (rr + (1.0 if vertical else 0.5)) * res + oy
            xs = (cc + (0.5 if vertical else 1.0)) * res + ox
            for p, q, x, y in zip(np.minimum(la, lb).tolist(), np.maximum(la, lb).tolist(), xs.tolist(), ys.tolist()):
                acc = sums.setdefault((p, q), [0.0, 0.0, 0.0])
                acc[0] += x
                acc[1] += y
                acc[2] += 1.0
        self.door = {k: (v[0] / v[2], v[1] / v[2]) for k, v in sorted(sums.items())}
        # How far apart the parts are: middle to doorway to middle, through the parts between.
        inf = float("inf")
        dist = [[0.0 if i == j else inf for j in range(n + 1)] for i in range(n + 1)]
        step = [[j if i == j else 0 for j in range(n + 1)] for i in range(n + 1)]  # the next part on the way
        for (p, q), d in self.door.items():
            length = math.dist(self.centre[p], d) + math.dist(d, self.centre[q])
            if length < dist[p][q]:
                dist[p][q] = dist[q][p] = length
                step[p][q], step[q][p] = q, p
        for k in range(1, n + 1):
            for i in range(1, n + 1):
                dik = dist[i][k]
                if dik == inf:
                    continue
                for j in range(1, n + 1):
                    if dik + dist[k][j] < dist[i][j] - 1e-12:
                        dist[i][j] = dik + dist[k][j]
                        step[i][j] = step[i][k]
        for i in range(1, n + 1):  # parts not joined through the others (an area can cut them apart)
            for j in range(1, n + 1):
                if dist[i][j] == inf:
                    dist[i][j] = 1.5 * math.dist(self.centre[i], self.centre[j]) + 1.0
                    step[i][j] = j
        self.dist = dist
        self.step = step
        # From the robot: through its own part, or straight to each part if it stands in none.
        own = int(labels[start]) if labels[start] else 0
        first = {}
        if own:
            first[own] = math.dist(robot, self.centre[own])
            for (p, q), d in self.door.items():
                if own in (p, q):
                    other = q if p == own else p
                    first[other] = min(first.get(other, inf), math.dist(robot, d) + math.dist(d, self.centre[other]))
        self.from_robot = [0.0] * (n + 1)
        for j in range(1, n + 1):
            via = [c + dist[i][j] for i, c in sorted(first.items())]
            self.from_robot[j] = min(via) if via else 1.5 * math.dist(robot, self.centre[j]) + 1.0

    def part_of_segment(self, seg: Tuple[Point, Point]) -> int:
        """The part a stretch of edge lies in: the one under most of it."""
        pts = sample_segment(seg[0], seg[1], self.grid.resolution)
        h, w = self.labels.shape
        rows = np.clip(np.floor((pts[:, 1] - self.grid.origin[1]) / self.grid.resolution).astype(int), 0, h - 1)
        cols = np.clip(np.floor((pts[:, 0] - self.grid.origin[0]) / self.grid.resolution).astype(int), 0, w - 1)
        found = self.labels[rows, cols]
        found = found[found > 0]
        if len(found):
            return int(np.argmax(np.bincount(found)))
        mid = ((seg[0][0] + seg[1][0]) / 2.0, (seg[0][1] + seg[1][1]) / 2.0)
        return min(range(1, self.n + 1), key=lambda p: (math.dist(mid, self.centre[p]), p))

    def towards(self, p: int, q: int) -> Point:
        """Where to leave part p for part q: the doorway to the next part on the way."""
        nxt = self.step[p][q]
        return self.door.get((min(p, nxt), max(p, nxt)), self.centre[q])

    def order(self, todo: List[int]) -> List[int]:
        """The parts in the order with the shortest trips, starting from the robot."""
        m = len(todo)
        if m <= 1:
            return list(todo)
        d = [[self.dist[a][b] for b in todo] for a in todo]
        d0 = [self.from_robot[a] for a in todo]
        if m <= _EXACT_ORDER_PARTS:
            seq = _shortest_path_order(d0, d)
        else:
            seq = [min(range(m), key=lambda j: (d0[j], j))]
            left = [j for j in range(m) if j != seq[0]]
            while left:
                j = min(left, key=lambda j: (d[seq[-1]][j], j))
                seq.append(j)
                left.remove(j)
            seq = _two_opt_path(seq, d0, d)
        return [todo[j] for j in seq]


def _shortest_path_order(d0: List[float], d: List[List[float]]) -> List[int]:
    """Every order weighed (Held and Karp): the open path from the start that visits all, shortest."""
    m = len(d0)
    inf = float("inf")
    full = 1 << m
    cost = [[inf] * m for _ in range(full)]
    back = [[-1] * m for _ in range(full)]
    for j in range(m):
        cost[1 << j][j] = d0[j]
    for mask in range(1, full):
        row = cost[mask]
        for j in range(m):
            here = row[j]
            if here == inf:
                continue
            for k in range(m):
                if mask & (1 << k):
                    continue
                nm = mask | (1 << k)
                v = here + d[j][k]
                if v < cost[nm][k] - 1e-12:
                    cost[nm][k] = v
                    back[nm][k] = j
    mask = full - 1
    j = min(range(m), key=lambda j: (cost[mask][j], j))
    seq = []
    while j >= 0:
        seq.append(j)
        mask, j = mask & ~(1 << j), back[mask][j]
    return seq[::-1]


def _two_opt_path(seq: List[int], d0: List[float], d: List[List[float]]) -> List[int]:
    """Reverse runs of an open path from the start while that shortens it."""
    improved = True
    while improved:
        improved = False
        for i in range(len(seq) - 1):
            for j in range(i + 1, len(seq)):
                before = (d0[seq[i]] if i == 0 else d[seq[i - 1]][seq[i]]) + (d[seq[j]][seq[j + 1]] if j + 1 < len(seq) else 0.0)
                after = (d0[seq[j]] if i == 0 else d[seq[i - 1]][seq[j]]) + (d[seq[i]][seq[j + 1]] if j + 1 < len(seq) else 0.0)
                if after < before - 1e-9:
                    seq[i:j + 1] = seq[i:j + 1][::-1]
                    improved = True
    return seq


def _improve(seq: List[Tuple[int, int]], ways, backwards, cost, start: int, goal: int, limit: int = 200):
    """Better orders of the items, found one move at a time (each time the move that saves most):
    a run of up to _MOVE_RUN items taken out and put back elsewhere, as it was or driven backwards
    (the items in the opposite order, each the other way round; a single item may be put back any
    way), or a run of any length driven backwards where it is (2-opt). Trips cost the same both ways."""
    for _ in range(limit):
        n = len(seq)
        if n < 2:
            break
        a = [ways[i][j][0] for i, j in seq]  # where each item starts
        b = [ways[i][j][1] for i, j in seq]  # and ends
        back_ok = [backwards[i][j] >= 0 for i, j in seq]
        best_gain, move = 1e-9, None
        for i in range(n):
            p0 = start if i == 0 else b[i - 1]
            from_p0 = cost[p0]
            reversible = True
            for j in range(i, n):
                reversible = reversible and back_ok[j]
                size = j - i + 1
                q0 = goal if j == n - 1 else a[j + 1]
                out = from_p0[a[i]] + cost[b[j]][q0] - from_p0[q0]
                if size > _MOVE_RUN:  # long runs: only backwards where they are
                    if reversible:
                        gain = out - from_p0[b[j]] - cost[a[i]][q0] + from_p0[q0]
                        if gain > best_gain:
                            best_gain, move = gain, (i, j, i, 1)
                    continue
                ends = [(x, y, k) for k, (x, y) in enumerate(ways[seq[i][0]])] if size == 1 else \
                    [(a[i], b[j], 0)] + ([(b[j], a[i], 1)] if reversible else [])
                for pos in range(n - size + 1):  # between what is then at pos - 1 and pos
                    p = start if pos == 0 else b[pos - 1] if pos - 1 < i else b[pos - 1 + size]
                    q = goal if pos == n - size else a[pos] if pos < i else a[pos + size]
                    from_p = cost[p]
                    gap = out + from_p[q]
                    for x, y, how in ends:
                        gain = gap - from_p[x] - cost[y][q]
                        if gain > best_gain:
                            best_gain, move = gain, (i, j, pos, how)
        if move is None:
            break
        i, j, pos, how = move
        run = seq[i:j + 1]
        del seq[i:j + 1]
        if i == j:
            run = [(run[0][0], how)]
        elif how == 1:
            run = [(x, backwards[x][y]) for x, y in reversed(run)]
        seq[pos:pos] = run
    return seq


def _length(seq, ways, cost, start: int, goal: int) -> float:
    total, at = 0.0, start
    for i, j in seq:
        total += cost[at][ways[i][j][0]]
        at = ways[i][j][1]
    return total + cost[at][goal]


def _nearest_first(ways, cost, start: int) -> List[Tuple[int, int]]:
    """Each time the item whose start is nearest to where the robot is."""
    seq: List[Tuple[int, int]] = []
    left = list(range(len(ways)))
    at = start
    while left:
        best = None
        for i in left:
            for j, (a, _) in enumerate(ways[i]):
                key = (cost[at][a], i, j)
                if best is None or key < best:
                    best = key
        _, i, j = best
        seq.append((i, j))
        left.remove(i)
        at = ways[i][j][1]
    return seq


def _best_ways(seq, ways, cost, start: int, goal: int) -> List[Tuple[int, int]]:
    """The items in the same order, each driven the way that makes the whole shortest (a shortest
    path through the ways of one item after another)."""
    if not seq:
        return seq
    best = [(cost[start][a], -1) for a, _ in ways[seq[0][0]]]
    back = [best]
    for k in range(1, len(seq)):
        prev = ways[seq[k - 1][0]]
        here = []
        for a, _ in ways[seq[k][0]]:
            here.append(min((back[-1][j][0] + cost[prev[j][1]][a], j) for j in range(len(prev))))
        back.append(here)
    last = ways[seq[-1][0]]
    j = min(range(len(last)), key=lambda j: (back[-1][j][0] + cost[last[j][1]][goal], j))
    out = []
    for k in range(len(seq) - 1, -1, -1):
        out.append((seq[k][0], j))
        j = back[k][j][1]
    return out[::-1]


# ---------------------------------------------------------------- pieces
def cleaned_by(grid: Grid, pieces: Sequence[Piece], radius: float) -> np.ndarray:
    """Cells the tool passes over on the rows, connectors and edges (transits are Nav2's to plan)."""
    mask = np.zeros(grid.shape, dtype=bool)
    for p in pieces:
        if p.kind != TRANSIT:
            stamp_segment(mask, grid, p.start, p.end, radius)
    return mask


def _clusters(positions: np.ndarray, split: int) -> List[Tuple[int, int]]:
    """Group sorted positions into (first, last) clusters, splitting at gaps larger than `split`."""
    out = []
    first = prev = int(positions[0])
    for p in positions[1:]:
        p = int(p)
        if p - prev > split:
            out.append((first, prev))
            first = p
        prev = p
    out.append((first, prev))
    return out


def _stretch(c0: int, c1: int, a: int, b: int, min_steps: int) -> Tuple[int, int]:
    """Lengthen [c0, c1] to at least min_steps cells' length, staying inside [a, b]."""
    short = min_steps - (c1 - c0)
    if short <= 0:
        return c0, c1
    c0 -= short // 2
    c1 += short - short // 2
    if c0 < a:
        c1 += a - c0
        c0 = a
    if c1 > b:
        c0 -= c1 - b
        c1 = b
    return max(c0, a), c1


def _merge_runs(found: List[Tuple[int, int]], ok: np.ndarray, max_gap: int) -> List[Tuple[int, int]]:
    """Join runs separated by at most `max_gap` cells, if the cells between are all allowed."""
    out: List[Tuple[int, int]] = []
    for a, b in found:
        if out and a - out[-1][1] - 1 <= max_gap and ok[out[-1][1] + 1:a].all():
            out[-1] = (out[-1][0], b)
        else:
            out.append((a, b))
    return out


def _chains(rows: List[_Row], spacing: int) -> List[List[_Row]]:
    """Split the rows into regions: chains in which each row has exactly one neighbour on the
    lines up to `spacing` above it (overlapping it), and is that neighbour's only one below."""
    by_line: Dict[int, List[int]] = {}
    for i, r in enumerate(rows):
        by_line.setdefault(r.line, []).append(i)
    ups: Dict[int, List[int]] = {i: [] for i in range(len(rows))}
    downs: Dict[int, List[int]] = {i: [] for i in range(len(rows))}
    for i, r in enumerate(rows):
        for line in range(r.line + 1, r.line + spacing + 1):
            for j in by_line.get(line, ()):
                q = rows[j]
                if max(r.a, q.a) <= min(r.b, q.b):
                    ups[i].append(j)
                    downs[j].append(i)
    nxt = {i: ups[i][0] for i in ups if len(ups[i]) == 1 and len(downs[ups[i][0]]) == 1}
    has_prev = set(nxt.values())
    chains = []
    for i in range(len(rows)):
        if i in has_prev:
            continue
        chain = [rows[i]]
        while i in nxt:
            i = nxt[i]
            chain.append(rows[i])
        chains.append(chain)
    return chains


def _entries(chain: List[_Row]) -> List[List[Tuple[_Row, int, int]]]:
    """The ways to drive a region: from its bottom or top row, starting at either end. Each is
    [(row, from, to), ...] in driving order."""
    out = []
    orders = [chain] if len(chain) == 1 else [chain, chain[::-1]]
    for order in orders:
        for left_first in (True, False):
            legs = []
            forward = left_first
            for row in order:
                legs.append((row, row.a, row.b) if forward else (row, row.b, row.a))
                forward = not forward
            out.append(legs)
    return out


def _number(pieces: List[Piece]) -> List[Piece]:
    out = []
    rows = edges = 0
    for p in pieces:
        if p.kind == ROW:
            rows += 1
            p = Piece(p.kind, p.start, p.end, rows, p.pass_number, p.region)
        elif p.kind == EDGE:
            edges += 1
            p = Piece(p.kind, p.start, p.end, edges, 0, 0)
        out.append(p)
    return out


def _count(pieces: List[Piece], kind: str) -> int:
    return sum(1 for p in pieces if p.kind == kind)


def _turns(pieces: List[Piece], here: Point) -> int:
    """Changes of heading larger than 30 degrees from one piece to the next."""
    turns = 0
    heading = None
    for p in pieces:
        if p.length < _EPS:
            continue
        h = p.heading
        if heading is not None:
            change = abs(math.atan2(math.sin(h - heading), math.cos(h - heading)))
            if change > _TURN_ANGLE:
                turns += 1
        heading = h
    return turns


def count_turns(pieces: Sequence[Piece]) -> int:
    return _turns(list(pieces), (0.0, 0.0))


def _estimate(pieces, robot, s: Settings, res, planned, left_out, unreachable) -> Estimate:
    driven = sum(p.length for p in pieces if p.kind != TRANSIT)
    transit = sum(p.length for p in pieces if p.kind == TRANSIT) * s.transit_detour
    turns = _turns(pieces, robot)
    cell = res * res
    return Estimate(
        rows=_count(pieces, ROW),
        edges=_count(pieces, EDGE),
        connectors=_count(pieces, CONNECTOR),
        transits=_count(pieces, TRANSIT),
        turns=turns,
        distance=driven + transit,
        planned_area=float(planned.sum()) * cell,
        left_out_area=float(left_out.sum()) * cell,
        unreachable_area=float(unreachable.sum()) * cell,
        time=driven / s.cleaning_speed + transit / s.transit_speed + turns * s.turn_time,
    )


def remaining_estimate(pieces: Sequence[Piece], s: Settings) -> Tuple[float, int]:
    """(distance, turns) still to drive for the given pieces, transits estimated."""
    distance = sum(p.length * (s.transit_detour if p.kind == TRANSIT else 1.0) for p in pieces)
    return distance, count_turns(pieces)


def restart_from(pieces: Sequence[Piece], robot: Point) -> List[Piece]:
    """A saved plan, driven again from where the robot is now: whatever led to its first row or
    edge is replaced by one transit from the robot."""
    for i, p in enumerate(pieces):
        if p.kind in (ROW, EDGE):
            return [Piece(TRANSIT, (float(robot[0]), float(robot[1])), p.start)] + list(pieces[i:])
    return []

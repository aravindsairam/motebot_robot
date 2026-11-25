"""The rooms of a map: found by splitting the floor at its doorways, kept with the map in a file,
named by the user or by the robot. No ROS imports.

Finding them uses the same split as the cleaning planner (planner._split): a watershed on the
distance from the walls, where a passage much narrower than the rooms on both sides (a doorway)
keeps them apart. Around it:
  1. furniture standing free on the floor (legs, a sofa, stray dots on the map: see _furniture)
     counts as floor while the rooms are found, so that only the walls shape them;
  2. every floor cell joins the part of the nearest place the robot can stand;
  3. parts with less floor than `min_area` (a nook behind a narrow gap) join the neighbour they
     share the longest border with;
  4. doorways are looked for: gaps in straight wall lines, whichever way the map is turned (see
     _doorways). A doorway inside one part that cuts it into two rooms makes two of it (a hallway
     as narrow as its doors is not kept apart by the split alone), and where two rooms meet, the
     border is moved onto the doorway between them (the split alone puts it where the two floods
     met, which can be well inside one room);
  5. each room gets an outline (a polygon around its floor), its most open spot (where the robot goes
     to visit it: just inside its widest doorway, see _entry_spot) and its floor area.

No split is right every time, so rooms can also be changed by hand: two that touch joined into one
(join), or one cut in two along a line (cut). Both work on rooms known by their outlines alone, as
the app holds them.

The file, next to the map (map_files.rooms_path):
    rooms:
      - {name: Kitchen, outline: [[x, y], ...], x: 8.5, y: 1.6, area: 12.8}
"""

import math
import os
import re
import tempfile
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np
import yaml

from .grid import Grid, clearance_field, dilate, polygon_mask
from .planner import _PART_CAP, _nearest_label, _split

MIN_AREA = 2.0  # m2: less floor than this is not a room of its own
NECK = 0.7  # rooms split where a passage is narrower than this times the smaller room's widest place
DOOR_WIDTH = (0.5, 2.0)  # m: a gap this wide in a straight wall line can be a doorway or an arch
DOOR_WALL = 0.4  # m of wall on each side of it, in line
FURNITURE = 4.0  # m2: a thing standing free on the floor, up to this big, is furniture and not a wall,
WALL = (0.4, 3.0)  # unless it is long and thin like a wall: at least this long (m), and this many times longer than thick
HALLWAY = 0.5  # m: walls that run on this far from both ends of a gap, to one side, make it a stretch of hallway
STRAIGHT = 0.25  # degrees: walls turned less than this from the map's rows and columns are along them
SMALLEST = 0.5  # m2: a room cut in two by hand keeps at least this much floor on each side
MAX_ROOMS = 30
MAX_NAME = 40
STAND_CLEARANCE = 0.20  # m from the walls where the robot can stand: its radius and a little more
_NEW_NAME = re.compile(r"^Room (\d+)$")


@dataclass
class Room:
    name: str
    outline: List[Tuple[float, float]]  # map metres, in order around the floor
    x: float  # the most open spot
    y: float
    area: float  # m2 of floor
    cells: Optional[np.ndarray] = field(default=None, repr=False, compare=False)  # bool mask, when found from a map

    def to_dict(self) -> dict:
        return {"name": self.name, "outline": [[round(float(x), 3), round(float(y), 3)] for x, y in self.outline],
                "x": round(float(self.x), 3), "y": round(float(self.y), 3), "area": round(float(self.area), 2)}

    @staticmethod
    def from_dict(d: dict) -> "Room":
        return Room(name=str(d["name"]), outline=[(float(p[0]), float(p[1])) for p in d["outline"]],
                    x=float(d["x"]), y=float(d["y"]), area=float(d.get("area", 0.0)))


# ---------------------------------------------------------------- finding
def find(grid: Grid, min_area: float = MIN_AREA, walls: Optional[np.ndarray] = None) -> List[Room]:
    """The rooms of the floor in `grid`, biggest first, named "Room 1", "Room 2", ... `walls`: the
    occupied cells of the map, for finding doorways (without it the borders are not moved)."""
    res = grid.resolution
    cell_area = res * res
    furniture = _furniture(grid.floor, res)
    ground = grid.floor | furniture  # the floor the walls alone leave: this is what is split into rooms
    height = clearance_field(ground, res, _PART_CAP)
    stand = height >= STAND_CLEARANCE - 1e-9
    parts = _split(stand, height, NECK)
    floor_parts = np.where(ground, _nearest_label(parts, STAND_CLEARANCE + 2 * res, res), 0)

    # Small parts join the neighbour they share the longest border with, smallest first.
    while True:
        ids, counts = np.unique(floor_parts[floor_parts > 0], return_counts=True)
        small = [(c, i) for i, c in zip(ids.tolist(), counts.tolist()) if c * cell_area < min_area]
        if not small:
            break
        _, part = min(small)
        mask = floor_parts == part
        borders = _neighbours(floor_parts, mask)
        if borders:
            target = max(borders.items(), key=lambda kv: (kv[1], -kv[0]))[0]
            floor_parts[mask] = target
        else:
            floor_parts[mask] = 0  # alone and too small: a cupboard, not a room

    if walls is not None:
        floor_parts = _at_doorways(floor_parts, ground, res, walls & ~furniture, min_area)
    floor_parts = np.where(grid.floor, floor_parts, 0)
    height = clearance_field(grid.floor, res, _PART_CAP)  # from here on: where the robot can really stand

    rooms: List[Room] = []
    for part in np.unique(floor_parts[floor_parts > 0]).tolist():
        cells = floor_parts == part
        room = _room_of(cells, (floor_parts > 0) & ~cells, height, grid)
        if room is not None:
            rooms.append(room)
    rooms.sort(key=lambda room: (-room.area, room.x, room.y))
    for i, room in enumerate(rooms, start=1):
        room.name = f"Room {i}"
    return rooms


def _furniture(floor: np.ndarray, res: float) -> np.ndarray:
    """The cells that are not floor but stand free on it: table and chair legs, a sofa, a kitchen
    island, a patch the robot never saw into, stray dots on the map. Each is a group of cells that
    touches neither the walls around the home nor the edge of the map, is at most FURNITURE big, and
    is not long and thin like a piece of wall with a door at each end (WALL)."""
    import cv2

    h, w = floor.shape
    count, comp, stats, _ = cv2.connectedComponentsWithStats((~floor).astype(np.uint8), connectivity=8)
    out = np.zeros_like(floor)
    for i in range(1, count):
        x, y, bw, bh, cells = (int(v) for v in stats[i])
        if x == 0 or y == 0 or x + bw == w or y + bh == h or cells * res * res > FURNITURE:
            continue
        piece = comp[y:y + bh, x:x + bw] == i
        if math.hypot(bw, bh) * res >= WALL[0]:  # long enough to be a wall: is it thin as well?
            long_side = max(cv2.minAreaRect(np.argwhere(piece).astype(np.float32))[1]) + 1
            thick = 2.0 * float(cv2.distanceTransform(np.pad(piece, 1).astype(np.uint8), cv2.DIST_L2, 5).max())
            if long_side * res >= WALL[0] and long_side >= WALL[1] * thick:
                continue
        out[y:y + bh, x:x + bw] |= piece
    return out


def _room_of(cells: np.ndarray, other: np.ndarray, height: np.ndarray, grid: Grid) -> Optional[Room]:
    """The room with this floor, not yet named: its outline, the spot to visit it and its area.
    `other`: the floor of the other rooms; `height`: how far each cell is from the walls. None when
    the floor is too thin to have an outline."""
    outline = outline_of(cells, grid)
    if len(outline) < 3:
        return None
    spot = _entry_spot(cells, other, height, grid)
    if spot is None:  # no doorway: its most open place where the robot can stand, or failing that, of all its floor
        for where in (cells & (height >= STAND_CLEARANCE - 1e-9), cells):
            open_ = np.where(where, height, -1.0)
            spot = np.unravel_index(int(np.argmax(open_)), open_.shape)
            if open_[spot] >= 0:
                break
    x, y = grid.centre(*spot)
    return Room("", outline, x, y, float(cells.sum()) * grid.resolution ** 2, cells)


def _entry_spot(cells: np.ndarray, other: np.ndarray, height: np.ndarray, grid: Grid) -> Optional[Tuple[int, int]]:
    """Where a person would stand to see the room: just inside its widest doorway, toward the room's
    middle. (The room's most open place on the map can be under a bed or a table: the lidar sees only
    their legs.) `other`: the floor of the other rooms. None when the room has no neighbour."""
    import cv2

    res = grid.resolution
    border = cells & dilate(other, res, res)
    if not border.any():
        return None
    count, comp = cv2.connectedComponents(border.astype(np.uint8), connectivity=8)
    sizes = np.bincount(comp.ravel(), minlength=count)
    sizes[0] = 0
    door = comp == int(np.argmax(sizes))
    dr, dc = (float(v.mean()) for v in np.nonzero(door))
    mr, mc = (float(v.mean()) for v in np.nonzero(cells))
    length = math.hypot(mr - dr, mc - dc)
    if length < 1e-9:
        return None
    best = None
    for d in np.arange(0.6, 2.01, 0.1) / res:  # 0.6 to 2 m in from the doorway
        r, c = int(round(dr + (mr - dr) * d / length)), int(round(dc + (mc - dc) * d / length))
        if not (0 <= r < cells.shape[0] and 0 <= c < cells.shape[1]) or not cells[r, c]:
            continue
        if height[r, c] >= STAND_CLEARANCE + 0.1 - 1e-9:
            score = -abs(d * res - 1.0)  # about a metre in
            if best is None or score > best[0]:
                best = (score, (r, c))
    return best[1] if best else None


def _doorways(walls: np.ndarray, floor: np.ndarray, res: float) -> List[Tuple[np.ndarray, np.ndarray]]:
    """Gaps in straight wall lines (see _wall_gaps), each as the (rows, columns) of its cells. A map
    whose walls do not run along its rows and columns (the robot started it askew) is turned
    straight first, at twice the detail, and the gaps found there are laid back onto the map."""
    import cv2

    turn = _turn_to_straighten(walls)
    if abs(turn) < STRAIGHT:
        return _wall_gaps(walls, floor, res)
    h, w = walls.shape
    scale = 2
    side = scale * int(math.ceil(math.hypot(h, w))) + 4
    to_sheet = cv2.getRotationMatrix2D(((w - 1) / 2, (h - 1) / 2), turn, scale)
    to_sheet[:, 2] += ((side - 1) / 2 - (w - 1) / 2, (side - 1) / 2 - (h - 1) / 2)
    sheet_walls = cv2.warpAffine(walls.astype(np.uint8), to_sheet, (side, side), flags=cv2.INTER_NEAREST)
    sheet_walls = cv2.dilate(sheet_walls, np.ones((3, 3), np.uint8)).astype(bool)  # no pinholes where a turned wall steps
    sheet_floor = cv2.warpAffine(floor.astype(np.uint8), to_sheet, (side, side), flags=cv2.INTER_NEAREST).astype(bool)
    back = cv2.invertAffineTransform(to_sheet)
    found = []
    for rows, cols in _wall_gaps(sheet_walls, sheet_floor & ~sheet_walls, res / scale):
        n = len(rows) + 2 * scale  # one map cell more at each end, into the wall
        ys = np.linspace(rows[0] - scale * np.sign(rows[-1] - rows[0]), rows[-1] + scale * np.sign(rows[-1] - rows[0]), n)
        xs = np.linspace(cols[0] - scale * np.sign(cols[-1] - cols[0]), cols[-1] + scale * np.sign(cols[-1] - cols[0]), n)
        map_cols = np.rint(back[0, 0] * xs + back[0, 1] * ys + back[0, 2]).astype(int)
        map_rows = np.rint(back[1, 0] * xs + back[1, 1] * ys + back[1, 2]).astype(int)
        inside = (map_rows >= 0) & (map_rows < h) & (map_cols >= 0) & (map_cols < w)
        if inside.any():
            found.append((map_rows[inside], map_cols[inside]))
    return found


def _turn_to_straighten(walls: np.ndarray) -> float:
    """The turn, in degrees from -45 to 45, that lays the walls along the rows and columns of the
    picture: the one after which the wall cells fall into the fewest rows and columns (the sum of
    the squared counts per row and per column is then the largest). 0 for a map with no walls."""
    import cv2

    if not walls.any():
        return 0.0
    h, w = walls.shape
    side = int(math.ceil(math.hypot(h, w))) + 2
    picture = walls.astype(np.float32)

    def evenness(turn: float) -> float:
        to_sheet = cv2.getRotationMatrix2D(((w - 1) / 2, (h - 1) / 2), float(turn), 1.0)
        to_sheet[:, 2] += ((side - 1) / 2 - (w - 1) / 2, (side - 1) / 2 - (h - 1) / 2)
        turned = cv2.warpAffine(picture, to_sheet, (side, side), flags=cv2.INTER_LINEAR)
        return float((turned.sum(axis=0) ** 2).sum() + (turned.sum(axis=1) ** 2).sum())

    coarse = max(np.arange(-45.0, 45.0, 1.0).tolist(), key=evenness)
    return float(max((round(coarse + step / 10.0, 1) for step in range(-9, 10)), key=evenness))


def _wall_gaps(walls: np.ndarray, floor: np.ndarray, res: float) -> List[Tuple[np.ndarray, np.ndarray]]:
    """Gaps in straight wall lines: along a row or a column of the picture, a stretch of floor
    between two runs of wall each at least DOOR_WALL long, the stretch DOOR_WIDTH wide. Each as the
    (rows, columns) of its cells. Not a gap: the floor of a hallway where the walls between the
    rooms on its two sides line up across it. There, unlike at a door, walls run on from both ends
    of the gap, square to the line, for HALLWAY or more."""
    lo = int(math.ceil(DOOR_WIDTH[0] / res - 1e-9))
    hi = int(math.floor(DOOR_WIDTH[1] / res + 1e-9))
    wall_run = int(math.ceil(DOOR_WALL / res - 1e-9))
    reach = int(math.ceil(HALLWAY / res - 1e-9))
    found = []
    for transpose in (False, True):
        w = walls.T if transpose else walls
        f = floor.T if transpose else floor
        for line in range(w.shape[0]):
            runs_ = _true_runs(w[line])
            for (a0, a1), (b0, b1) in zip(runs_, runs_[1:]):
                gap = b0 - a1 - 1
                if not (lo <= gap <= hi) or a1 - a0 + 1 < wall_run or b1 - b0 + 1 < wall_run:
                    continue
                if f[line, a1 + 1:b0].mean() < 0.8:  # mostly floor, not unknown space or clutter
                    continue
                above = w[line + 1:line + 1 + reach, [a1, b0]]
                below = w[max(line - reach, 0):line, [a1, b0]]
                if any(len(side) == reach and (side.mean(axis=0) >= 0.8).all() for side in (above, below)):
                    continue  # hallway walls on both ends
                along = np.arange(a1 + 1, b0)
                fixed = np.full(len(along), line)
                found.append((along, fixed) if transpose else (fixed, along))
    return found


def _true_runs(values: np.ndarray) -> List[Tuple[int, int]]:
    padded = np.concatenate(([False], values.astype(bool), [False]))
    edges = np.flatnonzero(padded[1:] != padded[:-1])
    return [(int(a), int(b) - 1) for a, b in zip(edges[0::2], edges[1::2])]


def _at_doorways(labels: np.ndarray, floor: np.ndarray, res: float, walls: np.ndarray, min_area: float) -> np.ndarray:
    """The rooms in `labels` put right at the doorways. First, a doorway inside one room that cuts
    it into two rooms of at least `min_area` each makes two of it. Then, for each two rooms that
    meet, the border moves onto the doorway that cuts their floor in two and matches the two best."""
    doors = _doorways(walls, floor, res)
    if not doors:
        return labels
    out = labels.copy()
    min_cells = min_area / (res * res)

    def give(region: np.ndarray, into_a: np.ndarray, a: int, into_b: np.ndarray, b: int) -> None:
        rest = region & ~into_a & ~into_b  # the cut itself and small pieces: to the nearest side
        out[into_a], out[into_b] = a, b
        if rest.any():
            near = _nearest_label(np.where(into_a | into_b, out, 0), 3 * res, res)
            out[rest] = np.where(near[rest] > 0, near[rest], out[rest])

    for rows, cols in doors:
        for a in np.unique(out[rows, cols]).tolist():
            halves = _cut_in_two(out == a, rows, cols, res, min_cells) if a > 0 else None
            if halves is not None:
                give(out == a, halves[0], a, halves[1], int(out.max()) + 1)

    ids = [int(i) for i in np.unique(out[out > 0])]
    for a in ids:
        for b, _ in sorted(_neighbours(out, out == a).items()):
            if b <= a:
                continue
            union = (out == a) | (out == b)
            was_a, was_b = out == a, out == b
            best = None
            for rows, cols in doors:
                halves = _cut_in_two(union, rows, cols, res, min_cells) if union[rows, cols].any() else None
                if halves is None:
                    continue
                one, two = halves
                keep = float((one & was_a).sum() + (two & was_b).sum())
                swap = float((one & was_b).sum() + (two & was_a).sum())
                if best is None or max(keep, swap) > best[0]:
                    best = (max(keep, swap), (one, two) if keep >= swap else (two, one))
            if best is not None:
                give(union, best[1][0], a, best[1][1], b)
    return out


def _cut_in_two(region: np.ndarray, rows: np.ndarray, cols: np.ndarray, res: float, min_cells: float):
    """The two pieces of at least `min_cells` that the doorway at (rows, cols) cuts `region` into,
    or None when it does not leave exactly two such pieces."""
    import cv2

    cut = np.zeros_like(region)
    cut[rows, cols] = True
    cut = dilate(cut, res, res)  # a wall-thick line: no corner-to-corner leak through it
    count, comp = cv2.connectedComponents((region & ~cut).astype(np.uint8), connectivity=4)
    sizes = np.bincount(comp.ravel(), minlength=count)
    sizes[0] = 0
    big = [k for k in range(1, count) if sizes[k] >= min_cells]
    return (comp == big[0], comp == big[1]) if len(big) == 2 else None


def _neighbours(labels: np.ndarray, mask: np.ndarray) -> dict:
    """{label: cells of border shared with `mask`} over side-by-side neighbours."""
    out = {}
    for shifted_mask, shifted_labels in ((mask[1:, :], labels[:-1, :]), (mask[:-1, :], labels[1:, :]),
                                         (mask[:, 1:], labels[:, :-1]), (mask[:, :-1], labels[:, 1:])):
        touching = shifted_labels[shifted_mask]
        for label, count in zip(*np.unique(touching[touching > 0], return_counts=True)):
            out[int(label)] = out.get(int(label), 0) + int(count)
    mine = labels[mask]
    for label in np.unique(mine[mine > 0]).tolist():
        out.pop(int(label), None)
    return out


def outline_of(cells: np.ndarray, grid: Grid, tolerance: float = 0.10) -> List[Tuple[float, float]]:
    """A polygon around the cells (the biggest piece of them), simplified to within `tolerance`. It
    runs through the centres of the cells just outside them, so it holds the room's floor give or
    take the simplification."""
    import cv2

    grown = dilate(cells, grid.resolution * 1.5, grid.resolution).astype(np.uint8)
    contours, _ = cv2.findContours(grown, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return []
    contour = max(contours, key=cv2.contourArea)
    simple = cv2.approxPolyDP(contour, tolerance / grid.resolution, True).reshape(-1, 2)
    if len(simple) < 3:
        return []
    return [grid.centre(row, col) for col, row in simple.tolist()]


# ---------------------------------------------------------------- changing by hand
def join(rooms: Sequence[Room], a: int, b: int, grid: Grid) -> List[Room]:
    """The list with rooms `a` and `b` (their places in it, from 0) made one room, under the name
    and in the place of the bigger. The other rooms are left as they are. Raises ValueError, in
    words for the user, when these are not two rooms that touch."""
    import cv2

    if not (0 <= a < len(rooms) and 0 <= b < len(rooms)) or a == b:
        raise ValueError("pick two different rooms to join")
    res = grid.resolution
    cells = _floors(rooms, grid)
    seam = dilate(cells[a], 2 * res, res) & dilate(cells[b], 2 * res, res) & grid.floor  # the doorway between them
    count, comp = cv2.connectedComponents((cells[a] | cells[b] | seam).astype(np.uint8), connectivity=8)
    piece = int(np.bincount(comp[cells[a]], minlength=count).argmax()) if seam.any() else 0
    if piece == 0 or piece != int(np.bincount(comp[cells[b]], minlength=count).argmax()):
        raise ValueError(f"'{rooms[a].name}' and '{rooms[b].name}' do not touch")
    keep, gone = (a, b) if rooms[a].area >= rooms[b].area else (b, a)
    other = np.zeros(grid.shape, dtype=bool)
    for i, c in enumerate(cells):
        if i not in (a, b):
            other |= c
    joined = _room_of(comp == piece, other, clearance_field(grid.floor, res, _PART_CAP), grid)
    if joined is None:
        raise ValueError(f"'{rooms[a].name}' and '{rooms[b].name}' do not touch")
    joined.name = rooms[keep].name
    out = list(rooms)
    out[keep] = joined
    del out[gone]
    return out


def cut(rooms: Sequence[Room], line: Tuple[Tuple[float, float], Tuple[float, float]], grid: Grid) -> List[Room]:
    """The list with one room cut in two along `line` ((x0, y0), (x1, y1), map metres): the room
    most of the line lies in. The line is carried on from both ends to the room's walls, through
    furniture standing in the room, so it need not reach them. The bigger part keeps the name and
    the place in the list; the other comes right after it, as "Room n". Raises ValueError, in words
    for the user, when the line does not cut a room into two parts of at least SMALLEST each."""
    import cv2

    if len(rooms) >= MAX_ROOMS:
        raise ValueError(f"at most {MAX_ROOMS} rooms")
    res = grid.resolution
    (x0, y0), (x1, y1) = line
    if not all(math.isfinite(v) for v in (x0, y0, x1, y1)):
        raise ValueError("the line is not in a room")
    length = math.hypot(x1 - x0, y1 - y0)
    steps = max(int(math.ceil(2 * length / res)), 1)  # two looks per cell along the line
    between = [grid.cell_of(x0 + (x1 - x0) * k / steps, y0 + (y1 - y0) * k / steps) for k in range(steps + 1)]
    between = [cell for cell in between if grid.contains(*cell)]
    cells = _floors(rooms, grid)
    on_line = [sum(bool(c[cell]) for cell in between) for c in cells]
    if not on_line or max(on_line) == 0:
        raise ValueError("the line is not in a room")
    i = int(np.argmax(on_line))
    whole = _holes_filled(cells[i])
    rows, cols = [], []

    def take(along: float) -> bool:
        row, col = grid.cell_of(x0 + (x1 - x0) * along / length, y0 + (y1 - y0) * along / length)
        if not grid.contains(row, col) or not whole[row, col]:
            return False
        rows.append(row)
        cols.append(col)
        return True

    if length > 0:
        for k in range(steps + 1):
            take(length * k / steps)
        for step, end in ((-res / 2, 0.0), (res / 2, length)):  # on from each end, to the wall
            along = end
            while take(along):
                along += step
    halves = _cut_in_two(cells[i], np.array(rows), np.array(cols), res, SMALLEST / (res * res)) if rows else None
    if halves is None:
        raise ValueError("the line does not cut a room in two: put its ends near two walls of the room")
    # The cut itself and small pieces go to the nearer part.
    far = [cv2.distanceTransform((~half).astype(np.uint8), cv2.DIST_L2, 3) for half in halves]
    one = cells[i] & (far[0] <= far[1])
    big, small = sorted((one, cells[i] & ~one), key=lambda part: -int(part.sum()))
    other = np.zeros(grid.shape, dtype=bool)
    for k, c in enumerate(cells):
        if k != i:
            other |= c
    height = clearance_field(grid.floor, res, _PART_CAP)
    first, second = _room_of(big, other | small, height, grid), _room_of(small, other | big, height, grid)
    if first is None or second is None:
        raise ValueError("the line does not cut a room in two: put its ends near two walls of the room")
    first.name = rooms[i].name
    taken = {" ".join(room.name.split()).lower() for room in rooms}
    second.name = next(f"Room {n}" for n in range(1, len(rooms) + 2) if f"room {n}" not in taken)
    return list(rooms[:i]) + [first, second] + list(rooms[i + 1:])


def _floors(rooms: Sequence[Room], grid: Grid) -> List[np.ndarray]:
    """The floor of each room: as found, or for a room known by its outline alone (from the file or
    the app), the floor inside it. An outline runs through the cells just outside its room's floor
    (outline_of), so where two rooms meet, each outline holds a strip of the other room: there the
    floor goes to the room whose outline, drawn in by that one ring of cells, still holds it."""
    import cv2

    res = grid.resolution
    inside, core = [], []
    for room in rooms:
        picture = np.zeros(grid.shape, dtype=np.uint8)
        if len(room.outline) >= 3:
            corners = [[round((x - grid.origin[0]) / res - 0.5), round((y - grid.origin[1]) / res - 0.5)] for x, y in room.outline]
            cv2.fillPoly(picture, [np.array(corners, dtype=np.int32)], 1)
        inside.append(picture.astype(bool) & grid.floor)
        core.append(cv2.erode(picture, np.ones((3, 3), np.uint8)).astype(bool))
    out = []
    for i, room in enumerate(rooms):
        if room.cells is not None:
            out.append(room.cells)
            continue
        others = np.zeros(grid.shape, dtype=bool)
        for k, c in enumerate(core):
            if k != i:
                others |= c
        out.append(inside[i] & ~others)
    return out


def _holes_filled(cells: np.ndarray) -> np.ndarray:
    """The cells and what they close in on every side: the furniture standing in a room."""
    import cv2

    _, comp = cv2.connectedComponents(np.pad(~cells, 1, constant_values=True).astype(np.uint8), connectivity=4)
    return (comp != comp[0, 0])[1:-1, 1:-1]


# ---------------------------------------------------------------- names
def keep_names(found: List[Room], saved: Sequence[Room], grid: Grid, overlap: float = 0.5) -> List[Room]:
    """Found rooms that cover the same floor as a saved room (at least `overlap` of either) take its
    name; the others are "Room n", with numbers no saved or kept name uses."""
    taken = set()
    for room in found:
        if room.cells is None:
            continue
        best, best_share = None, 0.0
        for old in saved:
            if old.name in taken:
                continue
            old_cells = polygon_mask(grid, old.outline) & grid.floor
            both = float((old_cells & room.cells).sum())
            share = max(both / max(float(old_cells.sum()), 1.0), both / max(float(room.cells.sum()), 1.0))
            if share >= overlap and share > best_share:
                best, best_share = old, share
        if best is not None:
            room.name = best.name
            taken.add(best.name)
        else:
            room.name = ""
    used = {int(m.group(1)) for m in (_NEW_NAME.match(n) for n in taken) if m}
    n = 0
    for room in found:
        if not room.name:
            n += 1
            while n in used:
                n += 1
            room.name = f"Room {n}"
    return found


def problem(rooms: Sequence[Room]) -> str:
    """Why this list of rooms cannot be saved, or ""."""
    if len(rooms) > MAX_ROOMS:
        return f"at most {MAX_ROOMS} rooms"
    seen = set()
    for room in rooms:
        name = room.name.strip()
        if not name:
            return "every room needs a name"
        if len(name) > MAX_NAME:
            return f"the name '{name[:20]}...' is longer than {MAX_NAME} characters"
        if name.lower() in seen:
            return f"two rooms are called '{name}'"
        seen.add(name.lower())
        if len(room.outline) < 3:
            return f"the room '{name}' has no outline"
        values = [v for p in room.outline for v in p] + [room.x, room.y, room.area]
        if not all(math.isfinite(v) for v in values):
            return f"the room '{name}' has a corner that is not a number"
    return ""


def by_name(rooms: Sequence[Room], name: str) -> Optional[Room]:
    """The room called `name` (letter case and spaces at the ends do not matter)."""
    wanted = " ".join(name.split()).lower()
    for room in rooms:
        if " ".join(room.name.split()).lower() == wanted:
            return room
    return None


# ---------------------------------------------------------------- the file
def read(path: str) -> List[Room]:
    """The rooms in a rooms file; none if there is no file. Raises on a broken file."""
    if not os.path.exists(path):
        return []
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return [Room.from_dict(d) for d in data.get("rooms") or []]


def write(path: str, rooms: Sequence[Room]) -> None:
    directory = os.path.dirname(os.path.abspath(path))
    fd, temporary = tempfile.mkstemp(prefix="." + os.path.basename(path) + ".", suffix=".tmp", dir=directory)
    try:
        with os.fdopen(fd, "w") as f:
            f.write("# The rooms of this map: found from the map, named by the user or the robot.\n")
            yaml.safe_dump({"rooms": [r.to_dict() for r in rooms]}, f, sort_keys=False, allow_unicode=True)
        os.replace(temporary, path)
    except BaseException:
        if os.path.exists(temporary):
            os.unlink(temporary)
        raise

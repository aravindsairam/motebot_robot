"""The map as a grid of cells, and the few operations on it that planning and recording need.
No ROS imports.

Cells are indexed (row, column) with row 0 at the bottom of the map (y = origin y), as in an
OccupancyGrid. The centre of cell (row, column) is at origin + ((column + 0.5), (row + 0.5)) x resolution.

Distances to obstacles are measured from a point to the nearest point of an obstacle cell's
square, not to its centre: a wall cell occupies its whole square.
"""

import math
from collections import deque
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

Point = Tuple[float, float]


@dataclass(frozen=True, eq=False)
class Grid:
    floor: np.ndarray  # bool (height, width): True where the robot may be (free, known, not a no-go zone)
    resolution: float  # metres per cell
    origin: Tuple[float, float]  # map coordinates of the outer corner of cell (0, 0)

    @property
    def shape(self) -> Tuple[int, int]:
        return self.floor.shape

    def cell_of(self, x: float, y: float) -> Tuple[int, int]:
        """The (row, column) containing the point. It may lie outside the grid."""
        return (int(math.floor((y - self.origin[1]) / self.resolution)), int(math.floor((x - self.origin[0]) / self.resolution)))

    def centre(self, row: float, col: float) -> Point:
        return (self.origin[0] + (col + 0.5) * self.resolution, self.origin[1] + (row + 0.5) * self.resolution)

    def contains(self, row: int, col: int) -> bool:
        return 0 <= row < self.floor.shape[0] and 0 <= col < self.floor.shape[1]


def _square_distance(di: int, dj: int, res: float) -> float:
    """From a cell's centre to the nearest point of the square of the cell (di, dj) away."""
    return math.hypot(max(abs(di) - 0.5, 0.0), max(abs(dj) - 0.5, 0.0)) * res


def clearance_field(floor: np.ndarray, res: float, cap: float) -> np.ndarray:
    """For every cell centre, the distance to the nearest cell that is not floor (outside the map
    counts as not floor), exactly, up to `cap`; anything farther reads `cap`."""
    h, w = floor.shape
    k = int(math.ceil(cap / res)) + 1
    blocked = np.pad(~floor, k, constant_values=True)
    groups = {}
    for di in range(-k, k + 1):
        for dj in range(-k, k + 1):
            d = _square_distance(di, dj, res)
            if d < cap:
                groups.setdefault(round(d, 12), []).append((di, dj))
    out = np.full((h, w), cap, dtype=np.float64)
    unset = np.ones((h, w), dtype=bool)
    for d in sorted(groups):  # nearest first: the first hit is the distance
        hit = np.zeros((h, w), dtype=bool)
        for di, dj in groups[d]:
            hit |= blocked[k + di:k + di + h, k + dj:k + dj + w]
        hit &= unset
        out[hit] = d
        unset &= ~hit
    return out


def disc_offsets(radius: float, res: float) -> List[Tuple[int, int]]:
    """Cell offsets whose centres lie within `radius` of a cell's centre."""
    k = int(math.floor(radius / res + 1e-9))
    return [(di, dj) for di in range(-k, k + 1) for dj in range(-k, k + 1) if (di * di + dj * dj) * res * res <= radius * radius + 1e-12]


def dilate(mask: np.ndarray, radius: float, res: float) -> np.ndarray:
    """Cells whose centre is within `radius` of the centre of a True cell."""
    h, w = mask.shape
    k = int(math.floor(radius / res + 1e-9))
    padded = np.pad(mask, k)
    out = np.zeros_like(mask, dtype=bool)
    for di, dj in disc_offsets(radius, res):
        out |= padded[k + di:k + di + h, k + dj:k + dj + w]
    return out


def small_blobs(mask: np.ndarray, max_cells: int) -> np.ndarray:
    """The True cells that belong to groups (joined side by side or corner to corner) of at most
    `max_cells` cells that do not touch the edge of the grid: furniture legs and the like."""
    h, w = mask.shape
    seen = np.zeros_like(mask, dtype=bool)
    out = np.zeros_like(mask, dtype=bool)
    for r0, c0 in zip(*np.nonzero(mask)):
        if seen[r0, c0]:
            continue
        seen[r0, c0] = True
        stack, cells, edge = [(r0, c0)], [], False
        while stack:
            r, c = stack.pop()
            cells.append((r, c))
            edge = edge or r == 0 or c == 0 or r == h - 1 or c == w - 1
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < h and 0 <= cc < w and mask[rr, cc] and not seen[rr, cc]:
                        seen[rr, cc] = True
                        stack.append((rr, cc))
        if len(cells) <= max_cells and not edge:
            rows, cols = zip(*cells)
            out[list(rows), list(cols)] = True
    return out


def reachable(mask: np.ndarray, start: Tuple[int, int]) -> np.ndarray:
    """The cells of `mask` connected to `start` through side-by-side neighbours."""
    seen = np.zeros(mask.shape, dtype=bool)
    if not mask[start]:
        return seen
    h, w = mask.shape
    seen[start] = True
    todo = deque([start])
    while todo:
        r, c = todo.popleft()
        for nr, nc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
            if 0 <= nr < h and 0 <= nc < w and mask[nr, nc] and not seen[nr, nc]:
                seen[nr, nc] = True
                todo.append((nr, nc))
    return seen


def nearest(mask: np.ndarray, cell: Tuple[int, int]) -> Optional[Tuple[int, int]]:
    """The True cell nearest to `cell` (which may be outside the grid), or None if there is none."""
    rows, cols = np.nonzero(mask)
    if len(rows) == 0:
        return None
    d2 = (rows - cell[0]) ** 2 + (cols - cell[1]) ** 2
    i = int(np.argmin(d2))  # the first of equals: the lowest row, then column. Deterministic.
    return int(rows[i]), int(cols[i])


def runs(line: np.ndarray) -> List[Tuple[int, int]]:
    """The unbroken runs of True in a 1-D array, as (first, last) indices."""
    padded = np.concatenate(([False], line.astype(bool), [False]))
    edges = np.flatnonzero(padded[1:] != padded[:-1])
    return [(int(a), int(b) - 1) for a, b in zip(edges[0::2], edges[1::2])]


def polygon_mask(grid: Grid, polygon: Sequence[Point]) -> np.ndarray:
    """Cells whose centre is inside the polygon (even-odd rule; a centre on the edge counts as inside
    for the axis-aligned rectangles the app draws)."""
    h, w = grid.shape
    xs = grid.origin[0] + (np.arange(w) + 0.5) * grid.resolution
    ys = grid.origin[1] + (np.arange(h) + 0.5) * grid.resolution
    X, Y = np.meshgrid(xs, ys)
    pts = list(polygon)
    eps = 1e-9
    xmin, xmax = min(p[0] for p in pts), max(p[0] for p in pts)
    ymin, ymax = min(p[1] for p in pts), max(p[1] for p in pts)
    if len(pts) == 4 and _is_axis_rectangle(pts):
        return (X >= xmin - eps) & (X <= xmax + eps) & (Y >= ymin - eps) & (Y <= ymax + eps)
    inside = np.zeros((h, w), dtype=bool)
    for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]):
        crosses = (y0 > Y) != (y1 > Y)
        with np.errstate(divide="ignore", invalid="ignore"):
            x_at = x0 + (Y - y0) * (x1 - x0) / (y1 - y0)
        inside ^= crosses & (X < x_at)
    return inside


def _is_axis_rectangle(pts: Sequence[Point]) -> bool:
    xs = sorted({round(p[0], 9) for p in pts})
    ys = sorted({round(p[1], 9) for p in pts})
    return len(xs) == 2 and len(ys) == 2


def segment_cells(grid: Grid, p0: Point, p1: Point, radius: float) -> Tuple[np.ndarray, np.ndarray]:
    """(rows, columns) of the cells whose centre is within `radius` of the segment p0-p1, clipped
    to the grid."""
    res = grid.resolution
    ox, oy = grid.origin
    h, w = grid.shape
    c0 = max(int(math.floor((min(p0[0], p1[0]) - radius - ox) / res)) - 1, 0)
    c1 = min(int(math.ceil((max(p0[0], p1[0]) + radius - ox) / res)) + 1, w - 1)
    r0 = max(int(math.floor((min(p0[1], p1[1]) - radius - oy) / res)) - 1, 0)
    r1 = min(int(math.ceil((max(p0[1], p1[1]) + radius - oy) / res)) + 1, h - 1)
    if c0 > c1 or r0 > r1:
        return np.zeros(0, dtype=int), np.zeros(0, dtype=int)
    rr, cc = np.mgrid[r0:r1 + 1, c0:c1 + 1]
    xs = ox + (cc + 0.5) * res
    ys = oy + (rr + 0.5) * res
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    length2 = dx * dx + dy * dy
    if length2 == 0.0:
        t = np.zeros_like(xs)
    else:
        t = np.clip(((xs - p0[0]) * dx + (ys - p0[1]) * dy) / length2, 0.0, 1.0)
    near = (xs - (p0[0] + t * dx)) ** 2 + (ys - (p0[1] + t * dy)) ** 2 <= radius * radius + 1e-12
    return rr[near], cc[near]


def stamp_segment(mask: np.ndarray, grid: Grid, p0: Point, p1: Point, radius: float) -> None:
    """Set every cell whose centre is within `radius` of the segment p0-p1."""
    rows, cols = segment_cells(grid, p0, p1, radius)
    mask[rows, cols] = True


class Clearance:
    """How far each place is from the nearest obstacle (anything that is not floor)."""

    def __init__(self, grid: Grid, cap: float):
        self.grid = grid
        self.cap = cap
        self.field = clearance_field(grid.floor, grid.resolution, cap)  # per cell centre, exact up to cap
        self._k = int(math.ceil(cap / grid.resolution)) + 1
        self._blocked = np.pad(~grid.floor, self._k + 1, constant_values=True)

    def of_points(self, points: np.ndarray) -> np.ndarray:
        """Exact distance from each point (N x 2, metres) to the nearest obstacle square, up to cap."""
        res = self.grid.resolution
        ox, oy = self.grid.origin
        k = self._k
        u = (points[:, 0] - ox) / res  # cell units; cell j spans [j, j + 1)
        v = (points[:, 1] - oy) / res
        cu = np.floor(u).astype(int)
        cv = np.floor(v).astype(int)
        offs = np.arange(-k, k + 1)
        dj, di = np.meshgrid(offs, offs)
        dj, di = dj.ravel(), di.ravel()
        cols = cu[:, None] + dj[None, :]
        rows = cv[:, None] + di[None, :]
        pad = k + 1
        rr = np.clip(rows + pad, 0, self._blocked.shape[0] - 1)
        cc = np.clip(cols + pad, 0, self._blocked.shape[1] - 1)
        blocked = self._blocked[rr, cc]
        ddx = np.maximum(np.abs(u[:, None] - (cols + 0.5)) - 0.5, 0.0)
        ddy = np.maximum(np.abs(v[:, None] - (rows + 0.5)) - 0.5, 0.0)
        d = np.sqrt(ddx ** 2 + ddy ** 2) * res
        d = np.where(blocked, d, np.inf)
        return np.minimum(d.min(axis=1), self.cap)

    def segment_ok(self, p0: Point, p1: Point, need: float) -> bool:
        """Is every point of the segment at least `need` from any obstacle?"""
        pts = sample_segment(p0, p1, self.grid.resolution / 4.0)
        return bool((self.of_points(pts) >= need - 1e-9).all())


def sample_segment(p0: Point, p1: Point, step: float) -> np.ndarray:
    n = max(2, int(math.ceil(math.dist(p0, p1) / step)) + 1)
    t = np.linspace(0.0, 1.0, n)[:, None]
    return np.asarray(p0, dtype=float)[None, :] * (1.0 - t) + np.asarray(p1, dtype=float)[None, :] * t


def cells_on_line(grid: Grid, points: Iterable[Point]) -> np.ndarray:
    """Mask of the cells containing the given points (outside points are ignored)."""
    mask = np.zeros(grid.shape, dtype=bool)
    for x, y in points:
        r, c = grid.cell_of(x, y)
        if grid.contains(r, c):
            mask[r, c] = True
    return mask

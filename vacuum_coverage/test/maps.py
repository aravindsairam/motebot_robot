"""Small hand-made maps for the tests, and an independent check of what a plan promises.

The checks here are written separately from the planner on purpose: exact distances from points
to obstacle squares by brute force, so a mistake in the planner's faster method shows up.
"""

import math
from collections import deque

import numpy as np

from vacuum_coverage.grid import Grid

RES = 0.05


def room(width_m: float, height_m: float, wall: int = 2, margin: int = 2) -> np.ndarray:
    """Floor of one empty rectangular room: free inside, walls `wall` cells thick, unknown beyond.
    Returns a bool array, row 0 at the bottom."""
    w = int(round(width_m / RES))
    h = int(round(height_m / RES))
    floor = np.zeros((h + 2 * (wall + margin), w + 2 * (wall + margin)), dtype=bool)
    o = wall + margin
    floor[o:o + h, o:o + w] = True
    return floor


def grid(floor: np.ndarray, origin=(0.0, 0.0)) -> Grid:
    return Grid(floor=floor, resolution=RES, origin=origin)


def block(floor: np.ndarray, g: Grid, x0, y0, x1, y1) -> None:
    """Make the rectangle (map metres) an obstacle."""
    r0, c0 = g.cell_of(x0, y0)
    r1, c1 = g.cell_of(x1, y1)
    floor[r0:r1 + 1, c0:c1 + 1] = False


def obstacle_squares(g: Grid) -> np.ndarray:
    """Centres (x, y) of every cell that is not floor, plus a ring of cells just outside the map."""
    padded = np.pad(~g.floor, 1, constant_values=True)
    rows, cols = np.nonzero(padded)
    ox, oy = g.origin
    xs = ox + (cols - 1 + 0.5) * g.resolution
    ys = oy + (rows - 1 + 0.5) * g.resolution
    return np.stack([xs, ys], axis=1)


def exact_clearance(g: Grid, points: np.ndarray) -> np.ndarray:
    """Distance from each point to the nearest obstacle square, by brute force."""
    squares = obstacle_squares(g)
    h = g.resolution / 2.0
    out = np.empty(len(points))
    for i in range(0, len(points), 400):
        p = points[i:i + 400, None, :]
        d = np.maximum(np.abs(p - squares[None, :, :]) - h, 0.0)
        out[i:i + 400] = np.sqrt((d ** 2).sum(axis=2)).min(axis=1)
    return out


def sample(p0, p1, step=0.01) -> np.ndarray:
    n = max(2, int(math.ceil(math.dist(p0, p1) / step)) + 1)
    t = np.linspace(0.0, 1.0, n)[:, None]
    return np.asarray(p0)[None, :] * (1 - t) + np.asarray(p1)[None, :] * t


def cell_centres(g: Grid):
    rows, cols = np.indices(g.floor.shape)
    ox, oy = g.origin
    return ox + (cols + 0.5) * g.resolution, oy + (rows + 0.5) * g.resolution


def within_of_segments(g: Grid, segments, radius: float) -> np.ndarray:
    """Cells whose centre is within `radius` of any of the segments."""
    xs, ys = cell_centres(g)
    out = np.zeros(g.floor.shape, dtype=bool)
    for (x0, y0), (x1, y1) in segments:
        dx, dy = x1 - x0, y1 - y0
        length2 = dx * dx + dy * dy
        t = np.zeros_like(xs) if length2 == 0 else np.clip(((xs - x0) * dx + (ys - y0) * dy) / length2, 0.0, 1.0)
        out |= (xs - (x0 + t * dx)) ** 2 + (ys - (y0 + t * dy)) ** 2 <= radius ** 2 + 1e-12
    return out


def flood(mask: np.ndarray, start) -> np.ndarray:
    seen = np.zeros_like(mask)
    if not mask[start]:
        return seen
    seen[start] = True
    todo = deque([start])
    while todo:
        r, c = todo.popleft()
        for nr, nc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
            if 0 <= nr < mask.shape[0] and 0 <= nc < mask.shape[1] and mask[nr, nc] and not seen[nr, nc]:
                seen[nr, nc] = True
                todo.append((nr, nc))
    return seen

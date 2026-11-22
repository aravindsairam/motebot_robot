"""From a box in the camera picture to a place on the map, with the depth camera. No ROS imports.

Camera conventions (ROS optical frame): x to the right in the picture, y down, z forward. The
pinhole model: a pixel (u, v) at depth z is at x = (u - cx) z / fx, y = (v - cy) z / fy.
"""

import math
from typing import Optional, Sequence, Tuple

import numpy as np

MIN_DEPTH, MAX_DEPTH = 0.1, 8.0  # m: readings outside are not trusted (too close, or nothing there)


def near_point(box: Sequence[float], depth: np.ndarray, fx: float, fy: float, cx: float, cy: float,
               centre_share: float = 0.5, percentile: float = 20.0) -> Optional[Tuple[float, float, float]]:
    """The near side of the thing in `box` (x0, y0, x1, y1 pixels), as a point in the camera's optical
    frame: depth from the middle `centre_share` of the box (the box's edges often hold what is behind
    or under the thing), the nearer readings there (`percentile`), at the middle column of the box.
    None when the depth camera has no good reading there."""
    h, w = depth.shape
    x0, y0, x1, y1 = box
    mx, my = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    hw, hh = (x1 - x0) * centre_share / 2.0, (y1 - y0) * centre_share / 2.0
    c0, c1 = int(max(0, math.floor(mx - hw))), int(min(w, math.ceil(mx + hw)))
    r0, r1 = int(max(0, math.floor(my - hh))), int(min(h, math.ceil(my + hh)))
    if c1 <= c0 or r1 <= r0:
        return None
    patch = depth[r0:r1, c0:c1].astype(np.float64).ravel()
    good = patch[np.isfinite(patch) & (patch > MIN_DEPTH) & (patch < MAX_DEPTH)]
    if len(good) < max(5, 0.05 * len(patch)):
        return None
    z = float(np.percentile(good, percentile))
    return ((mx - cx) * z / fx, (my - cy) * z / fy, z)


def transform(matrix: np.ndarray, point: Sequence[float]) -> Tuple[float, float, float]:
    """A point through a 4x4 transform."""
    p = matrix @ np.array([point[0], point[1], point[2], 1.0])
    return float(p[0]), float(p[1]), float(p[2])


def matrix_from(translation: Sequence[float], quaternion_xyzw: Sequence[float]) -> np.ndarray:
    x, y, z, w = quaternion_xyzw
    rotation = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    out = np.eye(4)
    out[:3, :3] = rotation
    out[:3, 3] = translation
    return out


def distance_and_bearing(robot: Tuple[float, float, float], x: float, y: float) -> Tuple[float, float]:
    """From the robot (x, y, heading) to a point: metres, and radians from its heading (left positive)."""
    dx, dy = x - robot[0], y - robot[1]
    bearing = math.atan2(dy, dx) - robot[2]
    return math.hypot(dx, dy), math.atan2(math.sin(bearing), math.cos(bearing))

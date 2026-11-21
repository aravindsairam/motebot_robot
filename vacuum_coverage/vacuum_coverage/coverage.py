"""What has really been cleaned, from where the robot went. No ROS imports.

The robot model has no brush, so the cleaning tool is a disc `cleaning_width` across, centred on
the robot. Ten times a second the cleaning node hands in where the robot is on the map and where it
is by its wheel odometry. Every cell the disc passes over between two readings is marked cleaned,
but only while the robot sweeps: on rows, connectors and edges. Trips (to the first row, between
parts of the house, back onto a row after a pause, to the dock) add distance but clean nothing.
The distance comes from odometry, which is smooth; the map position jumps a little each time
localization corrects it, and those jumps are not distance driven.
"""

import math
from typing import List, Optional

import numpy as np

from .grid import Grid, Point, stamp_segment

JUMP = 0.5  # m: a map position this far from the last one is a localization correction, not a drive
PATH_SPACING = 0.05  # m between kept points of the path driven


class Recorder:
    def __init__(self, grid: Grid, planned: np.ndarray, tool_radius: float):
        self.grid = grid
        self.planned = planned
        self.tool_radius = tool_radius
        self.cleaned = np.zeros(grid.shape, dtype=bool)
        self.distance = 0.0
        self.path: List[Point] = []
        self._planned_cells = max(int(planned.sum()), 1)
        self._last_map: Optional[Point] = None
        self._last_odom: Optional[Point] = None
        self._last_sweep: Optional[Point] = None  # the last position stamped while sweeping

    def add(self, map_xy: Point, odom_xy: Point, sweeping: bool = True) -> None:
        """The robot is at `map_xy` on the map and at `odom_xy` by its wheel odometry. `sweeping` is
        False on trips: they add distance and path, but clean nothing, and no stretch is swept
        across them."""
        if not sweeping:
            self._last_sweep = None
        elif self._last_sweep is None or math.dist(self._last_sweep, map_xy) > JUMP:
            stamp_segment(self.cleaned, self.grid, map_xy, map_xy, self.tool_radius)
        else:
            stamp_segment(self.cleaned, self.grid, self._last_sweep, map_xy, self.tool_radius)
        if sweeping:
            self._last_sweep = map_xy
        if self._last_odom is not None:
            self.distance += math.dist(self._last_odom, odom_xy)
        if not self.path or math.dist(self.path[-1], map_xy) >= PATH_SPACING - 1e-9:
            self.path.append((float(map_xy[0]), float(map_xy[1])))
        self._last_map, self._last_odom = map_xy, odom_xy

    def break_stretch(self) -> None:
        """Cleaning paused: the next reading starts a new stretch (nothing is swept to it)."""
        self._last_map = self._last_odom = self._last_sweep = None

    @property
    def coverage(self) -> float:
        """Share of the planned floor cleaned, 0 to 1."""
        return float((self.cleaned & self.planned).sum()) / self._planned_cells

    @property
    def cleaned_area(self) -> float:
        """m2 of floor cleaned, planned or not."""
        return float((self.cleaned & self.grid.floor).sum()) * self.grid.resolution ** 2

    def occupancy(self) -> np.ndarray:
        """The cleaned cells as OccupancyGrid values: 100 cleaned, 0 not."""
        return np.where(self.cleaned, 100, 0).astype(np.int8)

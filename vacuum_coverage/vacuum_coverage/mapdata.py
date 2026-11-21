"""The saved map and the no-go zones, as the planner sees them. No ROS imports.

The map is read from its files, not from the /map topic, so a plan can be made in any mode, even
while localization is off. The no-go cells come from the same function the mode manager uses to
make Nav2's keepout mask, so the planner and Nav2 agree on every cell.
"""

import hashlib
import os
from dataclasses import dataclass
from typing import Iterable, Optional

import numpy as np

from vacuum_mode_manager import map_files

from .grid import Grid


@dataclass(frozen=True, eq=False)
class MapData:
    name: str  # the map's name: its file name without extension
    cells: np.ndarray  # int8, as map_server serves them: -1 unknown, 0 free, 100 occupied; row 0 at the bottom
    nogo: np.ndarray  # bool: inside a no-go zone
    grid: Grid  # floor = free and not no-go
    checksum: str  # changes when the map's cells, size, position or the no-go zones change

    @property
    def open_floor(self) -> np.ndarray:
        """Every cell the robot may stand on: free and not in a no-go zone."""
        return self.grid.floor


def load(map_yaml: str, zones: Optional[Iterable[dict]]) -> MapData:
    """Read a saved map (its .yaml) and lay the no-go zones (as map_files.read_zones gives them) over it."""
    cells, meta = map_files.read_map(map_yaml)
    zones = list(zones or [])
    nogo = map_files.keepout_cells(meta, zones) if zones else np.zeros(cells.shape, dtype=bool)
    floor = (cells == 0) & ~nogo
    grid = Grid(floor=floor, resolution=meta["resolution"], origin=meta["origin"])
    digest = hashlib.sha256()
    digest.update(f"{cells.shape} {meta['resolution']:.9f} {meta['origin'][0]:.6f} {meta['origin'][1]:.6f}".encode())
    digest.update(cells.tobytes())
    digest.update(np.packbits(nogo).tobytes())
    name = os.path.splitext(os.path.basename(map_yaml))[0]
    return MapData(name=name, cells=cells, nogo=nogo, grid=grid, checksum=digest.hexdigest()[:16])

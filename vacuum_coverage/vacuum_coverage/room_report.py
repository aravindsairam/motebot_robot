"""How clean each room is, from the floor each recorded clean really cleaned. No ROS imports."""

from typing import Callable, List, Optional, Sequence

import numpy as np

from .grid import Grid, polygon_mask
from .rooms import Room

MOST = 50.0  # percent of a room's floor: a clean that covered this much cleaned the room
TOUCHED = 5.0  # percent: a clean that covered this much passed through it
WEEK = 7 * 24 * 3600.0


def report(room_list: Sequence[Room], grid: Grid, reachable: Optional[np.ndarray], records: Sequence[dict],
           cleaned_of: Callable[[str], Optional[np.ndarray]], now: float) -> List[dict]:
    """For each room: {name, floor_area, last_cleaned, last_share, last_touched, cleans_last_week}.
    `records`: newest first, each with "id" and "ended". `reachable`: the floor the robot can clean
    (a room's share is of that part of its floor); None: all its floor. Records whose cleaned floor
    is missing, or was recorded on a map of another size, are not counted."""
    masks = {}
    for r in records:
        mask = cleaned_of(r["id"])
        if mask is not None and mask.shape == grid.shape:
            masks[r["id"]] = mask
    out = []
    for room in room_list:
        inside = polygon_mask(grid, room.outline) & grid.floor
        cells = inside & reachable if reachable is not None else inside
        if not cells.any():
            cells = inside
        n = float(cells.sum())
        entry = {"name": room.name, "floor_area": n * grid.resolution ** 2, "last_cleaned": 0.0, "last_share": 0.0,
                 "last_touched": 0.0, "cleans_last_week": 0}
        for r in records:
            mask = masks.get(r["id"])
            if mask is None or n == 0:
                continue
            share = 100.0 * float((mask & cells).sum()) / n
            ended = float(r.get("ended") or r.get("started") or 0.0)
            if share >= TOUCHED and not entry["last_touched"]:
                entry["last_touched"] = ended
            if share >= MOST:
                if not entry["last_cleaned"]:
                    entry["last_cleaned"], entry["last_share"] = ended, share
                if now - ended <= WEEK:
                    entry["cleans_last_week"] += 1
        out.append(entry)
    return out

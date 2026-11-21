"""The no-go zones of the map in use, read from that map's zones file (the one the mode manager
makes Nav2's keepout mask from), and the phone's command to change them. No ROS imports.

Each saved map has its own file, <maps dir>/<map>_zones.yaml (vacuum_mode_manager's map_files
decides the name for every package):
    zones:
      - name: stairwell
        polygon: [[4.55, 2.00], [5.65, 2.00], [5.65, 3.30], [4.55, 3.30]]
A map without the file has no zones.
"""

import os
from typing import Awaitable, Callable, List, Optional, Tuple

import yaml

from vacuum_mode_manager import map_files

from . import vacuum_pb2 as pb

Zone = Tuple[str, List[Tuple[float, float]]]
Result = pb.Reply.Result


def read_zones(path: str) -> List[Zone]:
    """The zones in the file, as (name, [(x, y), ...]). A missing file means no zones. Raises
    ValueError when the file cannot be read as zones, so a typo is reported, not ignored."""
    if not path or not os.path.isfile(path):
        return []
    with open(path, encoding="utf-8") as f:
        try:
            content = yaml.safe_load(f) or {}
        except yaml.YAMLError as exc:
            raise ValueError(f"not valid YAML: {exc}") from exc
    if not isinstance(content, dict):
        raise ValueError("expected a 'zones:' list at the top")
    out = []
    for i, zone in enumerate(content.get("zones") or []):
        name = str(zone.get("name", f"zone {i + 1}")) if isinstance(zone, dict) else ""
        polygon = zone.get("polygon") if isinstance(zone, dict) else None
        try:
            corners = [(float(x), float(y)) for x, y in polygon]
        except (TypeError, ValueError) as exc:
            raise ValueError(f"zone {i + 1}: 'polygon' must be a list of [x, y] corners") from exc
        if len(corners) < 3:
            raise ValueError(f"zone {i + 1} ({name}): a polygon needs at least 3 corners")
        out.append((name, corners))
    return out


class ZonesFile:
    """Reads the file again when it changes. `changed()` returns the new zones, or None when
    nothing has changed since the last call."""

    def __init__(self, path: str):
        self._path = path
        self._seen: Optional[tuple] = ()  # () = never looked

    def _signature(self) -> Optional[tuple]:
        try:
            st = os.stat(self._path)
        except (OSError, TypeError):
            return None  # missing: no zones
        # The inode too: the mode manager replaces the file (a rename), so even a rewrite of the
        # same size within the file system's clock tick is seen.
        return (st.st_ino, st.st_mtime_ns, st.st_size)

    def changed(self) -> Optional[List[Zone]]:
        signature = self._signature()
        if signature == self._seen:
            return None
        zones = read_zones(self._path) if signature is not None else []
        self._seen = signature  # only once read: a broken file is tried again on the next call
        return zones

    def forget(self) -> None:
        """Read the file again on the next `changed()`, whether or not it looks changed."""
        self._seen = ()


class MapZones:
    """Follows the map the robot uses, and reads the zones of whichever map that is.

    `follow()` is given the map in use from every robot state ("" while there is none, as while
    mapping); `changed()` returns (map name, zones) when they differ from what it last returned.
    Nothing is known, and nothing returned, until the first robot state."""

    def __init__(self, maps_dir: str):
        self._maps_dir = maps_dir
        self.map_name: Optional[str] = None  # None: not heard yet
        self._file: Optional[ZonesFile] = None
        self._news = False  # the robot left its map ("") and that has not been reported yet

    def follow(self, map_name: str) -> bool:
        """The robot uses `map_name` now. True when that is another map than before."""
        if map_name == self.map_name:
            return False
        self.map_name = map_name
        self._file = ZonesFile(map_files.zones_path(self._maps_dir, map_name)) if map_name else None
        self._news = True
        return True

    def changed(self, force: bool = False) -> Optional[Tuple[str, List[Zone]]]:
        """(map name, zones) if the map or its file changed since the last call (with `force`,
        the file is read again regardless), else None. Raises ValueError for a broken file."""
        if self.map_name is None:
            return None
        if self._file is None:
            news, self._news = self._news, False
            return ("", []) if news or force else None
        if force:
            self._file.forget()
        found = self._file.changed()  # a new map's file always counts as changed the first time
        return None if found is None else (self.map_name, found)


def zones_from_phone(zones) -> Tuple[Optional[List[Zone]], str]:
    """The zones of a SetNoGoZones command in metres, or None and what is wrong with them."""
    out = []
    for number, zone in enumerate(zones, start=1):
        cm = list(zone.points_cm)
        if len(cm) % 2:
            return None, f"zone {number} ({zone.name}) has an odd number of coordinates"
        out.append((zone.name, [(cm[i] / 100.0, cm[i + 1] / 100.0) for i in range(0, len(cm), 2)]))
    return out, ""


SaveZones = Callable[[str, List[Zone]], Awaitable[Optional[Tuple[bool, str]]]]


def set_zones_command(save: SaveZones, send_now: Callable[[], Awaitable[None]]):
    """What the gateway does with a SetNoGoZones command.

    save(map name, zones)  asks the robot (the mode manager) to keep these zones; gives (success,
                           message), or None when it cannot be asked (its service is absent)
    send_now()             reads the zones again and sends them to every phone, without waiting
                           for the next look at the file
    """

    async def run(message):
        zones, problem = zones_from_phone(message.zones)
        if problem:
            return Result.RESULT_BAD_REQUEST, problem
        answer = await save(message.map_name, zones)
        if answer is None:
            return Result.RESULT_NOT_AVAILABLE, "the mode manager is not running (no /set_no_go_zones service)"
        success, text = answer
        if not success:
            return Result.RESULT_REFUSED, text
        try:
            await send_now()
        except Exception:  # the zones are saved all the same; the next look at the file sends them
            pass
        return Result.RESULT_OK, text

    return run

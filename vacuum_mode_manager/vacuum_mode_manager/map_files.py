"""Reading and writing map files, and planar pose maths. No ROS imports.

A saved map is two files, as Nav2's map_server expects:
    <name>.pgm    grey image: 254 free, 0 occupied, 205 unknown
    <name>.yaml   resolution, origin, and the image file name
and, next to them, files that belong to it and are named after it:
    <name>_zones.yaml     its no-go zones (none if the file is missing)
    <name>_rooms.yaml     its rooms (none if the file is missing)
    <name>_things.yaml    things the robot has seen on it (a sofa, a bed), with where
    <name>_keepout.*      Nav2's keepout mask, made from the zones each time it is needed
Every package finds these files through the functions here, so all of them use the same names.
"""

import json
import math
import os
import re
import tempfile
from typing import Iterable, Sequence, Tuple

import numpy as np
import yaml
from PIL import Image, ImageDraw

FREE, OCCUPIED, UNKNOWN = 254, 0, 205
# Turning slam_toolbox's grid (0..100) into the image: at least 65 is occupied, at most 25 free.
OCCUPIED_THRESH, FREE_THRESH = 0.65, 0.25
# What the saved YAML tells map_server when it reads the image back. It computes a pixel's
# occupancy as (255 - pixel) / 255: free 254 -> 0.004, unknown 205 -> 0.196, occupied 0 -> 1.0, and
# calls a cell free when that is below free_thresh. With 0.25 (as written until 2026-11-14) the
# unknown cells came back FREE, so AMCL and Nav2 treated never-seen space as free. 0.196, the
# value the classic ROS map_saver writes, keeps 205 unknown (0.19608 is not below 0.196).
YAML_OCCUPIED_THRESH, YAML_FREE_THRESH = 0.65, 0.196

Pose2D = Tuple[float, float, float]  # x, y, yaw

_MAP_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]*$")
# A map's own files end in these, so no map may: its .yaml would then be another map's zones or
# mask, and saving it would overwrite them.
_RESERVED_ENDINGS = ("_keepout", "_zones", "_rooms", "_things")


def map_name_problem(name: str) -> str:
    """Why `name` cannot be a map's name, or "" if it can."""
    if not _MAP_NAME.match(name) or name.endswith(_RESERVED_ENDINGS):
        return "map names may use letters, digits, '-' and '_', and must not end in '_keepout', '_zones', '_rooms' or '_things'"
    return ""


def zones_path(maps_dir: str, map_name: str) -> str:
    """The file with a saved map's no-go zones. A map without one has no zones."""
    return os.path.join(maps_dir, map_name + "_zones.yaml")


def rooms_path(maps_dir: str, map_name: str) -> str:
    """The file with a saved map's rooms. A map without one has no rooms yet."""
    return os.path.join(maps_dir, map_name + "_rooms.yaml")


def things_path(maps_dir: str, map_name: str) -> str:
    """The file of things the robot has seen on a saved map, and where."""
    return os.path.join(maps_dir, map_name + "_things.yaml")


def zones_path_for(map_yaml: str) -> str:
    """The zones file of the map saved as `map_yaml` (<dir>/<name>.yaml)."""
    stem = os.path.splitext(os.path.abspath(map_yaml))[0]
    return zones_path(os.path.dirname(stem), os.path.basename(stem))


def keepout_stem(maps_dir: str, map_name: str) -> str:
    """Where Nav2's keepout mask for a map is written (without the .pgm / .yaml)."""
    return os.path.join(maps_dir, map_name + "_keepout")


# The mask Nav2 is given when it runs without a saved map (exploring). The name starts with a
# character no map's name may start with, so it is never a saved map's file or mask.
EMPTY_KEEPOUT_NAME = "_none_keepout"


def write_empty_keepout_mask(maps_dir: str) -> str:
    """Write a keepout mask that keeps the robot out of nothing: a few free cells at the map's
    origin. Nav2's keepout filter needs some mask (with no file name at all it warns every two
    seconds) and is content with one smaller than the costmap. Returns the yaml path."""
    image = np.full((4, 4), FREE, dtype=np.uint8)
    return write_map(os.path.join(maps_dir, EMPTY_KEEPOUT_NAME), image, 0.05, (0.0, 0.0))


def compose(a: Pose2D, b: Pose2D) -> Pose2D:
    """The pose `b`, given in frame A's child, expressed in A's parent: a * b."""
    ax, ay, ayaw = a
    bx, by, byaw = b
    c, s = math.cos(ayaw), math.sin(ayaw)
    yaw = ayaw + byaw
    return ax + c * bx - s * by, ay + s * bx + c * by, math.atan2(math.sin(yaw), math.cos(yaw))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def grid_to_image(data: Sequence[int], width: int, height: int) -> np.ndarray:
    """Occupancy values (-1 unknown, 0..100 probability) to a map image, top row first."""
    grid = np.asarray(data, dtype=np.int16).reshape(height, width)
    img = np.full((height, width), UNKNOWN, dtype=np.uint8)
    known = grid >= 0
    img[known & (grid >= OCCUPIED_THRESH * 100)] = OCCUPIED
    img[known & (grid <= FREE_THRESH * 100)] = FREE
    return np.flipud(img)  # grid row 0 is the bottom of the map; image row 0 is the top


def write_map(stem: str, image: np.ndarray, resolution: float, origin_xy: Tuple[float, float]) -> str:
    """Write <stem>.pgm and <stem>.yaml. Returns the yaml path."""
    os.makedirs(os.path.dirname(os.path.abspath(stem)), exist_ok=True)
    Image.fromarray(image, mode="L").save(stem + ".pgm")
    # Same layout as Nav2's map_saver writes, so every tool that reads one reads the other.
    lines = [
        f"image: {os.path.basename(stem)}.pgm",
        "mode: trinary",
        f"resolution: {float(resolution):.6g}",
        f"origin: [{float(origin_xy[0]):.6g}, {float(origin_xy[1]):.6g}, 0]",
        "negate: 0",
        f"occupied_thresh: {YAML_OCCUPIED_THRESH}",
        f"free_thresh: {YAML_FREE_THRESH}",
    ]
    with open(stem + ".yaml", "w") as f:
        f.write("\n".join(lines) + "\n")
    return stem + ".yaml"


def read_map_meta(map_yaml: str) -> dict:
    """Resolution, origin and image size of a saved map."""
    with open(map_yaml) as f:
        meta = yaml.safe_load(f)
    image_path = meta["image"]
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(os.path.abspath(map_yaml)), image_path)
    width, height = Image.open(image_path).size
    return {
        "resolution": float(meta["resolution"]),
        "origin": (float(meta["origin"][0]), float(meta["origin"][1])),
        "width": width,
        "height": height,
    }


def read_zones(zones_yaml: str) -> list:
    """No-go zones as [{'name': str, 'polygon': [[x, y], ...]}, ...]; empty if the file is missing."""
    if not zones_yaml or not os.path.isfile(zones_yaml):
        return []
    with open(zones_yaml, encoding="utf-8") as f:
        return (yaml.safe_load(f) or {}).get("zones") or []


_ZONES_HEADER = """\
# No-go zones of one saved map, in its map frame (metres): each zone is a list of [x, y] corners.
# Written by the mode manager whenever the zones are changed (from the app, or with
# 'ros2 run vacuum_mode_manager zones'), so comments added by hand are not kept.
"""


def _number(value: float) -> str:
    return repr(round(float(value), 4) + 0.0)  # to 0.1 mm; + 0.0 turns -0.0 into 0.0


def write_zones(path: str, zones: Iterable[dict]) -> None:
    """Write a zones file whole or not at all: into a temporary file next to it, then renamed over
    it. Anyone reading the file (the cleaning node, the gateway) sees the old list or the new one,
    never half of one. Zones as read_zones gives them."""
    lines = [_ZONES_HEADER.rstrip("\n")]
    zones = list(zones)
    if not zones:
        lines.append("zones: []")
    else:
        lines.append("zones:")
        for zone in zones:
            # A JSON string is also a valid YAML string, with any quote or accent kept as it is.
            lines.append(f"  - name: {json.dumps(str(zone['name']), ensure_ascii=False)}")
            corners = ", ".join(f"[{_number(x)}, {_number(y)}]" for x, y in zone["polygon"])
            lines.append(f"    polygon: [{corners}]")
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix="." + os.path.basename(path) + ".", suffix=".tmp", dir=directory)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
            f.flush()
            os.fsync(f.fileno())  # on disk before the rename, so a power cut leaves one file or the other
        os.chmod(temporary, 0o644)  # mkstemp makes it private to its owner
        os.replace(temporary, path)
    except BaseException:
        try:
            os.remove(temporary)
        except OSError:
            pass
        raise


def read_map(map_yaml: str) -> Tuple[np.ndarray, dict]:
    """A saved map as map_server serves it: cells -1 unknown, 0 free, 100 occupied, row 0 at the
    bottom (as in an OccupancyGrid); and its metadata (see read_map_meta)."""
    with open(map_yaml) as f:
        info = yaml.safe_load(f)
    image_path = info["image"]
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(os.path.abspath(map_yaml)), image_path)
    pixels = np.asarray(Image.open(image_path).convert("L"), dtype=np.float64)
    occupancy = (255.0 - pixels) / 255.0  # map_server's rule in trinary mode, not negated
    cells = np.full(pixels.shape, -1, dtype=np.int8)
    cells[occupancy > float(info.get("occupied_thresh", YAML_OCCUPIED_THRESH))] = 100
    cells[occupancy < float(info.get("free_thresh", YAML_FREE_THRESH))] = 0
    meta = {
        "resolution": float(info["resolution"]),
        "origin": (float(info["origin"][0]), float(info["origin"][1])),
        "width": pixels.shape[1],
        "height": pixels.shape[0],
    }
    return np.flipud(cells), meta


def _keepout_image(meta: dict, zones: Iterable[dict]) -> np.ndarray:
    """The keepout mask as an image (top row first): OCCUPIED inside every zone, FREE elsewhere."""
    res, (ox, oy), width, height = meta["resolution"], meta["origin"], meta["width"], meta["height"]
    mask = Image.new("L", (width, height), FREE)
    draw = ImageDraw.Draw(mask)
    for zone in zones:
        # map (x, y) in metres -> image (column, row); image rows run top to bottom
        pixels = [((x - ox) / res, height - (y - oy) / res) for x, y in zone["polygon"]]
        draw.polygon(pixels, fill=OCCUPIED)
    return np.asarray(mask)


def keepout_cells(meta: dict, zones: Iterable[dict]) -> np.ndarray:
    """Which map cells are inside a no-go zone (True), row 0 at the bottom. Exactly the cells the
    keepout mask given to Nav2 blocks: the cleaning planner uses this so both agree."""
    return np.flipud(_keepout_image(meta, zones) == OCCUPIED)


def write_keepout_mask(map_yaml: str, zones: Iterable[dict], out_stem: str) -> str:
    """Write a Nav2 keepout mask lined up with the map: black inside every zone. Returns the yaml path."""
    meta = read_map_meta(map_yaml)
    return write_map(out_stem, _keepout_image(meta, zones), meta["resolution"], meta["origin"])

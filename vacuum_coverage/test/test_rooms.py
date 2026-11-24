"""Rooms found from a map: split at doorways, borders on the doorways, names kept, the file."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

from vacuum_coverage import rooms
from vacuum_coverage.grid import Grid, polygon_mask
from vacuum_coverage.rooms import Room

RES = 0.05


def plan(width, height):
    """An empty map: unknown everywhere. Draw with wall() and floor()."""
    return np.full((int(round(height / RES)), int(round(width / RES))), -1, dtype=np.int8)


def fill(cells, x0, y0, x1, y1, value):
    cells[int(round(y0 / RES)):int(round(y1 / RES)), int(round(x0 / RES)):int(round(x1 / RES))] = value


def grid_of(cells):
    return Grid(floor=cells == 0, resolution=RES, origin=(0.0, 0.0)), cells == 100


def walled_room(cells, x0, y0, x1, y1, t=0.1):
    fill(cells, x0 - t, y0 - t, x1 + t, y1 + t, 100)
    fill(cells, x0, y0, x1, y1, 0)


def iou(a, b):
    return (a & b).sum() / (a | b).sum()


def truth(g, x0, y0, x1, y1):
    return polygon_mask(g, [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]) & g.floor


def test_two_rooms_through_a_door_are_two_rooms_split_on_the_door():
    cells = plan(9.0, 4.0)
    walled_room(cells, 0.2, 0.2, 4.2, 3.2)
    walled_room(cells, 4.3, 0.2, 8.3, 3.2)
    fill(cells, 4.2, 1.2, 4.3, 2.1, 0)  # a 0.9 m door in the wall between
    g, walls = grid_of(cells)
    found = rooms.find(g, walls=walls)
    assert [r.name for r in found] == ["Room 1", "Room 2"]
    left, right = truth(g, 0.2, 0.2, 4.2, 3.2), truth(g, 4.3, 0.2, 8.3, 3.2)
    assert max(iou(r.cells, left) for r in found) > 0.95 and max(iou(r.cells, right) for r in found) > 0.95
    for r in found:
        assert g.floor[g.cell_of(r.x, r.y)] and abs(r.area - 12.0) < 0.8
        inside = polygon_mask(g, r.outline)
        assert (inside & r.cells).sum() >= 0.98 * r.cells.sum()


def test_a_wide_arch_still_splits_and_the_border_lies_on_it_though_furniture_fences_a_corner():
    """The home's kitchen took the dining corner through its arch until borders were moved onto doorways."""
    cells = plan(11.0, 5.0)
    walled_room(cells, 0.2, 0.2, 6.2, 4.2)   # living room
    walled_room(cells, 6.3, 0.2, 9.8, 3.4)   # kitchen
    fill(cells, 6.2, 1.2, 6.3, 2.6, 0)       # a 1.4 m arch
    for x in (4.6, 5.2, 5.8):                # chair legs fencing the corner by the arch
        for y in (1.0, 1.6, 2.2, 2.8):
            fill(cells, x, y, x + 0.05, y + 0.05, 100)
    g, walls = grid_of(cells)
    found = rooms.find(g, walls=walls)
    assert len(found) == 2
    living, kitchen = truth(g, 0.2, 0.2, 6.2, 4.2), truth(g, 6.3, 0.2, 9.8, 3.4)
    assert max(iou(r.cells, living) for r in found) > 0.95
    assert max(iou(r.cells, kitchen) for r in found) > 0.93


def test_a_small_closet_is_not_a_room_of_its_own():
    cells = plan(6.0, 4.0)
    walled_room(cells, 0.2, 0.2, 4.2, 3.2)
    walled_room(cells, 4.3, 1.0, 5.3, 2.2)   # 1.2 m2 closet
    fill(cells, 4.2, 1.2, 4.3, 1.9, 0)
    g, walls = grid_of(cells)
    assert len(rooms.find(g, walls=walls)) == 1


@pytest.mark.parametrize("furniture", [
    [(3.6, 1.1), (4.4, 1.1), (3.6, 1.9), (4.4, 1.9)],  # the four legs of a table in the middle
    [(3.2, 1.05, 5.2, 1.95)],                          # a sofa: one solid block
])
def test_furniture_standing_free_does_not_split_a_room(furniture):
    cells = plan(9.0, 4.0)
    walled_room(cells, 0.2, 0.2, 8.2, 3.2)
    for thing in furniture:
        x0, y0, x1, y1 = thing if len(thing) == 4 else (thing[0], thing[1], thing[0] + RES, thing[1] + RES)
        fill(cells, x0, y0, x1, y1, 100)
    g, walls = grid_of(cells)
    found = rooms.find(g, walls=walls)
    assert len(found) == 1
    assert found[0].cells.sum() == g.floor.sum()  # all the floor, and none of the furniture
    assert g.floor[g.cell_of(found[0].x, found[0].y)]


@pytest.mark.parametrize("thick", [0.1, 0.4])
def test_a_wall_standing_free_with_a_way_round_each_end_still_splits(thick):
    cells = plan(9.0, 4.4)
    walled_room(cells, 0.2, 0.2, 8.2, 3.8)
    fill(cells, 4.2, 1.0, 4.2 + thick, 3.0, 100)  # 2 m of wall across the middle, 0.8 m open at each end
    g, walls = grid_of(cells)
    assert len(rooms.find(g, walls=walls)) == 2


HALLWAY_HOME = {"left": (0.2, 0.2, 2.9, 4.2), "hallway": (3.0, 0.2, 3.9, 4.2), "right": (4.0, 0.2, 7.0, 4.2)}


def hallway_home(tilt):
    """Two rooms off a hallway no wider than their doors (0.9 m), on a map turned by `tilt` degrees:
    (cells, {name: its true floor})."""
    size = 11.0
    n = int(round(size / RES))
    centres = (np.arange(n) + 0.5) * RES - size / 2
    x, y = np.meshgrid(centres, centres)
    a = math.radians(tilt)
    u = x * math.cos(a) + y * math.sin(a) + 3.6  # the plan's own axes, its middle at the map's
    v = -x * math.sin(a) + y * math.cos(a) + 2.2

    def box(x0, y0, x1, y1):
        return (u >= x0) & (u < x1) & (v >= y0) & (v < y1)

    cells = np.full((n, n), -1, dtype=np.int8)
    cells[box(0.1, 0.1, 7.1, 4.3)] = 100
    for room in HALLWAY_HOME.values():
        cells[box(*room)] = 0
    cells[box(2.9, 1.7, 3.0, 2.6)] = 0  # the door of the left room
    cells[box(3.9, 0.8, 4.0, 1.7)] = 0  # the door of the right room
    return cells, {name: box(*room) & (cells == 0) for name, room in HALLWAY_HOME.items()}


@pytest.mark.parametrize("tilt", [0, 3, 12, 31, 45])
def test_a_hallway_as_narrow_as_its_doors_is_a_room_whichever_way_the_map_is_turned(tilt):
    cells, true_rooms = hallway_home(tilt)
    g, walls = grid_of(cells)
    found = rooms.find(g, walls=walls)
    assert len(found) == 3
    for name, true_cells in true_rooms.items():
        assert max(iou(r.cells, true_cells) for r in found) > 0.9, name


def test_a_long_hallway_is_not_cut_where_the_walls_between_rooms_line_up_across_it():
    cells = plan(13.0, 7.2)
    walled_room(cells, 0.2, 3.0, 12.2, 4.0)  # the hallway, 1 m wide
    for x in (0.2, 4.3, 8.4):                # three rooms on each side, a 0.8 m door each
        walled_room(cells, x, 0.2, x + 3.8, 2.9)
        fill(cells, x + 1.5, 2.9, x + 2.3, 3.0, 0)
        walled_room(cells, x, 4.1, x + 3.8, 6.8)
        fill(cells, x + 1.5, 4.0, x + 2.3, 4.1, 0)
    g, walls = grid_of(cells)
    found = rooms.find(g, walls=walls)
    assert len(found) == 7
    assert max(iou(r.cells, truth(g, 0.2, 3.0, 12.2, 4.0)) for r in found) > 0.9


def test_found_rooms_keep_the_names_of_the_saved_rooms_they_match():
    cells = plan(9.0, 4.0)
    walled_room(cells, 0.2, 0.2, 4.2, 3.2)
    walled_room(cells, 4.3, 0.2, 8.3, 3.2)
    fill(cells, 4.2, 1.2, 4.3, 2.1, 0)
    g, walls = grid_of(cells)
    saved = [Room("Kitchen", [(4.3, 0.2), (8.3, 0.2), (8.3, 3.2), (4.3, 3.2)], 6.0, 1.5, 12.0),
             Room("Room 1", [(20.0, 20.0), (21.0, 20.0), (21.0, 21.0)], 20.5, 20.5, 1.0),  # matches nothing: dropped
             Room("Room 3", [(0.2, 0.2), (4.2, 0.2), (4.2, 3.2), (0.2, 3.2)], 2.0, 1.5, 12.0)]  # still there: keeps its name
    named = rooms.keep_names(rooms.find(g, walls=walls), saved, g)
    assert sorted(r.name for r in named) == ["Kitchen", "Room 3"]
    named = rooms.keep_names(rooms.find(g, walls=walls), saved[:2], g)
    assert sorted(r.name for r in named) == ["Kitchen", "Room 1"]


@pytest.mark.parametrize("changed, words", [
    (dict(name=" "), "needs a name"),
    (dict(name="x" * 41), "longer than 40"),
    (dict(outline=[(0.0, 0.0), (1.0, 0.0)]), "no outline"),
    (dict(x=float("nan")), "not a number"),
])
def test_rooms_that_cannot_be_saved(changed, words):
    good = dict(name="Hall", outline=[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)], x=1.0, y=0.5, area=2.0)
    assert words in rooms.problem([Room(**{**good, **changed})])
    assert "two rooms are called" in rooms.problem([Room(**good), Room(**{**good, "name": "hall"})])
    assert rooms.problem([Room(**good)]) == ""


def test_the_file_round_trip_and_finding_by_name(tmp_path):
    path = str(tmp_path / "home_rooms.yaml")
    assert rooms.read(path) == []
    saved = [Room("Living room", [(0.0, 0.0), (6.0, 0.0), (6.0, 4.0), (0.0, 4.0)], 3.0, 2.0, 22.5),
             Room("Chambre é", [(6.0, 0.0), (9.0, 0.0), (9.0, 3.0)], 7.0, 1.0, 4.5)]
    rooms.write(path, saved)
    back = rooms.read(path)
    assert [r.to_dict() for r in back] == [r.to_dict() for r in saved]
    assert rooms.by_name(back, "  living   ROOM ").name == "Living room"
    assert rooms.by_name(back, "garage") is None


# ---------------------------------------------------------------- the saved maps, against the true rooms
def _workspace_file(*parts):
    for folder in Path(__file__).resolve().parents:
        if (folder.joinpath(*parts)).is_file():
            return folder.joinpath(*parts)
    return None


@pytest.mark.parametrize("maps, name, count", [("maps", "house", 2), ("maps_home", "home", 6)])
def test_the_saved_maps_have_their_true_rooms(maps, name, count):
    path = _workspace_file(maps, f"{name}.yaml")
    if path is None:
        pytest.skip("the saved maps are not here")
    from vacuum_coverage import mapdata
    from vacuum_mode_manager import map_files
    m = mapdata.load(str(path), map_files.read_zones(str(path.parent / f"{name}_zones.yaml")))
    found = rooms.find(m.grid, walls=m.cells == 100)
    assert len(found) == count
    world_file = _workspace_file("sim", "scripts", "worlds", f"{name}.py")
    if world_file is None:
        return
    sys.path.insert(0, str(world_file.parent.parent))
    try:
        world = __import__(f"worlds.{name}", fromlist=["ROOMS"])
    finally:
        sys.path.pop(0)
    for room_name in world.ROOMS:
        true_cells = truth(m.grid, *world.room_world(room_name))
        best = max(iou(r.cells, true_cells) for r in found)
        assert best >= 0.8, f"{room_name}: best match {best:.2f}"


def _turned(mask, tilt):
    """The picture `mask` (0 / 1) turned by `tilt` degrees about its middle, on a sheet big enough to hold it."""
    import cv2
    h, w = mask.shape
    side = int(math.ceil(math.hypot(h, w))) + 4
    turn = cv2.getRotationMatrix2D(((w - 1) / 2, (h - 1) / 2), tilt, 1.0)
    turn[:, 2] += ((side - w) / 2, (side - h) / 2)
    return cv2.warpAffine(mask.astype(np.uint8), turn, (side, side), flags=cv2.INTER_NEAREST).astype(bool)


@pytest.mark.parametrize("tilt", [2, 10, 33])
def test_the_saved_home_map_keeps_its_true_rooms_when_the_map_is_turned(tilt):
    """A robot that starts a new map askew to the walls gets a map like this one."""
    path = _workspace_file("maps_home", "home.yaml")
    world_file = _workspace_file("sim", "scripts", "worlds", "home.py")
    if path is None or world_file is None:
        pytest.skip("the saved home map or its true rooms are not here")
    from vacuum_coverage import mapdata
    from vacuum_mode_manager import map_files
    m = mapdata.load(str(path), map_files.read_zones(str(path.parent / "home_zones.yaml")))
    sys.path.insert(0, str(world_file.parent.parent))
    try:
        world = __import__("worlds.home", fromlist=["ROOMS"])
    finally:
        sys.path.pop(0)
    walls = _turned(m.cells == 100, tilt)
    g = Grid(floor=_turned(m.grid.floor, tilt) & ~walls, resolution=m.grid.resolution, origin=(0.0, 0.0))
    found = rooms.find(g, walls=walls)
    assert len(found) == 6
    for room_name in world.ROOMS:
        true_cells = _turned(truth(m.grid, *world.room_world(room_name)), tilt) & g.floor
        best = max(iou(r.cells, true_cells) for r in found)
        assert best >= 0.75, f"{room_name}: best match {best:.2f}"

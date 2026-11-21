"""What the mode manager accepts as a map's new no-go zones (/set_no_go_zones), and when."""

import math
import os
from pathlib import Path

import pytest

from vacuum_mode_manager import map_files
from vacuum_mode_manager import state_machine as sm
from vacuum_mode_manager import zone_rules as zr


def _maps_folder():
    """The saved maps. They are kept in the workspace (ros2_ws/maps), not in this package, so the
    folder is looked for upwards from here; "" when this package is used without them."""
    for folder in Path(__file__).resolve().parents:
        if (folder / "maps" / "house_zones.yaml").is_file():
            return str(folder / "maps")
    return ""


MAPS = _maps_folder()

EXTENT = (-1.0, -2.0, 9.0, 6.0)  # a 10 m x 8 m map
DOCK = (-0.119, 0.0)  # as in docking.yaml


def zone(name, *corners):
    return {"name": name, "polygon": [list(c) for c in corners]}


def box(name, x0, y0, x1, y1):
    return {"name": name, "polygon": zr.rectangle(x0, y0, x1, y1)}


STAIRWELL = box("stairwell", 4.55, 2.0, 5.65, 3.3)


def check(*zones, robot=None, dock=DOCK, saved=()):
    return zr.check_zones(list(zones), EXTENT, robot=robot, dock=dock, saved=saved)


# ---------------------------------------------------------------- when
@pytest.mark.parametrize("mode", [sm.IDLE, sm.NAVIGATION, sm.MANUAL, sm.DOCKING, sm.DOCKED, sm.FAILSAFE])
def test_zones_can_change_in_these_modes(mode):
    assert zr.mode_problem(mode, False, "house", "house") == ""


@pytest.mark.parametrize("mode,changing,in_use,asked,expected", [
    (sm.MAPPING, False, "house", "house", "while a map is being made"),
    (sm.EXPLORING, False, "house", "house", "while a map is being made"),
    (sm.CLEANING, False, "house", "house", "stop the clean first"),
    (sm.IDLE, True, "house", "house", "changing modes"),
    (sm.IDLE, False, "", "house", "no saved map yet"),
    (sm.IDLE, False, "", "", "no saved map yet"),
    (sm.NAVIGATION, False, "house", "upstairs", "uses the map 'house', not 'upstairs'"),
    (sm.NAVIGATION, False, "house", "", "not 'none'"),
])
def test_zones_cannot_change_now(mode, changing, in_use, asked, expected):
    assert expected in zr.mode_problem(mode, changing, in_use, asked)


def test_map_extent_from_the_map_files():
    meta = {"resolution": 0.05, "origin": (-0.6, -2.3), "width": 200, "height": 100}
    x0, y0, x1, y1 = zr.map_extent(meta)
    assert (x0, y0) == (-0.6, -2.3) and math.isclose(x1, 9.4) and math.isclose(y1, 2.7)


# ---------------------------------------------------------------- good lists
def test_good_zones_pass():
    assert check() == ""  # no zones at all: every zone deleted
    assert check(STAIRWELL, box("rug", 1.0, 1.0, 2.0, 1.5), zone("corner", (6, 4), (7, 4), (6.5, 5))) == ""
    assert check(box("small", 3.0, 3.0, 3.1, 3.1)) == ""  # exactly 10 cm across
    assert check(box("edge", -1.0, -2.0, 9.0, -1.0)) == ""  # touching the map's edges
    # an L shape: not convex, but a real zone
    assert check(zone("L", (2, 2), (4, 2), (4, 2.5), (2.5, 2.5), (2.5, 4), (2, 4))) == ""


def test_corners_from_float32_still_pass():
    import numpy as np  # geometry_msgs/Polygon holds float32 corners: 10 cm may come out as 9.99999 cm

    corners = [(float(np.float32(x)), float(np.float32(y))) for x, y in [(8.85, 5.85), (8.95, 5.85), (8.95, 5.95), (8.85, 5.95)]]
    assert check(zone("tiny", *corners)) == ""


def test_up_to_50_zones():
    zones = [box(f"z{i}", 1 + (i % 10) * 0.5, 1 + (i // 10) * 0.5, 1.2 + (i % 10) * 0.5, 1.2 + (i // 10) * 0.5) for i in range(50)]
    assert check(*zones) == ""
    assert "at most 50" in check(*zones, box("one more", 7, 5, 7.5, 5.5))


# ---------------------------------------------------------------- names
@pytest.mark.parametrize("name,expected", [
    ("", "zone 1 has no name"),
    ("   ", "zone 1 has no name"),
    ("x" * 41, "too long"),
    ("two\nlines", "line break"),
])
def test_bad_names(name, expected):
    assert expected in check(box(name, 1, 1, 2, 2))


def test_names_up_to_40_characters_and_any_letters():
    assert check(box("x" * 40, 1, 1, 2, 2), box("rug é 🐈", 3, 3, 4, 4)) == ""


def test_a_name_may_not_be_used_twice():
    assert check(box("rug", 1, 1, 2, 2), box("rug", 3, 3, 4, 4)) == "two zones are called 'rug'"


# ---------------------------------------------------------------- shapes
@pytest.mark.parametrize("corners,expected", [
    ([(1, 1), (2, 2)], "has 2 corners: a zone has 3 to 20"),
    ([(1 + math.cos(a / 3), 1 + math.sin(a / 3)) for a in range(21)], "has 21 corners"),
    ([(1, 1), (2, float("nan")), (2, 2)], "not a number"),
    ([(1, 1), (2, float("inf")), (2, 2)], "not a number"),
    ([(8.5, 5.5), (9.5, 5.5), (9.5, 5.9)], "reaches outside the map"),
    ([(1, -2.5), (2, -1), (1, -1)], "reaches outside the map"),
    ([(1, 1), (5, 1), (5, 1.05), (1, 1.05)], "less than 10 cm across"),
    ([(1, 1), (1.05, 1), (1.05, 5), (1, 5)], "less than 10 cm across"),
    ([(1, 1), (4, 4.03), (4, 4.1), (1, 1.07)], "less than 10 cm across"),  # a thin strip on a slant
    ([(1, 1), (2, 2), (3, 3)], "less than 10 cm across"),  # in a line
    ([(1, 1), (2, 2), (2, 1), (1, 2)], "crosses itself"),  # a bow tie
])
def test_shapes_that_are_not_zones(corners, expected):
    assert expected in check(zone("z", *corners))


def test_a_zone_that_folds_back_on_itself_has_no_area():
    # wide enough, does not cross itself, but encloses nothing: out along a line and back
    assert "almost no area" in check(zone("z", (1, 1), (3, 1), (3, 1.5), (3, 1), (1, 1.0001)))


def test_a_narrow_slanted_rectangle_is_measured_across_not_along():
    # 2 m long, 15 cm wide, turned by 0.5 rad
    corners = [(1.0, 1.0), (1 + 2 * math.cos(0.5), 1 + 2 * math.sin(0.5))]
    corners += [(corners[1][0] - 0.15 * math.sin(0.5), corners[1][1] + 0.15 * math.cos(0.5)),
                (1.0 - 0.15 * math.sin(0.5), 1.0 + 0.15 * math.cos(0.5))]
    assert math.isclose(zr.narrowest_width(corners), 0.15)
    assert check(zone("slant", *corners)) == ""


# ---------------------------------------------------------------- the robot and the dock
def test_a_new_zone_over_or_next_to_the_robot_is_refused():
    assert "covers the robot" in check(box("rug", 1, 1, 2, 2), robot=(1.5, 1.5))
    assert "within 20 cm of it" in check(box("rug", 1, 1, 2, 2), robot=(2.15, 1.5))
    assert check(box("rug", 1, 1, 2, 2), robot=(2.25, 1.5)) == ""
    assert check(box("rug", 1, 1, 2, 2), robot=None) == ""  # where the robot is is not known: not checked


def test_a_zone_near_the_dock_is_refused():
    assert "within 50 cm of the dock" in check(box("hall", 0.3, -0.5, 1.0, 0.5))
    assert "within 50 cm of the dock" in check(box("hall", -1.0, -2.0, 1.0, -0.45))
    assert check(box("hall", 0.4, -0.5, 1.0, 0.5)) == ""
    assert check(box("hall", 0.3, -0.5, 1.0, 0.5), dock=None) == ""


def test_zones_already_saved_and_not_moved_are_not_held_to_the_robot_and_dock_rules():
    # The robot cleans 15 cm from the stairwell; deleting another zone must still be possible.
    saved = [STAIRWELL, box("rug", 1, 1, 2, 2)]
    beside_stairwell = (4.4, 2.5)
    assert check(STAIRWELL, robot=beside_stairwell, saved=saved) == ""
    # A float32 copy of the same corners counts as the same zone.
    copy = {"name": "stairwell", "polygon": [[x + 1e-7, y - 1e-7] for x, y in STAIRWELL["polygon"]]}
    assert check(copy, robot=beside_stairwell, saved=saved) == ""
    # Moved, even a little, it is a new zone and checked like one.
    moved = box("stairwell", 4.5, 2.0, 5.65, 3.3)
    assert "within 20 cm" in check(moved, robot=beside_stairwell, saved=saved)
    # Nor to the map's edges: the house's stairwell runs past the saved map's edge, and a map
    # replaced by a smaller one keeps its zones. A new zone must be inside the map.
    past_the_edge = box("landing", 7.0, 5.0, 8.0, 6.5)
    assert check(past_the_edge, saved=[past_the_edge]) == ""
    assert "outside the map" in check(past_the_edge)
    # Its shape is always checked.
    thin = zone("thin", (1, 1), (5, 1), (5, 1.05), (1, 1.05))
    assert "less than 10 cm across" in check(thin, saved=[thin])


@pytest.mark.skipif(not MAPS, reason="no house map")
def test_the_house_maps_own_zones_can_be_sent_back_unchanged_and_added_to():
    # What the app and the zones command do: read the list, change it, send all of it back.
    saved = map_files.read_zones(map_files.zones_path(MAPS, "house"))
    assert [z["name"] for z in saved] == ["stairwell"]
    extent = zr.map_extent(map_files.read_map_meta(os.path.join(MAPS, "house.yaml")))
    at_the_dock = (0.0, 0.0)
    assert zr.check_zones(saved, extent, robot=at_the_dock, dock=DOCK, saved=saved) == ""
    rug = box("rug", 2.0, 0.5, 2.6, 1.0)
    assert zr.check_zones(saved + [rug], extent, robot=at_the_dock, dock=DOCK, saved=saved) == ""


# ---------------------------------------------------------------- geometry helpers
def test_distance_to_a_polygon():
    square = [(0, 0), (1, 0), (1, 1), (0, 1)]
    assert zr.distance_to_polygon((0.5, 0.5), square) == 0.0
    assert math.isclose(zr.distance_to_polygon((1.5, 0.5), square), 0.5)
    assert math.isclose(zr.distance_to_polygon((2, 2), square), math.sqrt(2))


def test_rectangle_from_any_two_opposite_corners():
    assert zr.rectangle(2, 3, 1, 1) == [[1, 1], [2, 1], [2, 3], [1, 3]]
    assert zr.area(zr.rectangle(2, 3, 1, 1)) == 2.0  # anticlockwise


def test_zones_cannot_change_while_a_map_is_being_made_whatever_the_mode_is_called():
    # failsafe entered from mapping: the saved map named is not the one being made
    assert "while a map is being made" in zr.mode_problem(sm.FAILSAFE, False, "house", "house", making_map=True)
    assert zr.mode_problem(sm.FAILSAFE, False, "house", "house", making_map=False) == ""

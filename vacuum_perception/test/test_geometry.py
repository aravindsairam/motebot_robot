"""From a box in the picture to a place on the map."""

import math

import numpy as np
import pytest

from vacuum_perception import finder, geometry

FX = FY = 300.0
CX, CY = 320.0, 240.0


def test_the_near_side_in_the_middle_of_the_box_is_found_in_the_camera_frame():
    depth = np.full((480, 640), 5.0, dtype=np.float32)  # a far wall
    depth[200:300, 400:500] = 2.0  # the thing, 2 m away
    depth[200:300, 400:410] = 6.0  # the box's edge shows what is behind it
    x, y, z = geometry.near_point((400, 200, 500, 300), depth, FX, FY, CX, CY)
    assert z == pytest.approx(2.0) and x == pytest.approx((450 - CX) * 2.0 / FX) and y == pytest.approx((250 - CY) * 2.0 / FY)


def test_no_good_depth_is_no_point():
    depth = np.full((480, 640), np.nan, dtype=np.float32)
    assert geometry.near_point((100, 100, 200, 200), depth, FX, FY, CX, CY) is None
    depth[:] = 20.0  # nothing there within range
    assert geometry.near_point((100, 100, 200, 200), depth, FX, FY, CX, CY) is None
    assert geometry.near_point((700, 100, 800, 200), depth, FX, FY, CX, CY) is None  # outside the picture


def test_camera_to_map_and_distance_and_bearing():
    # A camera at (1, 2, 0.07) on the map looking along the map's +x: optical z forward -> map x,
    # optical x right -> map -y, optical y down -> map -z.
    q = (-0.5, 0.5, -0.5, 0.5)
    m = geometry.matrix_from((1.0, 2.0, 0.07), q)
    x, y, z = geometry.transform(m, (0.5, 0.0, 2.0))  # 2 m ahead, 0.5 m to the right in the picture
    assert (x, y, z) == pytest.approx((3.0, 1.5, 0.07))
    d, b = geometry.distance_and_bearing((1.0, 2.0, 0.0), x, y)
    assert d == pytest.approx(math.hypot(2.0, 0.5)) and b == pytest.approx(-math.atan2(0.5, 2.0))


def test_the_detector_prompt_and_names():
    assert finder.prompt(["Sofa", " red  chair ", "", "t.v."]) == "sofa. red chair. t v."
    assert finder.best_name("chair", ["red chair", "sofa"]) == "red chair"
    assert finder.best_name("sofa", ["sofa"]) == "sofa"
    assert finder.best_name("lamp", ["sofa"]) == "lamp"


def test_a_box_over_most_of_the_picture_is_not_a_thing():
    assert finder.plausible((100, 100, 300, 300), 640, 480)
    assert not finder.plausible((0, 240, 640, 480), 640, 480)   # the floor, edge to edge
    assert not finder.plausible((10, 10, 600, 470), 640, 480)   # most of the picture

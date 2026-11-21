"""Recording what was really cleaned, from where the robot went."""


import numpy as np
import pytest

from maps import RES, grid, room, within_of_segments
from vacuum_coverage.coverage import Recorder

TOOL = 0.15


def test_a_straight_drive_cleans_a_band_the_tool_wide():
    g = grid(room(3.0, 2.0))
    planned = g.floor.copy()
    rec = Recorder(g, planned, TOOL)
    for i in range(21):  # 1 m along y = 1.0, every 5 cm, as ten readings a second at 0.5 m/s would
        x = 1.0 + i * 0.05
        rec.add((x, 1.0), (x, 0.0))
    expected = within_of_segments(g, [((1.0, 1.0), (2.0, 1.0))], TOOL) & g.floor
    assert (rec.cleaned == expected).all()
    assert rec.distance == pytest.approx(1.0)
    assert rec.cleaned_area == pytest.approx(expected.sum() * RES * RES)
    assert rec.coverage == pytest.approx(expected.sum() / planned.sum())


def test_cells_between_two_readings_are_cleaned_too():
    g = grid(room(3.0, 2.0))
    rec = Recorder(g, g.floor.copy(), TOOL)
    rec.add((1.0, 1.0), (0.0, 0.0))
    rec.add((1.4, 1.0), (0.4, 0.0))  # a long gap between readings (less than a jump)
    expected = within_of_segments(g, [((1.0, 1.0), (1.4, 1.0))], TOOL) & g.floor
    assert (rec.cleaned == expected).all()


def test_distance_comes_from_odometry_not_from_jumps_of_the_map_position():
    g = grid(room(3.0, 2.0))
    rec = Recorder(g, g.floor.copy(), TOOL)
    rec.add((1.0, 1.0), (0.0, 0.0))
    rec.add((1.04, 1.0), (0.03, 0.0))  # localization nudges the map position 1 cm more than the wheels moved
    assert rec.distance == pytest.approx(0.03)


def test_a_jump_of_the_map_position_is_not_swept():
    g = grid(room(3.0, 2.0))
    rec = Recorder(g, g.floor.copy(), TOOL)
    rec.add((0.6, 1.0), (0.0, 0.0))
    rec.add((2.6, 1.0), (0.02, 0.0))  # 2 m in one reading: localization was corrected, nothing was driven
    swept = within_of_segments(g, [((0.6, 1.0), (2.6, 1.0))], TOOL)
    ends = within_of_segments(g, [((0.6, 1.0), (0.6, 1.0)), ((2.6, 1.0), (2.6, 1.0))], TOOL)
    assert (rec.cleaned == (ends & g.floor)).all()
    assert swept.sum() > ends.sum()


def test_a_break_starts_a_new_stretch():
    g = grid(room(3.0, 2.0))
    rec = Recorder(g, g.floor.copy(), TOOL)
    rec.add((0.6, 1.0), (0.0, 0.0))
    rec.break_stretch()
    rec.add((1.6, 1.0), (1.0, 0.0))
    assert rec.distance == 0.0
    assert not rec.cleaned[g.cell_of(1.1, 1.0)]


def test_the_path_is_kept_every_5_cm():
    g = grid(room(3.0, 2.0))
    rec = Recorder(g, g.floor.copy(), TOOL)
    for i in range(101):
        x = 0.5 + i * 0.01
        rec.add((x, 1.0), (x, 0.0))
    xs = [p[0] for p in rec.path]
    assert len(rec.path) == 21
    assert all(b - a == pytest.approx(0.05) for a, b in zip(xs, xs[1:]))


def test_coverage_counts_only_planned_floor():
    g = grid(room(3.0, 2.0))
    planned = np.zeros(g.shape, dtype=bool)
    r, c = g.cell_of(1.0, 1.0)
    planned[r - 2:r + 3, c - 2:c + 3] = True  # 25 planned cells around (1, 1)
    rec = Recorder(g, planned, TOOL)
    rec.add((1.0, 1.0), (0.0, 0.0))
    rec.add((2.0, 1.0), (1.0, 0.0))
    assert rec.coverage == pytest.approx((rec.cleaned & planned).sum() / 25)
    assert rec.cleaned_area > 25 * RES * RES


def test_the_cleaned_cells_as_an_occupancy_grid():
    g = grid(room(1.0, 1.0))
    rec = Recorder(g, g.floor.copy(), TOOL)
    rec.add((0.6, 0.6), (0.0, 0.0))
    cells = rec.occupancy()
    assert cells.dtype == np.int8 and cells.shape == g.shape
    assert set(np.unique(cells)) == {0, 100}
    assert (cells == 100).sum() == rec.cleaned.sum()


def test_readings_that_are_not_sweeping_clean_nothing_but_count_distance_and_path():
    g = grid(room(3.0, 2.0))
    rec = Recorder(g, g.floor.copy(), TOOL)
    for i in range(11):
        x = 0.5 + i * 0.05
        rec.add((x, 1.0), (x, 0.0), sweeping=False)
    assert not rec.cleaned.any()
    assert rec.distance == pytest.approx(0.5)
    assert len(rec.path) == 11


def test_no_stretch_is_swept_across_a_trip():
    g = grid(room(3.0, 2.0))
    rec = Recorder(g, g.floor.copy(), TOOL)
    rec.add((0.5, 1.0), (0.5, 0.0), sweeping=True)  # the end of a row
    rec.add((1.0, 1.0), (1.0, 0.0), sweeping=False)  # a trip
    rec.add((1.5, 1.0), (1.5, 0.0), sweeping=True)  # the next row starts
    expected = within_of_segments(g, [((0.5, 1.0), (0.5, 1.0)), ((1.5, 1.0), (1.5, 1.0))], TOOL) & g.floor
    assert (rec.cleaned == expected).all()
    assert rec.distance == pytest.approx(1.0)


def test_sweeping_is_the_default():
    g = grid(room(3.0, 2.0))
    rec = Recorder(g, g.floor.copy(), TOOL)
    rec.add((1.0, 1.0), (0.0, 0.0))
    rec.add((1.2, 1.0), (0.2, 0.0))
    assert (rec.cleaned == (within_of_segments(g, [((1.0, 1.0), (1.2, 1.0))], TOOL) & g.floor)).all()

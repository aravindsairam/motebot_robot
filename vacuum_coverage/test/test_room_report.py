"""How clean each room is, from the floor each clean really cleaned."""

import numpy as np

from vacuum_coverage.grid import Grid
from vacuum_coverage.room_report import WEEK, report
from vacuum_coverage.rooms import Room

RES = 0.05
NOW = 1_800_000_000.0


def setup():
    floor = np.ones((40, 80), dtype=bool)  # 4 m x 2 m: two rooms of 2 m x 2 m
    grid = Grid(floor=floor, resolution=RES, origin=(0.0, 0.0))
    left = Room("Hall", [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)], 1.0, 1.0, 4.0)
    right = Room("Kitchen", [(2.0, 0.0), (4.0, 0.0), (4.0, 2.0), (2.0, 2.0)], 3.0, 1.0, 4.0)
    return grid, [left, right]


def cleaned(columns):
    mask = np.zeros((40, 80), dtype=bool)
    mask[:, columns] = True
    return mask


def test_the_last_clean_that_covered_most_of_a_room_and_the_last_that_passed_through():
    grid, rooms = setup()
    records = [  # newest first
        {"id": "c", "ended": NOW - 3600},               # 10 % of the kitchen
        {"id": "b", "ended": NOW - 86400},              # all the hall, 60 % of the kitchen
        {"id": "a", "ended": NOW - 10 * 86400},         # all of both, but more than a week ago
        {"id": "old", "ended": NOW - 2 * 86400},        # no cleaned floor kept: not counted
    ]
    masks = {"c": cleaned(slice(40, 44)), "b": cleaned(slice(0, 64)), "a": cleaned(slice(0, 80))}
    hall, kitchen = report(rooms, grid, None, records, masks.get, NOW)
    assert hall["last_cleaned"] == NOW - 86400 and hall["last_share"] == 100.0 and hall["cleans_last_week"] == 1
    assert hall["last_touched"] == NOW - 86400 and abs(hall["floor_area"] - 4.0) < 1e-9
    assert kitchen["last_touched"] == NOW - 3600
    assert kitchen["last_cleaned"] == NOW - 86400 and abs(kitchen["last_share"] - 60.0) < 1e-9
    assert kitchen["cleans_last_week"] == 1


def test_a_room_never_cleaned_and_a_map_of_another_size():
    grid, rooms = setup()
    records = [{"id": "x", "ended": NOW}]
    out = report(rooms, grid, None, records, lambda _rid: np.ones((5, 5), dtype=bool), NOW)
    assert all(e["last_cleaned"] == 0.0 and e["last_touched"] == 0.0 for e in out)


def test_shares_count_only_the_floor_the_robot_can_clean():
    grid, rooms = setup()
    reachable = cleaned(slice(0, 20))  # only half the hall can be cleaned
    out = report(rooms[:1], grid, reachable, [{"id": "a", "ended": NOW - WEEK - 1}], {"a": cleaned(slice(0, 20))}.get, NOW)
    assert out[0]["last_share"] == 100.0 and out[0]["cleans_last_week"] == 0

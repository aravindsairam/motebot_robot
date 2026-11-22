"""The planner against the seven promises of the plan (docs/2026-11-18-coverage-cleaning-plan.md,
section 7.3), on small hand-made maps and on the saved house map."""

import math
import os
import time
from pathlib import Path

import numpy as np
import pytest

from maps import RES, block, exact_clearance, flood, grid, room, sample, within_of_segments
from vacuum_coverage import planner
from vacuum_coverage.planner import CONNECTOR, EDGE, ROW, TRANSIT, Options, Piece, PlanError, Settings, make_plan

S = Settings()


def _saved_map(name):
    """A file of the saved house map. The maps are kept in the workspace (ros2_ws/maps), not in this
    package, so it is looked for upwards from here; "" when this package is used without them."""
    for folder in Path(__file__).resolve().parents:
        if (folder / "maps" / name).is_file():
            return str(folder / "maps" / name)
    return ""


HOUSE_YAML = _saved_map("house.yaml")
ZONES_YAML = _saved_map("house_zones.yaml")


# ---------------------------------------------------------------- helpers
def pieces(plan, kind):
    return [p for p in plan.pieces if p.kind == kind]


def segments(plan, *kinds):
    return [(p.start, p.end) for p in plan.pieces if p.kind in kinds]


def check_clearance(g, plan, kinds, clearance, tolerance=1e-6):
    pts = [sample(p.start, p.end) for p in plan.pieces if p.kind in kinds]
    if not pts:
        return
    pts = np.concatenate(pts)
    worst = exact_clearance(g, pts).min()
    assert worst >= clearance - tolerance, f"a {kinds} point is {worst:.3f} m from an obstacle"


def check_chain(plan, robot):
    """Each piece starts where the one before ended; the first starts at the robot."""
    here = robot
    for p in plan.pieces:
        assert math.dist(here, p.start) < 1e-9, f"{p} does not start at {here}"
        here = p.end


def check_planned_floor_is_covered(g, plan, settings=S, at_least=0.98):
    """Promise 2: the reachable floor is fixed before any row is laid; the rows (with the connectors
    between them, which clean too) reach at least 98 % of it. The planned floor is what the tool
    passes over on rows, connectors and edges; the reachable floor it misses is counted as left out."""
    tool = within_of_segments(g, segments(plan, ROW, CONNECTOR, EDGE), settings.cleaning_width / 2.0) & g.floor
    assert (plan.planned <= tool).all() and ((tool & plan.reachable) <= plan.planned).all()  # an area cuts the rest off
    assert not (plan.left_out & plan.planned).any()
    reachable = plan.reachable
    assert not (plan.left_out & ~reachable).any()
    share = (plan.planned & reachable).sum() / reachable.sum()
    assert share >= at_least, f"the rows reach {share * 100:.1f}% of the reachable floor"


def check_rows_alternate(plan):
    """Promise 3: neighbouring rows of one region are driven in opposite directions."""
    by_region = {}
    for p in pieces(plan, ROW):
        by_region.setdefault((p.pass_number, p.region), []).append(p)
    for rows in by_region.values():
        for a, b in zip(rows, rows[1:]):
            da = np.subtract(a.end, a.start)
            db = np.subtract(b.end, b.start)
            assert np.dot(da, db) < 0, f"{a} and {b} run the same way"


def check_connectors(g, plan, settings=S):
    """Promise 4: a connector is a short straight hop inside row-clear space."""
    for c in pieces(plan, CONNECTOR):
        assert math.dist(c.start, c.end) <= settings.max_connector_length + 1e-9
    check_clearance(g, plan, (CONNECTOR,), settings.row_clearance, tolerance=0.005)


def check_all(g, plan, robot, settings=S):
    check_chain(plan, robot)
    check_clearance(g, plan, (ROW,), settings.row_clearance)
    check_planned_floor_is_covered(g, plan, settings)
    check_rows_alternate(plan)
    check_connectors(g, plan, settings)


# ---------------------------------------------------------------- an empty room
def test_empty_room_is_covered_with_alternating_rows():
    floor = room(3.0, 2.0)
    g = grid(floor)
    robot = (1.5, 1.2)
    plan = make_plan(g, robot, S, Options(direction="x"))
    check_all(g, plan, robot)
    rows = pieces(plan, ROW)
    assert all(abs(r.start[1] - r.end[1]) < 1e-9 for r in rows), "rows along x are horizontal"
    assert plan.estimate.rows == len(rows) and len(rows) >= 5
    # one region: rows joined by connectors, a single transit from the robot to the first row
    assert len(pieces(plan, TRANSIT)) == 1
    assert plan.estimate.planned_area == pytest.approx(plan.planned.sum() * RES * RES)


def test_rows_keep_row_clearance_from_the_walls():
    g = grid(room(2.0, 1.5))
    plan = make_plan(g, (1.0, 0.8), S, Options(direction="y"))
    check_clearance(g, plan, (ROW,), S.row_clearance)
    assert all(abs(r.start[0] - r.end[0]) < 1e-9 for r in pieces(plan, ROW)), "rows along y are vertical"


def test_auto_direction_picks_fewer_turns():
    # a long thin room: rows along its length need far fewer turns
    g = grid(room(4.0, 1.0))
    plan = make_plan(g, (0.5, 0.5), S, Options(direction="auto"))
    rows = pieces(plan, ROW)
    assert all(abs(r.start[1] - r.end[1]) < 1e-9 for r in rows)
    assert plan.directions == ["x"]


# ---------------------------------------------------------------- around furniture
def test_rows_go_around_a_box_in_the_middle():
    floor = room(3.0, 2.5)
    g = grid(floor)
    block(floor, g, 1.4, 1.1, 2.0, 1.6)  # a table in the middle
    robot = (0.6, 0.6)
    plan = make_plan(g, robot, S, Options(direction="x"))
    check_all(g, plan, robot)
    # some rows end at the box: more than one region
    assert len({p.region for p in pieces(plan, ROW)}) >= 2


# ---------------------------------------------------------------- two rooms
def two_rooms(door_open=True):
    floor = room(5.0, 2.0)
    g = grid(floor)
    block(floor, g, 2.4, 0.0, 2.55, 2.4)  # a dividing wall
    if door_open:
        r0, c0 = g.cell_of(2.4, 0.6)
        r1, c1 = g.cell_of(2.55, 1.4)
        floor[r0:r1 + 1, c0:c1 + 1] = True  # an 0.8 m doorway
    return g


def test_two_rooms_through_a_doorway_are_both_planned():
    g = two_rooms(door_open=True)
    robot = (0.8, 1.0)
    plan = make_plan(g, robot, S, Options(direction="x"))
    check_all(g, plan, robot)
    xs = [x for p in pieces(plan, ROW) for x in (p.start[0], p.end[0])]
    assert min(xs) < 1.0 and max(xs) > 4.0
    assert plan.unreachable.sum() == 0


def test_a_room_the_robot_cannot_reach_is_left_out_and_reported():
    g = two_rooms(door_open=False)
    robot = (0.8, 1.0)
    plan = make_plan(g, robot, S, Options(direction="x"))
    check_all(g, plan, robot)
    assert max(x for p in pieces(plan, ROW) for x in (p.start[0], p.end[0])) < 2.4
    assert plan.estimate.unreachable_area > 1.0
    assert not (plan.unreachable & plan.planned).any()


def test_the_robot_starting_on_the_dock_starts_from_the_nearest_reachable_cell():
    g = grid(room(3.0, 2.0))
    robot = (0.3, 1.2)  # 10 cm from the wall, closer than row_clearance: like the docked robot
    plan = make_plan(g, robot, S, Options())
    check_all(g, plan, robot)
    assert plan.pieces[0].kind == TRANSIT


def test_no_floor_to_clean_is_an_error():
    g = grid(room(0.4, 0.4))  # smaller than the robot needs
    with pytest.raises(PlanError):
        make_plan(g, (0.2, 0.2), S, Options())


# ---------------------------------------------------------------- rooms: one at a time, each its own way
def wide_and_tall_rooms():
    """A wide, shallow room (3.0 x 1.6 m) and a narrow, tall one (1.6 x 3.0 m) beside it, joined by
    an 0.8 m doorway: rows along x suit the first, rows along y the second."""
    floor = room(4.75, 3.0)
    g = grid(floor)
    block(floor, g, 0.0, 1.6, 3.0, 3.2)  # the wide room is only 1.6 m deep
    block(floor, g, 3.0, 0.0, 3.15, 3.2)  # the wall between the rooms
    r0, c0 = g.cell_of(3.0, 0.4)
    r1, c1 = g.cell_of(3.15, 1.2)
    floor[r0:r1 + 1, c0:c1 + 1] = True
    return g


def three_rooms_in_a_row():
    floor = room(7.5, 2.0)
    g = grid(floor)
    for x in (2.4, 4.9):
        block(floor, g, x, 0.0, x + 0.15, 2.4)
        r0, c0 = g.cell_of(x, 0.6)
        r1, c1 = g.cell_of(x + 0.15, 1.4)
        floor[r0:r1 + 1, c0:c1 + 1] = True
    return g


def part_of(g, plan, piece):
    """The part (room) a piece lies in: the one under its middle."""
    return int(plan.parts[g.cell_of((piece.start[0] + piece.end[0]) / 2.0, (piece.start[1] + piece.end[1]) / 2.0)])


def check_parts_one_at_a_time(g, plan, kinds=(ROW,)):
    """Every part is finished before the next one starts: its pieces come in one unbroken run."""
    seen = [part_of(g, plan, p) for p in plan.pieces if p.kind in kinds]
    runs_of = [q for i, q in enumerate(seen) if i == 0 or seen[i - 1] != q]
    assert len(runs_of) == len(set(runs_of)), f"the parts are visited as {runs_of}"
    assert runs_of == [q for q in plan.part_order if q in runs_of]
    return runs_of


def horizontal(p):
    return abs(p.start[1] - p.end[1]) < 1e-9


def test_two_rooms_are_cleaned_one_after_the_other_each_its_own_way():
    g = wide_and_tall_rooms()
    robot = (0.8, 0.8)
    plan = make_plan(g, robot, S, Options())
    check_all(g, plan, robot)
    assert plan.parts.max() == 2  # split at the doorway
    first, second = check_parts_one_at_a_time(g, plan)
    assert first == part_of(g, plan, Piece(ROW, robot, robot))  # the robot's own room first
    rows = pieces(plan, ROW)
    assert all(horizontal(r) for r in rows if part_of(g, plan, r) == first), "the wide room has rows along x"
    assert not any(horizontal(r) for r in rows if part_of(g, plan, r) == second), "the tall room has rows along y"
    assert plan.directions == ["mixed"]
    # one direction for the whole plan needs more turns and transits
    for direction in ("x", "y"):
        one_way = make_plan(g, robot, S, Options(direction=direction))
        check_parts_one_at_a_time(g, one_way)
        assert plan.estimate.turns + plan.estimate.transits < one_way.estimate.turns + one_way.estimate.transits


def test_a_forced_direction_holds_in_every_room():
    g = wide_and_tall_rooms()
    for direction in ("x", "y"):
        plan = make_plan(g, (0.8, 0.8), S, Options(direction=direction))
        check_chain(plan, (0.8, 0.8))
        check_clearance(g, plan, (ROW,), S.row_clearance)
        check_rows_alternate(plan)
        check_connectors(g, plan)
        check_parts_one_at_a_time(g, plan)
        assert all(horizontal(r) == (direction == "x") for r in pieces(plan, ROW))
        assert plan.directions == [direction]


def test_both_passes_and_the_edges_finish_a_room_before_the_next():
    g = wide_and_tall_rooms()
    robot = (0.8, 0.8)
    plan = make_plan(g, robot, S, Options(passes=2, edges=True))
    check_chain(plan, robot)
    check_rows_alternate(plan)
    check_clearance(g, plan, (ROW,), S.row_clearance)
    check_clearance(g, plan, (EDGE, CONNECTOR), S.edge_clearance, tolerance=0.005)  # hops between edges keep edge_clearance
    check_planned_floor_is_covered(g, plan)
    check_parts_one_at_a_time(g, plan, kinds=(ROW, EDGE))
    for part in plan.part_order:
        mine = [p for p in plan.pieces if p.kind in (ROW, EDGE) and part_of(g, plan, p) == part]
        kinds = [(p.kind, p.pass_number) for p in mine]
        assert kinds == sorted(kinds, key=lambda k: (k[0] == EDGE, k[1])), "pass 1, then pass 2, then the edges"
        one = {horizontal(p) for p in mine if p.kind == ROW and p.pass_number == 1}
        two = {horizontal(p) for p in mine if p.kind == ROW and p.pass_number == 2}
        assert len(one) == 1 and len(two) == 1 and one != two, "the second pass at right angles to that room's first"
    assert plan.directions == ["mixed", "mixed"]


def test_rooms_are_split_at_doorways_and_not_inside_a_room():
    assert make_plan(two_rooms(), (0.8, 1.0), S, Options()).parts.max() == 2
    assert make_plan(three_rooms_in_a_row(), (0.8, 1.0), S, Options()).parts.max() == 3
    floor = room(4.0, 3.0)
    g = grid(floor)
    block(floor, g, 1.6, 1.4, 4.2, 3.2)  # an L-shaped room: no narrow passage in it
    assert make_plan(g, (0.6, 0.6), S, Options()).parts.max() == 1
    floor = room(3.0, 2.5)
    g = grid(floor)
    block(floor, g, 1.4, 1.1, 2.0, 1.6)  # a table in the middle of a room
    assert make_plan(g, (0.6, 0.6), S, Options()).parts.max() == 1


@pytest.mark.parametrize("robot, expected", [((0.8, 1.0), "ABC"), ((6.5, 1.0), "CBA")])
def test_rooms_are_visited_in_the_order_with_the_shortest_trips(robot, expected):
    g = three_rooms_in_a_row()
    plan = make_plan(g, robot, S, Options())
    check_all(g, plan, robot)
    order = check_parts_one_at_a_time(g, plan)
    names = {}
    for r in pieces(plan, ROW):
        x = (r.start[0] + r.end[0]) / 2.0
        names.setdefault(part_of(g, plan, r), "A" if x < 2.4 else "B" if x < 4.9 else "C")
    assert "".join(names[q] for q in order) == expected
    # each room is left by the doorway to the next: no trip longer than the way through it
    assert max(p.length for p in pieces(plan, TRANSIT)) < 2.0


def test_the_regions_of_a_room_are_visited_without_going_back_and_forth():
    # ways of driving five short items on a line, given in a muddled order: the improved order
    # drives them left to right (or right to left), each the way that leads on to the next
    xs = [3.0, 0.0, 4.0, 1.0, 2.0]
    points = [(0.0, 0.0)] + [(x, 0.0) for x in xs] + [(x + 0.5, 0.0) for x in xs]
    cost = [[abs(a[0] - b[0]) for b in points] + [0.0] for a in points] + [[0.0] * (len(points) + 1)]
    start, end = 0, len(points)  # from the robot at x = 0; the last point costs nothing to reach: no aim
    ways = [[(1 + i, 6 + i), (6 + i, 1 + i)] for i in range(5)]
    backwards = [[1, 0] for _ in range(5)]
    first = [(i, 1) for i in range(5)]
    seq = planner._improve(list(first), ways, backwards, cost, start, end)
    seq = planner._best_ways(seq, ways, cost, start, end)
    assert [xs[i] for i, _ in seq] == [0.0, 1.0, 2.0, 3.0, 4.0]
    assert all(w == 0 for _, w in seq)
    assert planner._length(seq, ways, cost, start, end) == pytest.approx(2.0)  # four 0.5 m hops
    assert planner._length(first, ways, cost, start, end) > 10.0


def test_rows_the_edge_pass_makes_needless_are_left_out():
    g = grid(room(3.0, 2.0))
    rows_only = make_plan(g, (1.0, 1.0), S, Options(direction="x"))
    with_edges = make_plan(g, (1.0, 1.0), S, Options(direction="x", edges=True))
    assert with_edges.estimate.rows < rows_only.estimate.rows  # the rows along the walls
    assert (with_edges.planned >= rows_only.planned).all()


# ---------------------------------------------------------------- no-go zones
def test_a_no_go_zone_is_kept_out_of():
    floor = room(3.0, 2.0)
    g = grid(floor)
    block(floor, g, 2.0, 0.0, 3.0, 1.0)  # no-go cells are not floor for the planner
    robot = (0.6, 1.4)
    plan = make_plan(g, robot, S, Options())
    check_all(g, plan, robot)
    pts = np.concatenate([sample(p.start, p.end) for p in plan.pieces if p.kind in (ROW, CONNECTOR)])
    # the zone's cells span x 2.0 to 3.05, y 0.0 to 1.05
    dx = np.maximum(np.maximum(2.0 - pts[:, 0], pts[:, 0] - 3.05), 0.0)
    dy = np.maximum(np.maximum(0.0 - pts[:, 1], pts[:, 1] - 1.05), 0.0)
    assert np.hypot(dx, dy).min() >= S.row_clearance - 1e-6


# ---------------------------------------------------------------- an area
def test_an_area_keeps_the_robot_centre_inside_it():
    g = grid(room(4.0, 3.0))
    area = [(1.0, 0.5), (2.5, 0.5), (2.5, 2.0), (1.0, 2.0)]
    robot = (0.6, 0.6)
    plan = make_plan(g, robot, S, Options(), area=area)
    check_all(g, plan, robot)
    first_row = next(i for i, p in enumerate(plan.pieces) if p.kind == ROW)  # the robot starts outside
    pts = np.concatenate([sample(p.start, p.end) for p in plan.pieces[first_row:] if p.kind in (ROW, CONNECTOR)])
    # ... and keeps area_margin from its edge, so a few centimetres of overshoot or localization error
    # at a row's end do not take the real robot outside (phase 23 measured up to 5.4 cm without it)
    m = S.area_margin
    assert (pts[:, 0] >= 1.0 + m - 1e-9).all() and (pts[:, 0] <= 2.5 - m + 1e-9).all()
    assert (pts[:, 1] >= 0.5 + m - 1e-9).all() and (pts[:, 1] <= 2.0 - m + 1e-9).all()
    # only floor inside the area counts as reachable or planned (the tool may sweep beyond it)
    rows, cols = np.nonzero(plan.planned | plan.reachable)
    xs, ys = (cols + 0.5) * RES, (rows + 0.5) * RES
    assert (xs >= 1.0 - 1e-9).all() and (xs <= 2.5 + 1e-9).all() and (ys >= 0.5 - 1e-9).all() and (ys <= 2.0 + 1e-9).all()


def test_an_area_with_no_reachable_floor_is_refused():
    floor = room(4.0, 3.0)
    g = grid(floor)
    block(floor, g, 2.0, 1.0, 3.0, 2.0)
    with pytest.raises(PlanError, match="area"):
        make_plan(g, (0.6, 0.6), S, Options(), area=[(2.2, 1.2), (2.8, 1.2), (2.8, 1.8), (2.2, 1.8)])


def test_a_spot_is_a_square_around_the_point():
    area = planner.spot_area(2.0, 1.0, 1.5)
    xs = [p[0] for p in area]
    ys = [p[1] for p in area]
    assert (min(xs), max(xs), min(ys), max(ys)) == (1.25, 2.75, 0.25, 1.75)


# ---------------------------------------------------------------- two passes, edges
def test_the_second_pass_runs_at_right_angles():
    g = grid(room(2.5, 2.0))
    robot = (1.0, 1.0)
    one = make_plan(g, robot, S, Options(direction="x", passes=1))
    two = make_plan(g, robot, S, Options(direction="x", passes=2))
    check_chain(two, robot)
    first = [p for p in pieces(two, ROW) if p.pass_number == 1]
    second = [p for p in pieces(two, ROW) if p.pass_number == 2]
    assert first and second
    assert all(abs(p.start[1] - p.end[1]) < 1e-9 for p in first)
    assert all(abs(p.start[0] - p.end[0]) < 1e-9 for p in second)
    assert two.directions == ["x", "y"]
    assert 1.6 < two.estimate.distance / one.estimate.distance < 2.6
    check_rows_alternate(two)


def test_edges_follow_walls_and_furniture_at_edge_clearance():
    floor = room(3.0, 2.5)
    g = grid(floor)
    block(floor, g, 1.4, 1.1, 2.0, 1.6)
    robot = (0.6, 0.6)
    rows_only = make_plan(g, robot, S, Options(direction="x"))
    with_edges = make_plan(g, robot, S, Options(direction="x", edges=True))
    check_chain(with_edges, robot)
    edges = pieces(with_edges, EDGE)
    assert len(edges) >= 8  # four walls and four sides of the table
    check_clearance(g, with_edges, (EDGE,), S.edge_clearance)
    for e in edges:  # straight, along x or y, one goal each
        assert abs(e.start[0] - e.end[0]) < 1e-9 or abs(e.start[1] - e.end[1]) < 1e-9
        assert math.dist(e.start, e.end) >= S.min_edge_length - 1e-9
    # the edges come after the rows, and clean floor the rows cannot reach
    kinds = [p.kind for p in with_edges.pieces if p.kind in (ROW, EDGE)]
    assert kinds.index(EDGE) > max(i for i, k in enumerate(kinds) if k == ROW)
    assert with_edges.planned.sum() > rows_only.planned.sum()
    check_planned_floor_is_covered(g, with_edges)
    assert with_edges.estimate.edges == len(edges)


# ---------------------------------------------------------------- planned floor, independently
def test_planned_and_left_out_floor_is_what_the_tool_can_reach_from_row_clear_space():
    floor = room(3.0, 2.5)
    g = grid(floor)
    block(floor, g, 1.4, 1.1, 2.0, 1.6)
    robot = (0.6, 0.6)
    plan = make_plan(g, robot, S, Options())
    xs, ys = np.meshgrid(np.arange(floor.shape[1]), np.arange(floor.shape[0]))
    centres = np.stack([(xs.ravel() + 0.5) * RES, (ys.ravel() + 0.5) * RES], axis=1)
    clear = exact_clearance(g, centres).reshape(floor.shape)
    reach = flood(clear >= S.transit_clearance, g.cell_of(*robot))
    row_clear = (clear >= S.row_clearance) & reach
    rows, cols = np.nonzero(row_clear)
    tool = within_of_segments(g, [((x, y), (x, y)) for x, y in zip((cols + 0.5) * RES, (rows + 0.5) * RES)],
                              S.cleaning_width / 2.0)
    assert ((plan.planned | plan.left_out) >= (tool & floor)).all()
    assert not (plan.left_out & ~(tool & floor)).any()


# ---------------------------------------------------------------- estimate, determinism, the house
def test_the_estimate_adds_up():
    g = grid(room(3.0, 2.0))
    plan = make_plan(g, (1.0, 1.0), S, Options())
    driven = sum(math.dist(p.start, p.end) for p in plan.pieces if p.kind != TRANSIT)
    transit = sum(math.dist(p.start, p.end) for p in plan.pieces if p.kind == TRANSIT)
    e = plan.estimate
    assert e.distance == pytest.approx(driven + transit * S.transit_detour)
    assert e.transits == len(pieces(plan, TRANSIT)) and e.connectors == len(pieces(plan, CONNECTOR))
    assert e.time == pytest.approx(driven / S.cleaning_speed + transit * S.transit_detour / S.transit_speed
                                   + e.turns * S.turn_time)
    assert e.turns >= 2 * (e.rows - 1)


def test_the_same_inputs_give_the_same_plan():
    floor = room(3.0, 2.5)
    g = grid(floor)
    block(floor, g, 1.4, 1.1, 2.0, 1.6)
    a = make_plan(g, (0.6, 0.6), S, Options(passes=2, edges=True))
    b = make_plan(g, (0.6, 0.6), S, Options(passes=2, edges=True))
    assert a.pieces == b.pieces
    assert (a.planned == b.planned).all()
    # and with rooms: the same split, the same order of rooms, the same directions
    g = wide_and_tall_rooms()
    for options in (Options(), Options(passes=2, edges=True)):
        a = make_plan(g, (0.8, 0.8), S, options)
        b = make_plan(g, (0.8, 0.8), S, options)
        assert a.pieces == b.pieces and a.part_order == b.part_order and a.directions == b.directions
        assert (a.parts == b.parts).all()


@pytest.mark.skipif(not os.path.isfile(HOUSE_YAML), reason="no saved house map")
def test_the_house_map():
    from vacuum_coverage import mapdata
    from vacuum_mode_manager import map_files

    g = mapdata.load(HOUSE_YAML, map_files.read_zones(ZONES_YAML)).grid
    robot = (0.0, 0.0)  # the start pose, in front of the dock
    started = time.perf_counter()
    plan = make_plan(g, robot, S, Options())
    took = time.perf_counter() - started
    assert took < 1.0, f"planning the house took {took:.2f} s"
    check_chain(plan, robot)
    check_clearance(g, plan, (ROW,), S.row_clearance)
    # 97.5 % here, not 98 (since 2026-11-19): furniture legs count 5 cm bigger
    # (small_obstacle_margin), so a few short rows between the coffee table's and the desk's legs no
    # longer fit (97.85 % then). Without it, phase 20 skipped the row between the desk's legs.
    check_planned_floor_is_covered(g, plan, at_least=0.975)
    check_rows_alternate(plan)
    check_connectors(g, plan)
    # both rooms: rows on both sides of the dividing wall (house x 5.0 = map x 4.55)
    xs = [x for p in pieces(plan, ROW) for x in (p.start[0], p.end[0])]
    assert min(xs) < 2.0 and max(xs) > 5.5
    assert plan.left_out.sum() * RES * RES < 0.04 * plan.estimate.planned_area
    # room by room (the living room and the bedroom, split at the doorway), each finished before the
    # next; inside a room no trip goes across it and back
    assert plan.parts.max() >= 2
    check_parts_one_at_a_time(g, plan)
    # 3 m, not 2 (since 2026-11-19): with the furniture legs counted bigger, the best
    # order found has one 2.85 m trip in the living room (the old one-direction plan had 4.1 m).
    assert max(p.length for p in pieces(plan, TRANSIT)) < 3.0


def test_a_saved_plan_restarts_from_where_the_robot_is_now():
    g = grid(room(3.0, 2.0))
    plan = make_plan(g, (0.6, 0.6), S, Options())
    restarted = planner.restart_from(plan.pieces, (2.0, 1.5))
    assert restarted[0].kind == TRANSIT and restarted[0].start == (2.0, 1.5)
    first_row = next(p for p in plan.pieces if p.kind == ROW)
    assert restarted[0].end == first_row.start
    assert [p for p in restarted if p.kind == ROW] == pieces(plan, ROW)
    check_chain(type(plan)(restarted, plan.planned, plan.left_out, plan.unreachable, plan.estimate), (2.0, 1.5))


# ---------------------------------------------------------------- rows at an angle
@pytest.mark.parametrize("given, angle", [
    ("auto", None), (" Auto ", None), ("x", 0.0), ("y", 90.0), ("45", 45.0), (45, 45.0), ("37.5", 37.5),
    ("180", 0.0), ("-30", 150.0), (225.0, 45.0),
])
def test_a_row_direction_is_auto_or_an_angle_below_180(given, angle):
    assert planner.parse_direction(given) == angle


@pytest.mark.parametrize("given", ["diagonal", "", "nan", float("inf")])
def test_a_row_direction_that_is_none_is_refused(given):
    with pytest.raises(ValueError):
        planner.parse_direction(given)


def test_an_angle_of_0_or_90_is_the_same_plan_as_x_or_y():
    floor = room(3.0, 2.0)
    g = grid(floor)
    block(floor, g, 1.4, 0.9, 1.7, 1.2)
    for name, angle in (("x", "0"), ("y", "90"), ("x", "180")):
        assert make_plan(g, (0.6, 0.6), S, Options(direction=name)).pieces == make_plan(g, (0.6, 0.6), S, Options(direction=angle)).pieces


def _row_angle(p):
    return math.degrees(p.heading) % 180.0


@pytest.mark.parametrize("angle", [10.0, 30.0, 45.0, 120.0, 135.0, 172.5])
def test_rows_at_an_angle_run_that_way_and_keep_every_promise_but_the_share(angle):
    """Slanted rows meet straight walls at their ends, so they leave more floor than rows along the
    walls: the share promised is lower. Everything else holds as for x and y."""
    floor = room(3.0, 2.5)
    g = grid(floor)
    block(floor, g, 1.5, 1.2, 1.9, 1.6)
    robot = (0.7, 0.7)
    plan = make_plan(g, robot, S, Options(direction=str(angle)))
    rows = pieces(plan, ROW)
    assert len(rows) >= 6
    for p in rows:
        off = abs(_row_angle(p) - angle)
        assert min(off, 180.0 - off) < 1e-6, f"{p} runs at {_row_angle(p):.2f}°"
    assert plan.directions == [planner.direction_name(angle)]
    check_chain(plan, robot)
    check_clearance(g, plan, (ROW,), S.row_clearance, tolerance=0.0125)  # between two sampled centres: a quarter cell
    check_planned_floor_is_covered(g, plan, at_least=0.90)
    check_rows_alternate(plan)
    check_connectors(g, plan)


@pytest.mark.parametrize("angle", [0.0, 30.0, 90.0])
def test_neighbouring_rows_are_no_farther_apart_than_the_row_spacing(angle):
    """Closer rows than the cleaning width overlap on purpose: none of them is left out as needless."""
    g = grid(room(3.0, 2.5))
    for spacing in (0.25, 0.20, 0.15):
        plan = make_plan(g, (0.7, 0.7), Settings(row_spacing=spacing), Options(direction=f"{angle:g}"))
        across = (-math.sin(math.radians(angle)), math.cos(math.radians(angle)))
        offsets = sorted({round(p.start[0] * across[0] + p.start[1] * across[1], 6) for p in pieces(plan, ROW)})
        gaps = np.diff(offsets)
        assert len(gaps) >= 5 and gaps.max() <= spacing + 1e-6, f"rows up to {gaps.max():.3f} m apart at a spacing of {spacing}"


def test_closer_rows_mean_more_rows_and_a_longer_clean():
    g = grid(room(3.0, 2.5))
    wide = make_plan(g, (0.7, 0.7), Settings(row_spacing=0.30), Options(direction="x"))
    close = make_plan(g, (0.7, 0.7), Settings(row_spacing=0.10), Options(direction="x"))
    assert close.estimate.rows > 2 * wide.estimate.rows
    assert close.estimate.time > 2 * wide.estimate.time


def test_a_faster_cleaning_speed_shortens_the_estimate_only():
    g = grid(room(3.0, 2.5))
    slow = make_plan(g, (0.7, 0.7), Settings(cleaning_speed=0.10), Options(direction="x"))
    fast = make_plan(g, (0.7, 0.7), Settings(cleaning_speed=0.30), Options(direction="x"))
    assert slow.pieces == fast.pieces
    assert fast.estimate.time < slow.estimate.time


def test_the_second_pass_of_slanted_rows_runs_at_right_angles_to_them():
    g = grid(room(2.5, 2.0))
    plan = make_plan(g, (1.0, 1.0), S, Options(direction="30", passes=2))
    first = {round(_row_angle(p), 6) for p in pieces(plan, ROW) if p.pass_number == 1}
    second = {round(_row_angle(p), 6) for p in pieces(plan, ROW) if p.pass_number == 2}
    assert first == {30.0} and second == {120.0}
    assert plan.directions == ["30°", "120°"]


def test_edges_follow_the_walls_whatever_the_row_angle():
    g = grid(room(2.5, 2.0))
    straight = make_plan(g, (1.0, 1.0), S, Options(direction="x", edges=True))
    slanted = make_plan(g, (1.0, 1.0), S, Options(direction="45", edges=True))
    assert pieces(slanted, EDGE)
    assert {frozenset((p.start, p.end)) for p in pieces(slanted, EDGE)} <= {frozenset((p.start, p.end)) for p in pieces(straight, EDGE)}
    for p in pieces(slanted, EDGE):
        assert abs(p.start[0] - p.end[0]) < 1e-9 or abs(p.start[1] - p.end[1]) < 1e-9


def test_slanted_rows_on_the_house_map_keep_clear_and_reach_most_of_the_floor():
    if not HOUSE_YAML:
        pytest.skip("the saved house map is not here")
    from vacuum_coverage import mapdata
    from vacuum_mode_manager import map_files
    m = mapdata.load(HOUSE_YAML, map_files.read_zones(ZONES_YAML))
    for angle in ("45", "100"):
        plan = make_plan(m.grid, (0.0, 0.0), S, Options(direction=angle))
        check_chain(plan, (0.0, 0.0))
        check_clearance(m.grid, plan, (ROW,), S.row_clearance, tolerance=0.0125)
        check_planned_floor_is_covered(m.grid, plan, at_least=0.92)
        assert make_plan(m.grid, (0.0, 0.0), S, Options(direction=angle)).pieces == plan.pieces  # the same again

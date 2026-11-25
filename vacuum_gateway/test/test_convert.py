from pathlib import Path
from types import SimpleNamespace

import math

import pytest

from vacuum_gateway import convert, geometry, requirements, samples
from vacuum_gateway import vacuum_pb2 as pb


def grid(cells, stamp=0, width=3, height=2, yaw=0.0):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=stamp),
        info=SimpleNamespace(
            width=width, height=height, resolution=0.05,
            origin=SimpleNamespace(position=SimpleNamespace(x=-1.0, y=2.0),
                                   orientation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))),
        ),
        data=cells,
    )


def test_map_copies_size_origin_and_cells():
    out = convert.map_message(grid([-1, 0, 100, 0, 0, 50], yaw=0.3), version=7)
    assert (out.version, out.width, out.height) == (7, 3, 2)
    assert math.isclose(out.resolution, 0.05, rel_tol=1e-6)
    assert (out.origin_x, out.origin_y) == (-1.0, 2.0) and math.isclose(out.origin_yaw, 0.3)
    assert geometry.rle_decode(out.cells_rle).tolist() == [-1, 0, 100, 0, 0, 50]


def test_map_content_ignores_the_stamp_but_not_the_cells():
    a = convert.map_content(grid([0] * 6, stamp=1))
    assert a == convert.map_content(grid([0] * 6, stamp=2))
    assert a != convert.map_content(grid([0] * 5 + [100]))


def test_robot_state_copies_every_field():
    msg = SimpleNamespace(
        stamp=SimpleNamespace(sec=10, nanosec=250_000_000),
        state="mapping",
        previous_state="idle",
        reason="requested",
        transitioning=True,
        allowed_transitions=("idle", "manual"),
        faults=["lidar data stopped"],
        current_map="house",
    )
    out = convert.robot_state(msg)
    assert out.state == "mapping" and out.previous_state == "idle" and out.reason == "requested"
    assert out.transitioning is True
    assert list(out.allowed_transitions) == ["idle", "manual"]
    assert list(out.faults) == ["lidar data stopped"]
    assert out.current_map == "house"
    assert out.entered_time == 10.25
    assert out.making_map is False  # an older robot without the field: the app goes by the mode then


def test_robot_state_says_a_map_is_being_made():
    msg = SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0), state="exploring", previous_state="mapping", reason="requested",
                          transitioning=False, allowed_transitions=["mapping", "idle", "failsafe"], faults=[], current_map="house",
                          making_map=True)
    assert convert.robot_state(msg).making_map is True


def exploration(**extra):
    fields = dict(state="exploring", message="exploring", job_id=4, started=1794988290.5, places_left=3, mapped_area=18.25,
                  exploring_time=130.5, time_limit=1200.0, distance=12.5, goals_sent=6, goals_failed=1,
                  stamp=SimpleNamespace(sec=5, nanosec=0))
    fields.update(extra)
    return SimpleNamespace(**fields)


@pytest.mark.parametrize("state,expected", [
    ("none", "STATE_NONE"), ("starting", "STATE_STARTING"), ("exploring", "STATE_EXPLORING"), ("returning", "STATE_RETURNING"),
    ("ending", "STATE_ENDING"), ("finished", "STATE_FINISHED"), ("stopped", "STATE_STOPPED"), ("failed", "STATE_FAILED"),
    ("something new", "STATE_NONE"),
])
def test_exploration_status_states(state, expected):
    out = convert.exploration_status(exploration(state=state))
    assert out.state == getattr(pb.ExplorationStatus, expected)


def test_exploration_status_copies_every_field_but_the_stamp():
    out = convert.exploration_status(exploration())
    assert (out.message, out.job_id, out.started, out.places_left, out.mapped_area, out.exploring_time, out.time_limit,
            out.distance, out.goals_sent, out.goals_failed) == ("exploring", 4, 1794988290.5, 3, 18.25, 130.5, 1200.0, 12.5, 6, 1)
    # two copies saying the same thing are equal, so the stream does not send an unchanged one early
    assert out == convert.exploration_status(exploration(stamp=SimpleNamespace(sec=9, nanosec=1)))


def test_frontiers_in_centimetres():
    poses = [SimpleNamespace(position=SimpleNamespace(x=1.25, y=-0.5, z=0.0)),
             SimpleNamespace(position=SimpleNamespace(x=6.054, y=2.396, z=0.0))]
    out = convert.frontiers(SimpleNamespace(poses=poses), 4)
    assert out.job_id == 4 and list(out.points_cm) == [125, -50, 605, 240]
    assert list(convert.frontiers(SimpleNamespace(poses=[]), 0).points_cm) == []


def test_safety_status_copies_every_field_but_the_stamp():
    msg = SimpleNamespace(source="manual", blocked_reason="command timeout", cliff=False, bumper=True,
                          off_floor=False, reflex_active=True, stale=["cliff"], stamp="ignored")
    out = convert.safety_status(msg)
    assert (out.source, out.blocked_reason, out.cliff, out.bumper, out.off_floor, out.reflex_active) == (
        "manual", "command timeout", False, True, False, True)
    assert list(out.stale) == ["cliff"]


def test_same_content_gives_same_bytes():
    # The gateway decides "unchanged" by comparing bytes; the stamp must not make them differ.
    a = SimpleNamespace(source="none", blocked_reason="idle", cliff=False, bumper=False, off_floor=False,
                        reflex_active=False, stale=[], stamp=1)
    b = SimpleNamespace(**{**vars(a), "stamp": 2})
    assert convert.safety_status(a).SerializeToString() == convert.safety_status(b).SerializeToString()


def test_hello_carries_the_protocol_version():
    out = convert.hello("Vacuum", "0.1.0")
    assert out.protocol_version == pb.PROTOCOL_VERSION == 1
    assert out.robot_name == "Vacuum" and out.gateway_version == "0.1.0"


def _samples_folder():
    """proto/samples of the project. It is kept in the main project, not in this package, so it is
    looked for upwards from here; None when this package is used without it."""
    for folder in Path(__file__).resolve().parents:
        if (folder / "proto" / "samples").is_dir():
            return folder / "proto" / "samples"
    return None


@pytest.mark.skipif(_samples_folder() is None, reason="proto/samples is in the main project")
def test_samples_on_disk_are_current():
    # proto/samples is what the app's tests decode. If this fails, run tools/gen_proto.sh.
    folder = _samples_folder()
    on_disk = {p.name: p.read_bytes() for p in folder.glob("*.bin")}
    assert on_disk == samples.build()


def test_requirements_are_met_here():
    assert requirements.problems() == []


def test_requirement_versions_are_read_sensibly():
    assert requirements._version("16.0") == (16, 0)
    assert requirements._version("6.33.6") == (6, 33)
    assert requirements._version("6.31.1rc2") == (6, 31)
    assert requirements._version("13") == (13, 0)


def test_route_in_centimetres():
    out = convert.route(5.0, [(0.0, 0.0), (1.234, -2.5)])
    assert list(out.points_cm) == [0, 0, 123, -250] and out.stamp == 5.0


def test_goal_statuses():
    def goal(byte, sec, status):
        return SimpleNamespace(goal_info=SimpleNamespace(goal_id=SimpleNamespace(uuid=[byte] * 16),
                                                         stamp=SimpleNamespace(sec=sec, nanosec=500_000_000)), status=status)
    msg = SimpleNamespace(status_list=[goal(1, 10, 2), goal(2, 11, 4)])
    assert convert.goal_statuses(msg) == [(bytes([1] * 16), 10.5, 2), (bytes([2] * 16), 11.5, 4)]


# ---------------------------------------------------------------- cleaning
def ros_job(**kw):
    base = dict(what=1, area=[0.5, 0.25, 2.0, 0.25, 2.0, 1.5], spot_x=0.0, spot_y=0.0, plan_name="", direction="x",
                passes=2, edges=2, return_to_dock=1)
    base.update(kw)
    return SimpleNamespace(**base)


def ros_status(**kw):
    base = dict(
        stamp=SimpleNamespace(sec=5, nanosec=0), state="cleaning", message="row 3 of 10", job_id=4, target="area",
        job=ros_job(), started=1_790_000_000.5, coverage=42.5, cleaned_area=3.25, planned_area=7.5, distance=12.0,
        cleaning_time=95.0, time_left=130.0, rows_done=2, rows_total=10, rows_skipped=1, current_row=3, edges_done=0,
        edges_total=4, edges_skipped=0, current_edge=1, bumps=1, pauses=2, piece=6)
    base.update(kw)
    return SimpleNamespace(**base)


def ros_estimate():
    return SimpleNamespace(rows=10, edges=4, turns=19, transits=2, distance=31.5, planned_area=7.5, left_out_area=0.25,
                           unreachable_area=0.0, time=320.0, planning_time=0.2)


def test_cleaning_status_copies_every_number_and_names_the_state():
    out = convert.cleaning_status(ros_status())
    assert out.state == pb.CleaningStatus.STATE_CLEANING and out.message == "row 3 of 10"
    assert (out.job_id, out.target, out.started) == (4, "area", 1_790_000_000.5)
    assert (out.coverage, out.cleaned_area, out.planned_area, out.distance) == (42.5, 3.25, 7.5, 12.0)
    assert (out.cleaning_time, out.time_left) == (95.0, 130.0)
    assert (out.rows_done, out.rows_total, out.rows_skipped, out.current_row) == (2, 10, 1, 3)
    assert (out.edges_done, out.edges_total, out.edges_skipped, out.current_edge) == (0, 4, 0, 1)
    assert (out.bumps, out.pauses, out.piece) == (1, 2, 6)
    assert out.job.what == pb.CleaningJob.WHAT_AREA and list(out.job.area) == [0.5, 0.25, 2.0, 0.25, 2.0, 1.5]
    assert out.job.edges == pb.CleaningJob.OPTION_ON and out.job.return_to_dock == pb.CleaningJob.OPTION_OFF
    for name, value in (("none", 0), ("starting", 1), ("paused", 3), ("finished", 4), ("stopped", 5), ("failed", 6),
                        ("ending", 7), ("odd", 0)):
        assert convert.cleaning_status(ros_status(state=name)).state == value


def test_cleaning_status_leaves_out_the_stamp_so_equal_copies_compare_equal():
    a = convert.cleaning_status(ros_status(stamp=SimpleNamespace(sec=1, nanosec=0))).SerializeToString()
    b = convert.cleaning_status(ros_status(stamp=SimpleNamespace(sec=2, nanosec=0))).SerializeToString()
    assert a == b


def test_a_cleaning_job_from_the_phone_becomes_the_robots_fields():
    job = pb.CleaningJob(what=pb.CleaningJob.WHAT_SPOT, spot_x=1.5, spot_y=-0.5, passes=2, edges=pb.CleaningJob.OPTION_OFF,
                         return_to_dock=pb.CleaningJob.OPTION_ON, direction="y")
    fields = convert.cleaning_job_fields(job)
    assert fields == {"what": 2, "area": [], "spot_x": 1.5, "spot_y": -0.5, "plan_name": "", "direction": "y", "passes": 2,
                      "edges": 1, "return_to_dock": 2, "room": ""}


def test_a_bad_cleaning_job_from_the_phone_is_refused():
    assert convert.cleaning_job_problem(pb.CleaningJob(what=pb.CleaningJob.WHAT_AREA, area=[0.0, 0.0, 1.0])) is not None
    assert convert.cleaning_job_problem(pb.CleaningJob(what=pb.CleaningJob.WHAT_SPOT, spot_x=math.nan)) is not None
    assert convert.cleaning_job_problem(pb.CleaningJob(what=7)) is not None
    assert convert.cleaning_job_problem(pb.CleaningJob(passes=3)) is not None
    assert convert.cleaning_job_problem(pb.CleaningJob()) is None


def test_cleaning_plan_pieces_in_centimetres():
    msg = SimpleNamespace(job_id=4, target="area", estimate=ros_estimate(), pieces=[
        SimpleNamespace(kind=2, x0=0.0, y0=0.0, x1=0.504, y1=0.25, number=0, pass_number=0, region=0),
        SimpleNamespace(kind=0, x0=0.504, y0=0.25, x1=2.0, y1=0.25, number=1, pass_number=1, region=1),
        SimpleNamespace(kind=3, x0=2.0, y0=0.25, x1=2.0, y1=-1.256, number=1, pass_number=0, region=0)])
    out = convert.cleaning_plan(msg)
    assert (out.job_id, out.target, out.estimate.rows, out.estimate.time) == (4, "area", 10, 320.0)
    kinds = [p.kind for p in out.pieces]
    assert kinds == [pb.CleaningPlan.KIND_TRANSIT, pb.CleaningPlan.KIND_ROW, pb.CleaningPlan.KIND_EDGE]
    row = out.pieces[1]
    assert (row.x0_cm, row.y0_cm, row.x1_cm, row.y1_cm, row.number, row.pass_number) == (50, 25, 200, 25, 1, 1)
    assert out.pieces[2].y1_cm == -126


def test_cleaned_area_is_run_length_encoded_like_the_map():
    out = convert.cleaned_area(grid([0, 0, 100, 100, 100, 0]), job_id=3)
    assert (out.job_id, out.width, out.height) == (3, 3, 2)
    assert (out.origin_x, out.origin_y) == (-1.0, 2.0)
    assert geometry.rle_decode(out.cells_rle).tolist() == [0, 0, 100, 100, 100, 0]


def test_cleaning_history_records_and_totals():
    record = SimpleNamespace(id="20261118-101500-3", started=1.0e9, ended=1.0e9 + 600, status=ros_status(state="finished"))
    totals = SimpleNamespace(cleans=23, area=412.5, time=18600.0, distance=1500.0)
    out = convert.cleaning_history([record], totals)
    assert out.totals.cleans == 23 and out.totals.area == 412.5
    [r] = out.records
    assert (r.id, r.started, r.ended, r.status.state) == ("20261118-101500-3", 1.0e9, 1.0e9 + 600, pb.CleaningStatus.STATE_FINISHED)
    assert r.picture_png == b""
    one = convert.cleaning_record(record, b"\x89PNG")
    assert one.picture_png == b"\x89PNG"


def test_cleaning_plan_info():
    info = SimpleNamespace(name="Living room", job=ros_job(), estimate=ros_estimate(), map_name="house", map_changed=True,
                           saved=1.79e9)
    out = convert.cleaning_plan_info(info)
    assert (out.name, out.map_changed, out.saved, out.estimate.rows) == ("Living room", True, 1.79e9, 10)


def test_reply_carries_the_cleaning_fields():
    est = convert.cleaning_estimate(ros_estimate())
    out = convert.reply(5, pb.Reply.RESULT_OK, "10 rows", estimate=est, replanned=True, job_id=12)
    assert out.estimate.rows == 10 and out.replanned and out.request_id == 5 and out.job_id == 12


def test_an_empty_cleaning_plan_is_still_a_message():
    out = convert.cleaning_plan(SimpleNamespace(job_id=0, target="", estimate=ros_estimate(), pieces=[]))
    assert out.job_id == 0 and len(out.pieces) == 0


def _settings_msg(**values):
    base = dict(cleaning_speed=0.25, manual_speed=0.15, row_spacing=0.2, row_angle_auto=False, row_angle=45.0,
                CLEANING_SPEED_MIN=0.1, CLEANING_SPEED_MAX=0.3, MANUAL_SPEED_MIN=0.1, MANUAL_SPEED_MAX=0.3,
                ROW_SPACING_MIN=0.1, ROW_SPACING_MAX=0.3, ROW_SPACING_STEP=0.05, MANUAL_TURN_PER_SPEED=4.0)
    base.update(values)
    return SimpleNamespace(**base)


def test_robot_settings_carry_the_values_the_limits_and_the_turn_speed():
    m = convert.robot_settings(_settings_msg())
    assert (m.cleaning_speed, m.manual_speed, m.row_spacing, m.row_angle_auto, m.row_angle) == pytest.approx((0.25, 0.15, 0.2, False, 45.0))
    assert (m.limits.cleaning_speed_min, m.limits.cleaning_speed_max) == pytest.approx((0.1, 0.3))
    assert (m.limits.manual_speed_min, m.limits.manual_speed_max) == pytest.approx((0.1, 0.3))
    assert (m.limits.row_spacing_min, m.limits.row_spacing_max, m.limits.row_spacing_step) == pytest.approx((0.1, 0.3, 0.05))
    assert m.manual_turn_speed == pytest.approx(0.6)


def test_settings_from_the_phone_fill_the_robots_message_and_nothing_else():
    msg = SimpleNamespace(cleaning_speed=0.0, manual_speed=0.0, row_spacing=0.0, row_angle_auto=True, row_angle=0.0)
    convert.fill_robot_settings(pb.RobotSettings(cleaning_speed=0.15, manual_speed=0.3, row_spacing=0.1, row_angle_auto=False,
                                                 row_angle=120.0, manual_turn_speed=99.0), msg)
    assert vars(msg) == pytest.approx(dict(cleaning_speed=0.15, manual_speed=0.3, row_spacing=0.1, row_angle_auto=False, row_angle=120.0))


def test_the_settings_message_limits_are_the_robot_settings_limits():
    from vacuum_interfaces.msg import RobotSettings
    from vacuum_mode_manager import robot_settings
    for name in ("CLEANING_SPEED_MIN", "CLEANING_SPEED_MAX", "MANUAL_SPEED_MIN", "MANUAL_SPEED_MAX", "ROW_SPACING_MIN",
                 "ROW_SPACING_MAX", "ROW_SPACING_STEP", "MANUAL_TURN_PER_SPEED"):
        assert getattr(RobotSettings, name) == pytest.approx(getattr(robot_settings, name)), name


def test_rooms_and_the_assistant_answer_in_centimetres():
    from types import SimpleNamespace as N
    room = N(name="Kitchen", outline=N(points=[N(x=0.0, y=0.0, z=0.0), N(x=2.004, y=0.0, z=0.0), N(x=2.0, y=-1.5, z=0.0)]),
             x=1.0, y=-0.5, area=3.25)
    out = convert.rooms([room], "home")
    assert out.map_name == "home" and list(out.rooms[0].points_cm) == [0, 0, 200, 0, 200, -150]
    assert (out.rooms[0].x_cm, out.rooms[0].y_cm, out.rooms[0].area) == (100, -50, 3.25)
    back = convert.room_fields(out.rooms[0])
    assert back == {"name": "Kitchen", "outline": [(0.0, 0.0), (2.0, 0.0), (2.0, -1.5)], "x": 1.0, "y": -0.5, "area": 3.25}
    answer = convert.assistant_answer(N(reply="On my way.", tools_used=["go_to_room"], picture_jpeg=[1, 2], mark=[1.5, -2.25]))
    assert answer.text == "On my way." and answer.picture_jpeg == bytes([1, 2]) and list(answer.mark_cm) == [150, -225]
    assert list(convert.assistant_answer(N(reply="", tools_used=[], picture_jpeg=[], mark=[])).mark_cm) == []


def test_a_change_to_the_rooms_is_two_rooms_to_join_or_a_line_to_cut_one_along():
    assert convert.rooms_change(pb.EditRooms(join=[2, 0])) == {"join": (2, 0)}
    assert convert.rooms_change(pb.EditRooms(cut_cm=[150, -25, 150, 310])) == {"line": [1.5, -0.25, 1.5, 3.1]}
    for neither in (pb.EditRooms(), pb.EditRooms(join=[1]), pb.EditRooms(join=[1, 2, 3]), pb.EditRooms(cut_cm=[1, 2, 3]),
                    pb.EditRooms(join=[0, 1], cut_cm=[0, 0, 100, 100])):
        assert convert.rooms_change(neither) is None

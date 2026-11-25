"""Sample messages written by the gateway's own code, for the app's tests.

The app's tests decode the data samples and check every field, and encode the same commands
and compare them byte for byte with the command samples. So a difference between how the two
ends read or write the format shows up without a phone. tools/gen_proto.sh writes the files to
proto/samples; test_convert.py checks that the files there are still what this code writes.
"""

import os
from types import SimpleNamespace
from typing import Dict

from . import convert, framing
from . import vacuum_pb2 as pb

_LIMIT = 1024 * 1024

# A 20 x 15 map: a run of 270 unknown cells (longer than one run can hold), then every kind of
# value. The app's test expects exactly these cells.
SAMPLE_MAP_CELLS = [-1] * 270 + [0] * 20 + [100] * 5 + [50, 0, 100, -1, 0]

# A goal id as Nav2 makes them: 16 bytes. The app's tests expect exactly these.
SAMPLE_GOAL_ID = bytes(range(0xA0, 0xB0))


def _frame(msg_type: int, message, channel: int = pb.CHANNEL_DATA) -> bytes:
    return framing.encode(channel, msg_type, message.SerializeToString(), _LIMIT)


def build() -> Dict[str, bytes]:
    """File name -> frame. The values are arbitrary but cover every field, empty and non-empty
    lists, true and false, and text outside plain ASCII. The app's test expects exactly these."""
    robot_state = SimpleNamespace(
        stamp=SimpleNamespace(sec=1234, nanosec=500_000_000),
        state="docking",
        previous_state="navigation",
        reason="docking: final approach (0.4 m, 2°)",
        transitioning=False,
        allowed_transitions=["idle", "mapping", "navigation", "manual", "failsafe"],
        faults=[],
        current_map="house",
        making_map=False,
    )
    exploring_state = SimpleNamespace(
        stamp=SimpleNamespace(sec=2000, nanosec=0),
        state="exploring",
        previous_state="mapping",
        reason="requested",
        transitioning=False,
        allowed_transitions=["mapping", "idle", "failsafe"],
        faults=[],
        current_map="house",
        making_map=True,
    )
    exploration = SimpleNamespace(
        state="exploring", message="looking again: making sure nothing is left", job_id=4, started=1794988290.5, places_left=3,
        mapped_area=18.25, exploring_time=130.5, time_limit=1200.0, distance=12.5, goals_sent=6, goals_failed=1)
    frontier_poses = SimpleNamespace(poses=[SimpleNamespace(position=SimpleNamespace(x=1.25, y=-0.5, z=0.0)),
                                            SimpleNamespace(position=SimpleNamespace(x=6.05, y=2.4, z=0.0))])
    failsafe_state = SimpleNamespace(
        stamp=SimpleNamespace(sec=0, nanosec=0),
        state="failsafe",
        previous_state="manual",
        reason="fault: lidar data stopped",
        transitioning=True,
        allowed_transitions=[],
        faults=["lidar data stopped", "bumper data stopped"],
        current_map="",
        making_map=False,
    )
    safety = SimpleNamespace(
        source="nav2", blocked_reason="cliff ahead", cliff=True, bumper=False, off_floor=False, reflex_active=True, stale=["bumper"]
    )
    grid = SimpleNamespace(
        info=SimpleNamespace(
            width=20, height=15, resolution=0.05,
            origin=SimpleNamespace(position=SimpleNamespace(x=-1.25, y=-2.5), orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)),
        ),
        data=SAMPLE_MAP_CELLS,
    )
    job = SimpleNamespace(what=1, area=[1.2, 0.2, 3.2, 0.2, 3.2, 1.4, 1.2, 1.4], spot_x=0.0, spot_y=0.0, plan_name="",
                          direction="", passes=2, edges=2, return_to_dock=1)
    estimate = SimpleNamespace(rows=31, edges=12, turns=64, transits=3, distance=88.5, planned_area=21.25, left_out_area=0.5,
                               unreachable_area=1.75, time=840.0, planning_time=0.25)
    status = SimpleNamespace(
        state="cleaning", message="row 12 of 31", job_id=7, target="Living room é", job=job, started=1794988290.5,
        coverage=43.5, cleaned_area=9.125, planned_area=21.25, distance=38.25, cleaning_time=370.0, time_left=360.0,
        rows_done=11, rows_total=31, rows_skipped=1, current_row=12, edges_done=0, edges_total=12, edges_skipped=0,
        current_edge=1, bumps=2, pauses=1, piece=23)
    finished = SimpleNamespace(**dict(vars(status), state="stopped", message="stopped: the robot left cleaning mode (now idle)",
                                      current_row=0, current_edge=0, time_left=0.0))
    plan = SimpleNamespace(job_id=7, target="area", estimate=estimate, pieces=[
        SimpleNamespace(kind=2, x0=0.0, y0=0.0, x1=1.225, y1=0.225, number=0, pass_number=0, region=0),
        SimpleNamespace(kind=0, x0=1.225, y0=0.225, x1=3.175, y1=0.225, number=1, pass_number=1, region=1),
        SimpleNamespace(kind=1, x0=3.175, y0=0.225, x1=3.175, y1=0.475, number=0, pass_number=1, region=1),
        SimpleNamespace(kind=0, x0=3.175, y0=0.475, x1=1.225, y1=0.475, number=2, pass_number=1, region=1),
        SimpleNamespace(kind=3, x0=1.0, y0=-0.5, x1=-1.5, y1=-0.5, number=1, pass_number=0, region=0)])
    cleaned = SimpleNamespace(info=grid.info, data=[0] * 200 + [100] * 70 + [0] * 30)
    plan_info = SimpleNamespace(name="Under the table", job=job, estimate=estimate, map_name="house", map_changed=True,
                                saved=1794988000.25)
    record = SimpleNamespace(id="20261118-101500-7", started=1794988290.5, ended=1794989130.75, status=finished)
    totals = SimpleNamespace(cleans=23, area=412.5, time=18600.0, distance=1520.25)
    # Values a 32-bit float holds exactly, so both ends read back the same numbers.
    settings = SimpleNamespace(
        cleaning_speed=0.25, manual_speed=0.125, row_spacing=0.25, row_angle_auto=False, row_angle=37.5,
        CLEANING_SPEED_MIN=0.125, CLEANING_SPEED_MAX=0.3125, MANUAL_SPEED_MIN=0.0625, MANUAL_SPEED_MAX=0.375,
        ROW_SPACING_MIN=0.125, ROW_SPACING_MAX=0.5, ROW_SPACING_STEP=0.0625, MANUAL_TURN_PER_SPEED=4.0)
    def poly(*pts):
        return SimpleNamespace(points=[SimpleNamespace(x=x, y=y, z=0.0) for x, y in pts])

    room_list = [
        SimpleNamespace(name="Kitchen", outline=poly((6.5, 0.0), (10.6, 0.0), (10.6, 3.2), (6.5, 3.2)), x=8.55, y=1.6, area=12.75),
        SimpleNamespace(name="Bedroom é", outline=poly((0.0, 4.1), (3.9, 4.1), (3.9, 7.6)), x=-1.25, y=5.5, area=6.5),
    ]
    answer = SimpleNamespace(reply="I'm in the kitchen. I can see a table and two stools.", tools_used=["go_to_room", "look"],
                             picture_jpeg=bytes([0xFF, 0xD8, 0xFF, 0xD9]), mark=[8.55, -1.25])
    spot = pb.CleaningJob(what=pb.CleaningJob.WHAT_SPOT, spot_x=2.5, spot_y=-1.25, return_to_dock=pb.CleaningJob.OPTION_OFF)
    return {
        "cleaning_status.bin": _frame(pb.DATA_CLEANING_STATUS, convert.cleaning_status(status)),
        "cleaning_plan.bin": _frame(pb.DATA_CLEANING_PLAN, convert.cleaning_plan(plan)),
        "cleaned_area.bin": _frame(pb.DATA_CLEANED_AREA, convert.cleaned_area(cleaned, 7)),
        "reply_start_cleaning.bin": _frame(pb.DATA_REPLY, convert.reply(
            59, pb.Reply.RESULT_OK, "31 rows, 88 m, about 14 min", estimate=convert.cleaning_estimate(estimate), job_id=7)),
        "cleaning_status_ending.bin": _frame(pb.DATA_CLEANING_STATUS, convert.cleaning_status(SimpleNamespace(**dict(
            vars(status), state="ending", message="ending: going back to the dock", current_row=0, current_edge=0, piece=0,
            time_left=0.0)))),
        "cleaning_plan_empty.bin": _frame(pb.DATA_CLEANING_PLAN, convert.cleaning_plan(SimpleNamespace(
            job_id=0, target="", estimate=SimpleNamespace(**{k: 0 for k in vars(estimate)}), pieces=[]))),
        "reply_preview.bin": _frame(pb.DATA_REPLY, convert.reply(
            50, pb.Reply.RESULT_OK, "the map has changed since this plan was saved: its rows were planned again",
            estimate=convert.cleaning_estimate(estimate), replanned=True)),
        "reply_plans.bin": _frame(pb.DATA_REPLY, convert.reply(51, pb.Reply.RESULT_OK, "", plans=[convert.cleaning_plan_info(plan_info)])),
        "reply_history.bin": _frame(pb.DATA_REPLY, convert.reply(52, pb.Reply.RESULT_OK, "",
                                                                   history=convert.cleaning_history([record], totals))),
        "reply_record.bin": _frame(pb.DATA_REPLY, convert.reply(53, pb.Reply.RESULT_OK, "",
                                                                  record=convert.cleaning_record(record, bytes(range(16))))),
        "command_start_cleaning.bin": _frame(pb.COMMAND_START_CLEANING, pb.StartCleaning(request_id=60, job=convert.cleaning_job(job)),
                                             pb.CHANNEL_COMMAND),
        "command_pause_cleaning.bin": _frame(pb.COMMAND_PAUSE_CLEANING, pb.PauseCleaning(request_id=61, resume=True), pb.CHANNEL_COMMAND),
        "command_preview_cleaning.bin": _frame(pb.COMMAND_PREVIEW_CLEANING, pb.PreviewCleaning(request_id=62, job=spot), pb.CHANNEL_COMMAND),
        "command_save_cleaning_plan.bin": _frame(pb.COMMAND_SAVE_CLEANING_PLAN, pb.SaveCleaningPlan(
            request_id=63, name="Under the table", job=convert.cleaning_job(job), replace=True), pb.CHANNEL_COMMAND),
        "command_list_cleaning_plans.bin": _frame(pb.COMMAND_LIST_CLEANING_PLANS, pb.ListCleaningPlans(request_id=64), pb.CHANNEL_COMMAND),
        "command_delete_cleaning_plan.bin": _frame(pb.COMMAND_DELETE_CLEANING_PLAN, pb.DeleteCleaningPlan(request_id=65, name="Hall"),
                                                   pb.CHANNEL_COMMAND),
        "command_list_cleaning_history.bin": _frame(pb.COMMAND_LIST_CLEANING_HISTORY, pb.ListCleaningHistory(request_id=66, max_records=20),
                                                    pb.CHANNEL_COMMAND),
        "command_get_cleaning_record.bin": _frame(pb.COMMAND_GET_CLEANING_RECORD, pb.GetCleaningRecord(request_id=67, id="20261118-101500-7"),
                                                  pb.CHANNEL_COMMAND),
        "hello.bin": _frame(pb.DATA_HELLO, convert.hello("Vacuum", "0.1.0")),
        "heartbeat.bin": _frame(pb.DATA_HEARTBEAT, convert.heartbeat(7)),
        "robot_state.bin": _frame(pb.DATA_ROBOT_STATE, convert.robot_state(robot_state)),
        "robot_state_failsafe.bin": _frame(pb.DATA_ROBOT_STATE, convert.robot_state(failsafe_state)),
        "robot_state_exploring.bin": _frame(pb.DATA_ROBOT_STATE, convert.robot_state(exploring_state)),
        "exploration_status.bin": _frame(pb.DATA_EXPLORATION_STATUS, convert.exploration_status(exploration)),
        "frontiers.bin": _frame(pb.DATA_FRONTIERS, convert.frontiers(frontier_poses, 4)),
        "reply_start_exploration.bin": _frame(pb.DATA_REPLY, convert.reply(70, pb.Reply.RESULT_OK, "exploring: a new map begins",
                                                                             job_id=4)),
        "command_start_exploration.bin": _frame(pb.COMMAND_START_EXPLORATION, pb.StartExploration(request_id=71, time_limit=120.0),
                                                pb.CHANNEL_COMMAND),
        "safety_status.bin": _frame(pb.DATA_SAFETY_STATUS, convert.safety_status(safety)),
        "map.bin": _frame(pb.DATA_MAP, convert.map_message(grid, 3)),
        "robot_pose.bin": _frame(pb.DATA_ROBOT_POSE, convert.robot_pose(1234.5, 1.25, -0.5, 1.5)),
        "reply.bin": _frame(pb.DATA_REPLY, convert.reply(
            42, pb.Reply.RESULT_REFUSED, "no saved map yet: switch to mapping, drive around, and save a map first")),
        "route.bin": _frame(pb.DATA_ROUTE, convert.route(1234.5, [(0.0, 0.0), (0.05, 0.0), (1.234, -2.5)])),
        "navigation_status.bin": _frame(pb.DATA_NAVIGATION, pb.NavigationStatus(
            goal_id=SAMPLE_GOAL_ID, state=pb.NavigationStatus.STATE_ON_THE_WAY, distance_remaining=3.25)),
        "reply_go_to.bin": _frame(pb.DATA_REPLY, convert.reply(43, pb.Reply.RESULT_OK, "on the way", goal_id=SAMPLE_GOAL_ID)),
        "reply_name_taken.bin": _frame(pb.DATA_REPLY, convert.reply(
            48, pb.Reply.RESULT_ALREADY_EXISTS, "a map named 'house' already exists; save with replace to overwrite it")),
        "no_go_zones.bin": _frame(pb.DATA_NO_GO_ZONES, convert.no_go_zones(
            [("stairwell", [(4.55, 2.0), (5.65, 2.0), (5.65, 3.3), (4.55, 3.3)]), ("rug é", [(-1.0, -0.5), (0.0, 0.0), (-0.5, 1.0)])],
            "house")),
        "robot_settings.bin": _frame(pb.DATA_ROBOT_SETTINGS, convert.robot_settings(settings)),
        "reply_settings_refused.bin": _frame(pb.DATA_REPLY, convert.reply(
            73, pb.Reply.RESULT_REFUSED, "the cleaning speed must be between 0.10 and 0.30 m/s (asked for 0.50)")),
        "rooms.bin": _frame(pb.DATA_ROOMS, convert.rooms(room_list, "home")),
        "assistant_status.bin": _frame(pb.DATA_ASSISTANT_STATUS, convert.assistant_status(
            SimpleNamespace(busy=True, conversation="phone-1a2b", doing="going to the kitchen"))),
        "reply_answer.bin": _frame(pb.DATA_REPLY, convert.reply(74, pb.Reply.RESULT_OK, "", answer=convert.assistant_answer(answer))),
        "reply_rooms.bin": _frame(pb.DATA_REPLY, convert.reply(75, pb.Reply.RESULT_OK, "found 2 rooms",
                                                                 rooms=convert.rooms(room_list, "home"))),
        # Commands, as the app must write them.
        "command_change_mode.bin": _frame(pb.COMMAND_CHANGE_MODE, pb.ChangeMode(request_id=42, mode="docking"), pb.CHANNEL_COMMAND),
        "command_ping.bin": _frame(pb.COMMAND_PING, pb.Ping(request_id=4000000000), pb.CHANNEL_COMMAND),
        "command_go_to.bin": _frame(pb.COMMAND_GO_TO, pb.GoTo(request_id=44, x=1.5, y=-0.25, heading=-1.25), pb.CHANNEL_COMMAND),
        "command_cancel_goal.bin": _frame(pb.COMMAND_CANCEL_GOAL, pb.CancelGoal(request_id=45, goal_id=SAMPLE_GOAL_ID), pb.CHANNEL_COMMAND),
        "command_drive.bin": _frame(pb.COMMAND_DRIVE, pb.Drive(request_id=46, forward=0.25, turn=-1.5), pb.CHANNEL_COMMAND),
        "command_save_map.bin": _frame(pb.COMMAND_SAVE_MAP, pb.SaveMap(request_id=47, name="upstairs-2", replace=True), pb.CHANNEL_COMMAND),
        "command_set_no_go_zones.bin": _frame(pb.COMMAND_SET_NO_GO_ZONES, pb.SetNoGoZones(request_id=68, map_name="house", zones=[
            pb.NoGoZones.Zone(name="stairwell", points_cm=[455, 200, 565, 200, 565, 330, 455, 330]),
            pb.NoGoZones.Zone(name="rug é", points_cm=[-100, -50, 0, 0, -50, 100])]), pb.CHANNEL_COMMAND),
        "command_find_rooms.bin": _frame(pb.COMMAND_FIND_ROOMS, pb.FindRooms(request_id=76), pb.CHANNEL_COMMAND),
        "command_set_rooms.bin": _frame(pb.COMMAND_SET_ROOMS, pb.SetRooms(
            request_id=77, map_name="home", rooms=convert.rooms(room_list, "home").rooms), pb.CHANNEL_COMMAND),
        "command_edit_rooms_join.bin": _frame(pb.COMMAND_EDIT_ROOMS, pb.EditRooms(
            request_id=81, rooms=convert.rooms(room_list, "home").rooms, join=[1, 0]), pb.CHANNEL_COMMAND),
        "command_edit_rooms_cut.bin": _frame(pb.COMMAND_EDIT_ROOMS, pb.EditRooms(
            request_id=82, rooms=convert.rooms(room_list, "home").rooms, cut_cm=[855, 20, 855, -150]), pb.CHANNEL_COMMAND),
        "command_ask_assistant.bin": _frame(pb.COMMAND_ASK_ASSISTANT, pb.AskAssistant(
            request_id=78, conversation="phone-1a2b", text="Clean the kitchen, please é"), pb.CHANNEL_COMMAND),
        "command_name_rooms.bin": _frame(pb.COMMAND_NAME_ROOMS, pb.NameRooms(
            request_id=79, rooms=convert.rooms(room_list, "home").rooms), pb.CHANNEL_COMMAND),
        "command_start_cleaning_room.bin": _frame(pb.COMMAND_START_CLEANING, pb.StartCleaning(
            request_id=80, job=pb.CleaningJob(what=pb.CleaningJob.WHAT_ROOM, room="Kitchen")), pb.CHANNEL_COMMAND),
        "command_set_robot_settings.bin": _frame(pb.COMMAND_SET_ROBOT_SETTINGS, pb.SetRobotSettings(
            request_id=72, settings=pb.RobotSettings(cleaning_speed=0.25, manual_speed=0.125, row_spacing=0.25,
                                                     row_angle_auto=False, row_angle=37.5)), pb.CHANNEL_COMMAND),
    }


def write(directory: str) -> None:
    os.makedirs(directory, exist_ok=True)
    for name, data in build().items():
        with open(os.path.join(directory, name), "wb") as f:
            f.write(data)

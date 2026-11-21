"""ROS messages to the app's messages. No ROS imports: anything with the right fields will do,
which lets the tests and the sample writer use plain objects.
"""

import math
from typing import List, Optional

import numpy as np

from . import geometry
from . import vacuum_pb2 as pb


def hello(robot_name: str, gateway_version: str) -> pb.Hello:
    return pb.Hello(protocol_version=pb.PROTOCOL_VERSION, robot_name=robot_name, gateway_version=gateway_version)


def heartbeat(sequence: int) -> pb.Heartbeat:
    return pb.Heartbeat(sequence=sequence)


def reply(request_id: int, result: int, message: str = "", **fields) -> pb.Reply:
    """The answer to a command. `fields` are the Reply's other fields (goal_id, estimate, ...)."""
    return pb.Reply(request_id=request_id, result=result, message=message, **fields)


def robot_state(msg) -> pb.RobotState:
    """From vacuum_interfaces/msg/RobotState."""
    return pb.RobotState(
        state=msg.state,
        previous_state=msg.previous_state,
        reason=msg.reason,
        transitioning=msg.transitioning,
        allowed_transitions=list(msg.allowed_transitions),
        faults=list(msg.faults),
        current_map=msg.current_map,
        entered_time=msg.stamp.sec + msg.stamp.nanosec * 1e-9,
        making_map=bool(getattr(msg, "making_map", False)),
    )


def map_message(msg, version: int) -> pb.Map:
    """From nav_msgs/msg/OccupancyGrid."""
    info = msg.info
    p, q = info.origin.position, info.origin.orientation
    return pb.Map(
        version=version,
        width=info.width,
        height=info.height,
        resolution=info.resolution,
        origin_x=p.x,
        origin_y=p.y,
        origin_yaw=geometry.yaw_from_quaternion(q.x, q.y, q.z, q.w),
        cells_rle=geometry.rle_encode(msg.data),
    )


def map_content(msg) -> tuple:
    """What makes two maps the same map: size, resolution, origin and cells (not the stamp)."""
    info = msg.info
    p, q = info.origin.position, info.origin.orientation
    return (info.width, info.height, info.resolution, p.x, p.y, q.z, q.w, np.asarray(msg.data, dtype=np.int8).tobytes())


def robot_pose(stamp: float, x: float, y: float, heading: float) -> pb.RobotPose:
    return pb.RobotPose(stamp=stamp, x=x, y=y, heading=heading)


def route(stamp: float, points: List[tuple]) -> pb.Route:
    """Points as (x, y) in metres, map coordinates."""
    out = []
    for x, y in points:
        out += [int(round(x * 100.0)), int(round(y * 100.0))]
    return pb.Route(stamp=stamp, points_cm=out)


def no_go_zones(zones, map_name: str = "") -> pb.NoGoZones:
    """Zones as (name, [(x, y), ...]) in metres, map coordinates, of the saved map `map_name`
    ("" while no saved map is in use)."""
    out = pb.NoGoZones(map_name=map_name)
    for name, corners in zones:
        cm = []
        for x, y in corners:
            cm += [int(round(x * 100.0)), int(round(y * 100.0))]
        out.zones.add(name=name, points_cm=cm)
    return out


def goal_statuses(msg) -> List[tuple]:
    """From action_msgs/msg/GoalStatusArray: (goal id, accepted at, status) per goal."""
    return [
        (bytes(bytearray(s.goal_info.goal_id.uuid)), s.goal_info.stamp.sec + s.goal_info.stamp.nanosec * 1e-9, int(s.status))
        for s in msg.status_list
    ]


def safety_status(msg) -> pb.SafetyStatus:
    """From vacuum_interfaces/msg/SafetyStatus. Its stamp is left out: only the content is passed
    on, so that two copies saying the same thing compare equal."""
    return pb.SafetyStatus(
        source=msg.source,
        blocked_reason=msg.blocked_reason,
        cliff=msg.cliff,
        bumper=msg.bumper,
        off_floor=msg.off_floor,
        reflex_active=msg.reflex_active,
        stale=list(msg.stale),
    )


# ---------------------------------------------------------------- cleaning
_CLEANING_STATES = {
    "none": pb.CleaningStatus.STATE_NONE,
    "starting": pb.CleaningStatus.STATE_STARTING,
    "ending": pb.CleaningStatus.STATE_ENDING,
    "cleaning": pb.CleaningStatus.STATE_CLEANING,
    "paused": pb.CleaningStatus.STATE_PAUSED,
    "finished": pb.CleaningStatus.STATE_FINISHED,
    "stopped": pb.CleaningStatus.STATE_STOPPED,
    "failed": pb.CleaningStatus.STATE_FAILED,
}
_STATUS_NUMBERS = ("job_id", "target", "started", "coverage", "cleaned_area", "planned_area", "distance", "cleaning_time",
                   "time_left", "rows_done", "rows_total", "rows_skipped", "current_row", "edges_done", "edges_total",
                   "edges_skipped", "current_edge", "bumps", "pauses", "piece")
_ESTIMATE_FIELDS = ("rows", "edges", "turns", "transits", "distance", "planned_area", "left_out_area", "unreachable_area", "time")
_JOB_FIELDS = ("what", "area", "spot_x", "spot_y", "plan_name", "direction", "passes", "edges", "return_to_dock")


def cleaning_job(msg) -> pb.CleaningJob:
    """From vacuum_interfaces/msg/CleaningJob. The numbers of both enums are the same."""
    return pb.CleaningJob(what=msg.what, area=list(msg.area), spot_x=msg.spot_x, spot_y=msg.spot_y, plan_name=msg.plan_name,
                          direction=msg.direction, passes=msg.passes, edges=msg.edges, return_to_dock=msg.return_to_dock)


def cleaning_job_fields(job: pb.CleaningJob) -> dict:
    """The fields of vacuum_interfaces/msg/CleaningJob for a job from the phone."""
    return {
        "what": int(job.what),
        "area": [float(v) for v in job.area],
        "spot_x": float(job.spot_x),
        "spot_y": float(job.spot_y),
        "plan_name": job.plan_name,
        "direction": job.direction,
        "passes": int(job.passes),
        "edges": int(job.edges),
        "return_to_dock": int(job.return_to_dock),
    }


def cleaning_job_problem(job: pb.CleaningJob) -> Optional[str]:
    """Why the robot could not even read this job, or None. The robot checks the rest."""
    if job.what not in pb.CleaningJob.What.values():
        return f"unknown kind of clean {job.what}"
    if job.edges not in pb.CleaningJob.Option.values() or job.return_to_dock not in pb.CleaningJob.Option.values():
        return "unknown option value"
    if job.passes > 2:
        return "passes must be 1 or 2"
    if job.what == pb.CleaningJob.WHAT_AREA and (len(job.area) < 6 or len(job.area) % 2):
        return "an area needs at least 3 corners, each an x and a y"
    if not all(math.isfinite(v) for v in list(job.area) + [job.spot_x, job.spot_y]):
        return "a corner or the spot is not a number"
    return None


def cleaning_estimate(msg) -> pb.CleaningEstimate:
    """From vacuum_interfaces/msg/CleaningEstimate."""
    return pb.CleaningEstimate(**{k: getattr(msg, k) for k in _ESTIMATE_FIELDS})


def cleaning_status(msg) -> pb.CleaningStatus:
    """From vacuum_interfaces/msg/CleaningStatus. The stamp is left out: two copies saying the same
    thing compare equal, so unchanged copies are not sent again before their repeat is due."""
    out = pb.CleaningStatus(state=_CLEANING_STATES.get(msg.state, pb.CleaningStatus.STATE_NONE), message=msg.message,
                            job=cleaning_job(msg.job), **{k: getattr(msg, k) for k in _STATUS_NUMBERS})
    return out


def _cm(v: float) -> int:
    return int(round(v * 100.0))


def cleaning_plan(msg) -> pb.CleaningPlan:
    """From vacuum_interfaces/msg/CleaningPlan. Piece kinds have the same numbers at both ends."""
    out = pb.CleaningPlan(job_id=msg.job_id, target=msg.target, estimate=cleaning_estimate(msg.estimate))
    for p in msg.pieces:
        out.pieces.add(kind=p.kind, x0_cm=_cm(p.x0), y0_cm=_cm(p.y0), x1_cm=_cm(p.x1), y1_cm=_cm(p.y1), number=p.number,
                       pass_number=p.pass_number)
    return out


def cleaned_area(msg, job_id: int) -> pb.CleanedArea:
    """From the nav_msgs/msg/OccupancyGrid on /cleaning/cleaned."""
    info = msg.info
    return pb.CleanedArea(job_id=job_id, width=info.width, height=info.height, resolution=info.resolution,
                          origin_x=info.origin.position.x, origin_y=info.origin.position.y,
                          cells_rle=geometry.rle_encode(msg.data))


def cleaning_plan_info(msg) -> pb.CleaningPlanInfo:
    """From vacuum_interfaces/msg/CleaningPlanInfo."""
    return pb.CleaningPlanInfo(name=msg.name, job=cleaning_job(msg.job), estimate=cleaning_estimate(msg.estimate),
                               map_changed=msg.map_changed, saved=msg.saved)


def cleaning_record(msg, picture: bytes = b"") -> pb.CleaningRecord:
    """From vacuum_interfaces/msg/CleaningRecord, with its picture when one is asked for."""
    return pb.CleaningRecord(id=msg.id, started=msg.started, ended=msg.ended, status=cleaning_status(msg.status), picture_png=picture)


_EXPLORATION_STATES = {
    "none": pb.ExplorationStatus.STATE_NONE,
    "starting": pb.ExplorationStatus.STATE_STARTING,
    "exploring": pb.ExplorationStatus.STATE_EXPLORING,
    "returning": pb.ExplorationStatus.STATE_RETURNING,
    "ending": pb.ExplorationStatus.STATE_ENDING,
    "finished": pb.ExplorationStatus.STATE_FINISHED,
    "stopped": pb.ExplorationStatus.STATE_STOPPED,
    "failed": pb.ExplorationStatus.STATE_FAILED,
}
_EXPLORATION_NUMBERS = ("job_id", "started", "places_left", "mapped_area", "exploring_time", "time_limit", "distance", "goals_sent",
                        "goals_failed")


def exploration_status(msg) -> pb.ExplorationStatus:
    """From vacuum_interfaces/msg/ExplorationStatus. The stamp is left out, as for the cleaning status."""
    return pb.ExplorationStatus(state=_EXPLORATION_STATES.get(msg.state, pb.ExplorationStatus.STATE_NONE), message=msg.message,
                                **{k: getattr(msg, k) for k in _EXPLORATION_NUMBERS})


def frontiers(msg, job_id: int) -> pb.Frontiers:
    """From the geometry_msgs/msg/PoseArray on /exploration/frontiers (map frame)."""
    out = pb.Frontiers(job_id=job_id)
    for p in msg.poses:
        out.points_cm.extend((_cm(p.position.x), _cm(p.position.y)))
    return out


def cleaning_history(records, totals) -> pb.CleaningHistory:
    """From the answer of /cleaning/history."""
    return pb.CleaningHistory(
        records=[cleaning_record(r) for r in records],
        totals=pb.CleaningTotals(cleans=totals.cleans, area=totals.area, time=totals.time, distance=totals.distance),
    )

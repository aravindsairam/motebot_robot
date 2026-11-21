"""The cleaning node: takes a cleaning job, drives it through Nav2, records what was really cleaned,
reports progress, keeps saved plans and the record of every clean.

It owns cleaning jobs, nothing else. It does not command the wheels (Nav2 does, through the safety
gate) and it does not decide modes (the mode manager does; this node asks it):

    /cleaning/start        -> checks and plans the job, answers (with the reason if refused), then
                              reports it "starting" and asks for cleaning mode; the mode manager
                              allows cleaning mode only while a job is starting
    in cleaning mode       -> waits for Nav2's costmaps, cancels goals left from before, drives the
                              pieces one by one; cancels any goal someone else sends meanwhile
    the job ends           -> reports "ending" and asks for docking (finished, and docking wanted)
                              or idle; the job is over at that hand-over
    the mode leaves cleaning (Stop, Dock, failsafe, another mode) -> the job ends as stopped

The robot is in cleaning mode exactly while a job runs or is paused. The mode manager watches
/cleaning/status in cleaning mode: silent for 1 s, or reporting no job (as a node restarted in the
middle of a clean does), means failsafe. This node never starts a job by itself.

Interfaces (types in vacuum_interfaces):
    /cleaning/start         srv StartCleaning            start a job; answers before the mode change
    /cleaning/status        msg CleaningStatus           on change and five times a second; latched
    /cleaning/plan          msg CleaningPlan             the job's rows, or the last preview's; latched
    /cleaning/plan_markers  visualization_msgs/MarkerArray  the same, for RViz
    /cleaning/cleaned       nav_msgs/OccupancyGrid       cleaned cells (100), same size as the map; latched
    /cleaning/pause, /cleaning/resume                    std_srvs/Trigger
    /cleaning/preview       srv PreviewCleaning          plan without moving (with a plan name: load it)
    /cleaning/save_plan, /cleaning/list_plans, /cleaning/delete_plan
    /cleaning/history       srv ListCleaningHistory      records newest first, and the totals
    /cleaning/get_record    srv GetCleaningRecord        one record with its picture

Written in Python: it orchestrates (goals, files, a 10 Hz position reading), nothing time-critical.
Timers run on the computer's steady clock, so they keep running if the simulator freezes.
"""

import math
import os
import threading
import time
import traceback
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from vacuum_interfaces.msg import (CleaningEstimate, CleaningJob, CleaningPiece, CleaningPlan, CleaningPlanInfo,
                                   CleaningRecord, CleaningStatus, CleaningTotals, RobotState, SafetyStatus)
from vacuum_interfaces.srv import (ChangeState, DeleteCleaningPlan, GetCleaningRecord, ListCleaningHistory,
                                   ListCleaningPlans, PreviewCleaning, SaveCleaningPlan, StartCleaning)
from vacuum_mode_manager import map_files

from . import job as jobs
from . import mapdata, picture, planner
from .coverage import Recorder
from .grid import polygon_mask
from .jobspec import AREA, EVERYTHING, PLAN, SPOT, JobSpec
from .planner import CONNECTOR, EDGE, ROW, TRANSIT, Piece, Plan, PlanError, Settings
from .store import CleaningStore, StoreError

CLEANING_MODE = "cleaning"
CHANGE_MODE_TIMEOUT = 250.0  # s: the mode manager's own worst case, plus margin
BUSY_RETRY = 5.0  # s to keep asking while the mode manager answers "busy" (another change in progress)
NAV2_READY_TIMEOUT = 30.0  # s for Nav2's costmaps to start publishing after cleaning mode is entered
FOREIGN_GOAL_GRACE = 1.0  # s before an unknown goal counts as someone else's (ours may not be known yet)
# Modes a job can start from. Not mapping (leaving it loses an unsaved map), not failsafe.
_START_MODES = ("idle", "docked", "manual", "navigation", "docking")
_KINDS = {ROW: CleaningPiece.ROW, CONNECTOR: CleaningPiece.CONNECTOR, TRANSIT: CleaningPiece.TRANSIT, EDGE: CleaningPiece.EDGE}
_WHAT = {CleaningJob.EVERYTHING: EVERYTHING, CleaningJob.AREA: AREA, CleaningJob.SPOT: SPOT, CleaningJob.PLAN: PLAN}
_WHAT_BACK = {v: k for k, v in _WHAT.items()}
_ACTIVE_GOAL = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
_SETTINGS_FIELDS = [f for f in Settings.__dataclass_fields__]


def _file_signature(path: str) -> tuple:
    """What changes when a file is written or replaced (the zones file is replaced, by a rename)."""
    try:
        st = os.stat(path)
    except OSError:
        return ()  # missing
    return st.st_ino, st.st_mtime_ns, st.st_size


class _Job:
    """One cleaning job, from the request to its record."""

    def __init__(self, job_id: int, spec: JobSpec, msg: CleaningJob):
        self.id = job_id
        self.spec = spec
        self.msg = msg
        self.state = CleaningStatus.STARTING
        self.message = "starting"
        self.started = time.time()
        self.ended = 0.0
        self.map: Optional[mapdata.MapData] = None
        self.plan: Optional[Plan] = None
        self.runner: Optional[jobs.JobRunner] = None
        self.recorder: Optional[Recorder] = None
        self.outcome: Optional[str] = None  # how it ended (jobs.FINISHED, STOPPED, FAILED), once it has
        self.note = ""  # a passing remark for the status message ("cancelled a goal from someone else")
        self.note_until = 0.0
        self.asked_mode = False  # this job asked the mode manager for cleaning mode
        self.in_mode = False  # the robot entered cleaning mode for this job
        self.seen_in_mode = False  # /robot_state has shown cleaning mode since then
        self.reached_cleaning = False  # the robot started driving the plan
        self.stop_reason: Optional[str] = None
        self.cleaning_time = 0.0  # s of robot time (simulation time), paused time not counted
        self.turning_time = 0.0  # s of it turning on the spot
        self.bumps = 0
        self.sweeping = False  # a row, connector or edge is being driven (trips are not cleaning)


class CleaningNode(Node):
    def __init__(self):
        super().__init__("cleaning_node")
        for name, default in Settings().__dict__.items():
            self.declare_parameter(name, default)
        self.declare_parameter("maps_dir", "")  # the saved maps, and with each its no-go zones
        self.declare_parameter("edges_by_default", False)
        self.declare_parameter("row_controller", "CleanRow")
        self.declare_parameter("row_goal_checker", "row_goal_checker")
        self.declare_parameter("goal_tolerance", 0.10)
        self.declare_parameter("rejoin_distance", 0.30)
        self.declare_parameter("max_skips_in_a_row", 3)
        self.declare_parameter("piece_timeout_margin", 30.0)
        self.declare_parameter("transit_timeout", 180.0)
        self.declare_parameter("record_rate", 10.0)
        self.declare_parameter("status_rate", 5.0)
        self.declare_parameter("cleaned_rate", 1.0)
        self.declare_parameter("history_size", 50)

        self._maps_dir = self.param("maps_dir")
        if not self._maps_dir:
            raise RuntimeError("parameter maps_dir is required")
        self._settings = Settings(**{k: type(getattr(Settings(), k))(self.param(k)) for k in _SETTINGS_FIELDS})
        self._run_settings = jobs.RunSettings(
            goal_tolerance=float(self.param("goal_tolerance")),
            rejoin_distance=float(self.param("rejoin_distance")),
            max_skips_in_a_row=int(self.param("max_skips_in_a_row")),
        )

        self._lock = threading.RLock()
        self._job: Optional[_Job] = None  # the job running or paused, else None
        self._last: Optional[_Job] = None  # the most recent job, running or ended
        self._job_counter = 0
        self._robot_state: Optional[RobotState] = None
        self._map_info = None  # (width, height, resolution, origin x, origin y) of the last /map
        self._costmap_at = 0.0  # when the local costmap last published (monotonic)
        self._our_goals = set()  # navigate_to_pose goal ids this node sent
        self._strangers = {}  # other senders' active navigate_to_pose goal ids -> when first seen
        self._cancelled_strangers = set()
        self._reflex = threading.Event()  # the safety gate began backing the robot off a cliff or bump
        self._reflex_active = False
        self._bumper = False
        self._last_odom = None  # (x, y, yaw, monotonic time) of the last reading while cleaning
        self._last_tick = None
        self._cleaned_sent = 0.0
        self._preview: Optional[CleaningPlan] = None
        self._map_cache: Tuple[str, tuple, Optional[mapdata.MapData]] = ("", (), None)

        # --- ROS plumbing
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._client_group = ReentrantCallbackGroup()
        services = ReentrantCallbackGroup()
        data = MutuallyExclusiveCallbackGroup()

        self._status_pub = self.create_publisher(CleaningStatus, "cleaning/status", latched)
        self._plan_pub = self.create_publisher(CleaningPlan, "cleaning/plan", latched)
        self._markers_pub = self.create_publisher(MarkerArray, "cleaning/plan_markers", latched)
        self._cleaned_pub = self.create_publisher(OccupancyGrid, "cleaning/cleaned", latched)

        self.create_subscription(RobotState, "robot_state", self._on_robot_state, latched, callback_group=data)
        self.create_subscription(SafetyStatus, "safety_status", self._on_safety, 10, callback_group=data)
        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(GoalStatusArray, "navigate_to_pose/_action/status", self._on_nav_status, status_qos, callback_group=data)
        self.create_subscription(OccupancyGrid, "map", self._on_map, latched, callback_group=data)
        # Not latched: only a costmap published since cleaning mode was entered says Nav2 is up.
        self.create_subscription(OccupancyGrid, "local_costmap/costmap", self._on_costmap, 1, callback_group=data)
        self._cancel_nav = self.create_client(CancelGoal, "navigate_to_pose/_action/cancel_goal", callback_group=self._client_group)

        self._change_state = self.create_client(ChangeState, "change_state", callback_group=self._client_group)
        self._clear_costmap = self.create_client(ClearEntireCostmap, "local_costmap/clear_entirely_local_costmap", callback_group=self._client_group)
        self._follow = ActionClient(self, FollowPath, "follow_path", callback_group=self._client_group)
        self._navigate = ActionClient(self, NavigateToPose, "navigate_to_pose", callback_group=self._client_group)

        self.tf = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self.tf, self)

        self.create_service(StartCleaning, "cleaning/start", self._on_start, callback_group=services)
        self.create_service(Trigger, "cleaning/pause", self._on_pause, callback_group=services)
        self.create_service(Trigger, "cleaning/resume", self._on_resume, callback_group=services)
        self.create_service(PreviewCleaning, "cleaning/preview", self._on_preview, callback_group=services)
        self.create_service(SaveCleaningPlan, "cleaning/save_plan", self._on_save_plan, callback_group=services)
        self.create_service(ListCleaningPlans, "cleaning/list_plans", self._on_list_plans, callback_group=services)
        self.create_service(DeleteCleaningPlan, "cleaning/delete_plan", self._on_delete_plan, callback_group=services)
        self.create_service(ListCleaningHistory, "cleaning/history", self._on_history, callback_group=services)
        self.create_service(GetCleaningRecord, "cleaning/get_record", self._on_get_record, callback_group=services)

        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / float(self.param("status_rate")), self._publish_status, callback_group=MutuallyExclusiveCallbackGroup(), clock=steady)
        self.create_timer(1.0 / float(self.param("record_rate")), self._record_tick, callback_group=MutuallyExclusiveCallbackGroup(), clock=steady)
        self._publish_status()
        self._publish_plan(None)
        self.get_logger().info(f"cleaning node started; plans and records in {self._maps_dir}/<map>_cleaning")

    def param(self, name):
        return self.get_parameter(name).value

    # ================================================================== robot state in
    def _on_robot_state(self, msg: RobotState) -> None:
        self._robot_state = msg
        with self._lock:
            job = self._job
            if job is None:
                return
            if job.in_mode and msg.state == CLEANING_MODE:
                job.seen_in_mode = True
            # A copy from before the switch may still arrive after the mode manager said yes: only a
            # change seen after cleaning mode was seen ends the job. While it is ending, the change is ours.
            if job.seen_in_mode and msg.state != CLEANING_MODE and job.state != CleaningStatus.ENDING:
                self._stop_job(job, f"the robot left cleaning mode (now {msg.state})")

    def _on_map(self, msg: OccupancyGrid) -> None:
        i = msg.info
        self._map_info = (i.width, i.height, i.resolution, i.origin.position.x, i.origin.position.y)

    def _on_costmap(self, _msg) -> None:
        self._costmap_at = time.monotonic()

    def _on_safety(self, msg: SafetyStatus) -> None:
        if msg.reflex_active and not self._reflex_active:
            self._reflex.set()
        self._reflex_active = msg.reflex_active
        with self._lock:
            job = self._job
            if msg.bumper and not self._bumper and job is not None and job.state == CleaningStatus.CLEANING:
                job.bumps += 1
        self._bumper = msg.bumper

    def _on_nav_status(self, msg: GoalStatusArray) -> None:
        """Remember navigate_to_pose goals this node did not send that are under way, and since when.
        A goal this node has just sent may show up here before it is known as ours, hence the grace."""
        now = time.monotonic()
        active = {bytes(bytearray(s.goal_info.goal_id.uuid)) for s in msg.status_list if s.status in _ACTIVE_GOAL}
        with self._lock:
            self._strangers = {g: self._strangers.get(g, now) for g in active if g not in self._our_goals}
            self._cancelled_strangers &= active

    def _cancel_strangers(self, job: _Job, grace: float) -> int:
        """Cancel, by their ids, the navigate_to_pose goals someone else sent (RViz, a phone's Go to,
        a trip left from navigation mode). Never "all goals": this node's own are cancelled by handle."""
        now = time.monotonic()
        with self._lock:
            ids = [g for g, seen in self._strangers.items()
                   if g not in self._our_goals and g not in self._cancelled_strangers and now - seen >= grace]
            self._cancelled_strangers.update(ids)
        if not ids or not self._cancel_nav.service_is_ready():
            return 0
        for goal_id in ids:
            request = CancelGoal.Request()
            request.goal_info.goal_id.uuid = list(goal_id)
            self._cancel_nav.call_async(request)  # the answer does not matter: the goal's status shows it
        self.get_logger().warn(f"cancelled {len(ids)} navigation goal(s) sent by someone else during the clean")
        with self._lock:
            job.note = "cancelled a navigation goal sent by someone else"
            job.note_until = now + 10.0
        return len(ids)

    # ================================================================== the map
    def _map_name(self) -> str:
        state = self._robot_state
        if state is not None and state.current_map:
            return state.current_map
        marker = os.path.join(self._maps_dir, "current_map.txt")
        if os.path.isfile(marker):
            with open(marker) as f:
                return f.read().strip()
        return ""

    def _load_map(self) -> mapdata.MapData:
        name = self._map_name()
        if not name:
            raise PlanError("there is no saved map yet: make a map first")
        yaml_path = os.path.join(self._maps_dir, name + ".yaml")
        if not os.path.isfile(yaml_path):
            raise PlanError(f"the map '{name}' is missing ({yaml_path})")
        # The zones of the map planned on: each saved map has its own (none if it has no file).
        zones_file = map_files.zones_path(self._maps_dir, name)
        stamp = tuple(_file_signature(p) for p in (yaml_path, yaml_path[:-5] + ".pgm", zones_file))
        cached_name, cached_stamp, cached = self._map_cache
        if cached is not None and cached_name == name and cached_stamp == stamp:
            return cached
        m = mapdata.load(yaml_path, map_files.read_zones(zones_file))
        self._map_cache = (name, stamp, m)
        return m

    def _store(self, map_name: Optional[str] = None) -> CleaningStore:
        name = map_name or self._map_name()
        if not name:
            raise StoreError("there is no saved map yet: plans and records belong to a map")
        return CleaningStore(self._maps_dir, name, int(self.param("history_size")))

    def _report_store_problems(self, store: CleaningStore) -> None:
        for problem in store.problems:
            self.get_logger().warn(problem)
        store.problems.clear()

    # ================================================================== where the robot is
    def _lookup(self, parent: str) -> Optional[Tuple[float, float, float]]:
        try:
            t = self.tf.lookup_transform(parent, "base_link", Time())  # the newest: AMCL stamps ahead
        except TransformException:
            return None
        tr, q = t.transform.translation, t.transform.rotation
        return tr.x, tr.y, map_files.yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def map_pose(self) -> Optional[Tuple[float, float, float]]:
        return self._lookup("map")

    # ================================================================== planning
    def _spec_from_msg(self, msg: CleaningJob) -> JobSpec:
        what = _WHAT.get(msg.what, f"kind {msg.what}")
        area = [(float(msg.area[i]), float(msg.area[i + 1])) for i in range(0, len(msg.area) - 1, 2)]
        edges = None if msg.edges == CleaningJob.OPTION_DEFAULT else msg.edges == CleaningJob.OPTION_ON
        return JobSpec(
            what=what,
            area=area,
            spot=(float(msg.spot_x), float(msg.spot_y)) if what == SPOT else None,
            plan_name=msg.plan_name.strip() if what == PLAN else "",
            direction=(msg.direction.strip().lower() or "auto"),
            passes=int(msg.passes) or 1,
            edges=edges,
            return_to_dock=msg.return_to_dock != CleaningJob.OPTION_OFF,
        )

    @staticmethod
    def _spec_to_msg(spec: JobSpec) -> CleaningJob:
        msg = CleaningJob()
        msg.what = _WHAT_BACK.get(spec.what, CleaningJob.EVERYTHING)
        msg.area = [float(v) for p in spec.area for v in p]
        if spec.spot is not None:
            msg.spot_x, msg.spot_y = float(spec.spot[0]), float(spec.spot[1])
        msg.plan_name = spec.plan_name
        msg.direction = spec.direction
        msg.passes = spec.passes
        msg.edges = CleaningJob.OPTION_DEFAULT if spec.edges is None else (CleaningJob.OPTION_ON if spec.edges else CleaningJob.OPTION_OFF)
        msg.return_to_dock = CleaningJob.OPTION_ON if spec.return_to_dock else CleaningJob.OPTION_OFF
        return msg

    def _make_plan(self, spec: JobSpec, m: mapdata.MapData, robot: Tuple[float, float]) -> Tuple[JobSpec, Plan, str]:
        """(the job as run, its plan, a note). A saved plan's own rows are used while the map is the
        same; otherwise they are planned again from its area and options."""
        why = spec.check()
        if why:
            raise PlanError(why)
        note = ""
        if spec.what == PLAN:
            store = self._store(m.name)
            saved = store.load_plan(spec.plan_name)
            self._report_store_problems(store)
            if saved is None:
                raise PlanError(f"there is no saved plan named '{spec.plan_name}'")
            run = spec.with_saved(JobSpec.from_dict(saved.job))
            if saved.map_checksum == m.checksum:
                pieces = planner.restart_from(saved.pieces, robot)
                if not pieces:
                    raise PlanError(f"the saved plan '{spec.plan_name}' has no rows")
                return run, self._plan_from_pieces(m, pieces, run), ""
            spec, note = run, "the map has changed since this plan was saved: its rows were planned again"
        plan = planner.make_plan(m.grid, robot, self._settings, spec.options(bool(self.param("edges_by_default"))),
                                 spec.area_polygon(self._settings.spot_size))
        return spec, plan, note

    def _plan_from_pieces(self, m: mapdata.MapData, pieces: List[Piece], spec: JobSpec) -> Plan:
        """A saved plan's rows, as they were saved; its planned floor worked out again as a fresh plan's
        is (only floor inside its area, if it has one)."""
        started = time.perf_counter()
        planned = planner.cleaned_by(m.grid, pieces, self._settings.cleaning_width / 2.0) & m.grid.floor
        area = spec.area_polygon(self._settings.spot_size)
        if area:
            planned &= polygon_mask(m.grid, area)
        empty = np.zeros(m.grid.shape, dtype=bool)
        estimate = planner._estimate(pieces, pieces[0].start, self._settings, m.grid.resolution, planned, empty, empty)
        estimate.planning_time = time.perf_counter() - started
        return Plan(pieces=pieces, planned=planned, left_out=empty, unreachable=empty, estimate=estimate)

    def _robot_xy_or_dock(self) -> Tuple[Tuple[float, float], str]:
        """Where to plan from, for a preview or a saved plan. Refused while mapping: plans are made on
        the saved map the robot localizes on, never on one still being built."""
        state = self._robot_state
        if state is not None and state.making_map:
            raise PlanError("the robot is making a map: save or discard the map first")
        pose = self.map_pose()
        if pose is not None:
            return (pose[0], pose[1]), ""
        return (0.0, 0.0), " (the robot's place on the map is not known: planned from the dock)"

    # ================================================================== jobs
    def _new_job(self, spec: JobSpec, msg: CleaningJob) -> _Job:
        """Make the job current. Call with the lock held; does nothing slow."""
        self._job_counter += 1
        job = _Job(self._job_counter, spec, msg)
        self._job = job
        self._last = job
        return job

    def _check_map(self, m: mapdata.MapData) -> None:
        """A plan is made on the saved map the robot localizes on, never on one still being built."""
        if self._map_info is None:
            return
        w, h, res, ox, oy = self._map_info
        gh, gw = m.grid.shape
        same = (w, h) == (gw, gh) and abs(res - m.grid.resolution) < 1e-6 and \
            abs(ox - m.grid.origin[0]) < 1e-3 and abs(oy - m.grid.origin[1]) < 1e-3
        if not same:
            raise PlanError(f"the map the robot is using does not match the saved map '{m.name}': try again in a moment")

    def _on_start(self, request, response):
        """Check and plan a job; answer; then run it on its own thread."""
        try:
            with self._lock:
                busy = self._job is not None
            state = self._robot_state
            if busy:
                raise PlanError("the robot is already cleaning (or paused, or finishing a clean): stop that clean first")
            if state is None:
                raise PlanError("the robot's mode is not known yet (is the mode manager running?)")
            if state.making_map:
                raise PlanError("the robot is making a map: save or discard the map first")
            if state.state not in _START_MODES:
                raise PlanError(f"a clean cannot start in {state.state} mode")
            spec = self._spec_from_msg(request.job)
            why = spec.check()
            if why:
                raise PlanError(why)
            m = self._load_map()
            self._check_map(m)
            pose = self.map_pose()
            if pose is None:
                raise PlanError("the robot does not know where it is on the map")
            spec, plan, note = self._make_plan(spec, m, (pose[0], pose[1]))
            with self._lock:
                if self._job is not None:
                    raise PlanError("the robot is already cleaning: stop that clean first")
                job = self._new_job(spec, request.job)
                job.map, job.plan = m, plan
                job.message = "starting"
        except (PlanError, StoreError) as exc:
            response.accepted, response.message = False, str(exc)
            self.get_logger().info(f"clean refused: {exc}")
            return response
        except Exception as exc:  # whatever happens, never leave a job half made
            self.get_logger().error(f"starting a clean failed in the cleaning node: {exc!r}\n{traceback.format_exc()}")
            response.accepted, response.message = False, f"the cleaning node failed while starting the clean: {exc}"
            return response
        response.accepted, response.job_id = True, job.id
        response.estimate = _estimate_msg(plan.estimate)
        response.message = note or (f"{plan.estimate.rows} rows, {plan.estimate.distance:.0f} m, "
                                    f"about {plan.estimate.time / 60:.0f} min")
        self.get_logger().info(f"job {job.id} ({spec.target}) accepted: {response.message}")
        self._publish_status()
        threading.Thread(target=self._run_job, args=(job,), daemon=True).start()
        return response

    def _stop_job(self, job: _Job, reason: str) -> None:
        with self._lock:
            if job.stop_reason is None:
                job.stop_reason = reason
            runner = job.runner
        if runner is not None:
            runner.stop(reason)

    def _run_job(self, job: _Job) -> None:
        """Everything a job goes through, from its start to its record and the next mode."""
        log = self.get_logger()
        try:
            outcome, reason = self._drive_job(job)
        except Exception as exc:  # a fault here must still end the job and leave cleaning mode
            log.error(f"cleaning job {job.id} failed in the cleaning node: {exc!r}\n{traceback.format_exc()}")
            outcome, reason = jobs.FAILED, f"the cleaning node failed: {exc}"
        try:
            self._end_job(job, outcome, reason)
        except Exception as exc:
            log.error(f"ending cleaning job {job.id} failed: {exc!r}\n{traceback.format_exc()}")
            with self._lock:
                job.state, job.message = CleaningStatus.FAILED, f"failed: {reason}"
                if self._job is job:
                    self._job = None

    def _stopped(self, job: _Job) -> Optional[str]:
        return job.stop_reason

    def _drive_job(self, job: _Job) -> Tuple[str, str]:
        log = self.get_logger()
        self._publish_plan(job)
        self._publish_cleaned_empty(job.map)
        self._set_state(job, CleaningStatus.STARTING, "switching to cleaning mode")
        state = self._robot_state
        if state is None or state.state != CLEANING_MODE:
            job.asked_mode = True
            ok, message = self._ask_mode(CLEANING_MODE)
            if not ok:
                return jobs.FAILED, f"the robot did not switch to cleaning mode: {message}"
        with self._lock:
            job.in_mode = True
        if not self._wait_for_mode(job):
            stopped = self._stopped(job)
            return (jobs.STOPPED, stopped) if stopped else (jobs.FAILED, "the robot did not stay in cleaning mode")
        self._set_state(job, CleaningStatus.STARTING, "waiting for navigation to start")
        if not self._wait_for_nav2(job):
            stopped = self._stopped(job)
            return (jobs.STOPPED, stopped) if stopped else (jobs.FAILED, "navigation did not start in cleaning mode")
        # A trip left over from navigation mode (or anyone's goal) would fight the first row.
        self._cancel_strangers(job, grace=0.0)
        log.info(f"job {job.id} ({job.spec.target}): {job.plan.estimate.rows} rows, {job.plan.estimate.distance:.0f} m, "
                 f"about {job.plan.estimate.time / 60:.0f} min")
        driver = _RosDriver(self, job)
        with self._lock:
            job.recorder = Recorder(job.map.grid, job.plan.planned, self._settings.cleaning_width / 2.0)
            job.runner = jobs.JobRunner(job.plan.pieces, driver, self._run_settings, log=self.get_logger().warn)
            if job.stop_reason is not None:
                job.runner.stop(job.stop_reason)
            self._last_odom = None
            self._last_tick = None
            job.reached_cleaning = True
            if not job.runner.paused:  # a pause may already have come in
                job.state = CleaningStatus.CLEANING
        self._publish_status()
        return job.runner.run()

    def _wait_for_mode(self, job: _Job, timeout: float = 30.0) -> bool:
        """After the mode manager said yes: wait until /robot_state shows cleaning mode, settled."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self._stopped(job):
                return False
            s = self._robot_state
            if job.seen_in_mode and s is not None and s.state == CLEANING_MODE and not s.transitioning:
                return True
            time.sleep(0.05)
        return False

    def _wait_for_nav2(self, job: _Job) -> bool:
        """Nav2 has just been switched on: wait until its local costmap publishes and its two actions
        answer, so the first trip is not refused for want of a costmap."""
        since = time.monotonic()
        end = since + NAV2_READY_TIMEOUT
        while time.monotonic() < end:
            if self._stopped(job):
                return False
            if self._costmap_at > since and self._follow.server_is_ready() and self._navigate.server_is_ready():
                time.sleep(1.0)  # one more costmap update: the obstacle layer fills from the next scans
                return not self._stopped(job)
            time.sleep(0.1)
        return False

    def _ask_mode(self, mode: str) -> Tuple[bool, str]:
        """Ask the mode manager for a mode; returns (done, its message). It answers once the change
        has finished, which can take minutes in bad cases. "Busy" (another change in progress) is
        asked again for a few seconds; a reply lost on a brand-new client (Humble) is asked again once."""
        if not self._change_state.wait_for_service(timeout_sec=5.0):
            return False, "the mode manager is not running (no /change_state service)"
        busy_until = time.monotonic() + BUSY_RETRY
        lost = 0
        while True:
            done = threading.Event()
            future = self._change_state.call_async(ChangeState.Request(new_state=mode))
            future.add_done_callback(lambda _f: done.set())
            if not done.wait(CHANGE_MODE_TIMEOUT if lost else 10.0):
                s = self._robot_state
                if s is not None and s.transitioning and not lost:
                    # The manager is working on it; its answer is still to come.
                    if done.wait(CHANGE_MODE_TIMEOUT):
                        response = future.result()
                        return bool(response.success), response.message
                lost += 1
                if lost >= 2:
                    return False, "no answer from the mode manager"
                continue
            response = future.result()
            if not response.success and response.message.startswith("busy") and time.monotonic() < busy_until:
                time.sleep(0.5)
                continue
            return bool(response.success), response.message

    def _end_job(self, job: _Job, outcome: str, reason: str) -> None:
        """Report the job as ending, hand over to the next mode, then report how it ended and keep
        its record. While it is ending the mode manager still sees a job, so cleaning mode is not a fault."""
        log = self.get_logger()
        state = self._robot_state
        with self._lock:
            job.ended = time.time()
            job.outcome = outcome
            leave = (job.in_mode or job.asked_mode) and state is not None and state.state == CLEANING_MODE
            if leave:
                job.state = CleaningStatus.ENDING
                docking = outcome == jobs.FINISHED and job.spec.return_to_dock
                job.message = "ending: going back to the dock" if docking else "ending"
        self._publish_status()
        if leave:
            next_mode = "docking" if outcome == jobs.FINISHED and job.spec.return_to_dock else "idle"
            ok, message = self._ask_mode(next_mode)
            if not ok and next_mode == "docking":
                log.warn(f"could not dock after cleaning ({message}); going to idle")
                ok, message = self._ask_mode("idle")
            if not ok:
                # The mode manager will see a robot in cleaning mode with no job: failsafe.
                log.error(f"could not leave cleaning mode: {message}")
        with self._lock:
            rec = job.recorder
            if outcome == jobs.FINISHED:
                job.state = CleaningStatus.FINISHED
                took = f"{job.cleaning_time / 60:.0f} min" if job.cleaning_time >= 90 else f"{job.cleaning_time:.0f} s"
                job.message = (f"cleaned {rec.cleaned_area:.1f} m² ({rec.coverage * 100:.0f}%) in {took}"
                               if rec is not None else "finished")
            elif outcome == jobs.STOPPED:
                job.state, job.message = CleaningStatus.STOPPED, f"stopped: {reason}"
            else:
                job.state, job.message = CleaningStatus.FAILED, f"failed: {reason}"
            if self._job is job:
                self._job = None
        # Two calls: rclpy refuses to log one line of code at two different levels.
        if job.state == CleaningStatus.FAILED:
            log.warn(f"job {job.id}: {job.message}")
        else:
            log.info(f"job {job.id}: {job.message}")
        if job.reached_cleaning:
            self._write_record(job)
        self._publish_cleaned(job, force=True)
        self._publish_plan(None)  # the rows disappear with the job
        self._publish_status()

    def _write_record(self, job: _Job) -> None:
        try:
            status = self._status(job)
            numbers = {k: getattr(status, k) for k in _STATUS_NUMBERS}
            numbers = {k: (float(v) if isinstance(v, float) else int(v)) for k, v in numbers.items()}
            record = {
                "job_id": job.id,
                "started": job.started,
                "ended": job.ended,
                "target": job.spec.target,
                "outcome": job.state,
                "reason": job.message,
                "job": job.spec.to_dict(),
                "status": numbers,
            }
            png = picture.record_picture(job.map, job.recorder.cleaned, job.recorder.path)
            store = self._store(job.map.name)
            rid = store.add_record(record, png)
            self._report_store_problems(store)
            self.get_logger().info(f"recorded clean {rid}")
        except Exception as exc:
            self.get_logger().error(f"could not write the record of job {job.id}: {exc!r}")

    # ================================================================== pause, resume
    def _on_pause(self, _request, response):
        with self._lock:
            job = self._job
            runner = job.runner if job is not None else None
        if job is None:
            response.success, response.message = False, "no cleaning job is running"
        elif runner is None:
            response.success, response.message = False, "the job is still starting; try again in a moment, or stop it"
        elif runner.paused:
            response.success, response.message = True, "already paused"
        elif runner.pause():
            with self._lock:
                job.recorder.break_stretch()
            self._set_state(job, CleaningStatus.PAUSED, "paused at " + self._where(job))
            response.success, response.message = True, "paused"
        else:
            response.success, response.message = False, "the job is ending"
        return response

    def _on_resume(self, _request, response):
        with self._lock:
            job = self._job
            runner = job.runner if job is not None else None
        if job is None or runner is None:
            response.success, response.message = False, "no cleaning job is paused"
        elif not runner.paused:
            response.success, response.message = job.state == CleaningStatus.CLEANING, "not paused"
        elif runner.resume():
            self._set_state(job, CleaningStatus.CLEANING, self._progress_text(job))
            response.success, response.message = True, "cleaning again"
        else:
            response.success, response.message = False, "the job is ending"
        return response

    # ================================================================== preview and saved plans
    def _on_preview(self, request, response):
        try:
            robot, where = self._robot_xy_or_dock()
            m = self._load_map()
            self._check_map(m)
            spec, plan, note = self._make_plan(self._spec_from_msg(request.job), m, robot)
        except (PlanError, StoreError) as exc:
            response.success, response.message = False, str(exc)
            return response
        response.success = True
        response.replanned = bool(note)
        response.message = (note or f"{plan.estimate.rows} rows, about {plan.estimate.time / 60:.0f} min") + where
        response.estimate = _estimate_msg(plan.estimate)
        msg = self._plan_msg(0, spec.target, plan)
        with self._lock:
            self._preview = msg
            running = self._job is not None
        if not running:
            self._plan_pub.publish(msg)
            self._markers_pub.publish(_markers(plan.pieces))
        return response

    def _on_save_plan(self, request, response):
        try:
            spec = self._spec_from_msg(request.job)
            if spec.what == PLAN:
                raise PlanError("a saved plan cannot be saved again under another name; choose what to clean instead")
            store = self._store()
            name = store.check_name(request.name)
            if store.load_plan(name) is not None and not request.replace:
                response.success, response.name_taken = False, True
                response.message = f"a plan named '{name}' already exists; save with replace to overwrite it"
                return response
            robot, _ = self._robot_xy_or_dock()
            m = self._load_map()
            self._check_map(m)
            spec, plan, _ = self._make_plan(spec, m, robot)
            estimate = _estimate_msg(plan.estimate)
            store.save_plan(name, spec.to_dict(), plan.pieces, _estimate_dict(estimate), m.checksum, replace=request.replace)
            self._report_store_problems(store)
        except StoreError as exc:
            response.success, response.name_taken, response.message = False, exc.name_taken, str(exc)
            return response
        except PlanError as exc:
            response.success, response.message = False, str(exc)
            return response
        response.success, response.estimate = True, estimate
        response.message = f"saved '{name}': {plan.estimate.rows} rows, about {plan.estimate.time / 60:.0f} min"
        self.get_logger().info(response.message)
        return response

    def _on_list_plans(self, _request, response):
        try:
            store = self._store()
            checksum = self._load_map().checksum
        except (StoreError, PlanError) as exc:
            self.get_logger().warn(f"cannot list plans: {exc}")
            return response
        for p in store.list_plans():
            info = CleaningPlanInfo()
            info.name, info.map_name, info.saved = p.name, p.map_name, float(p.saved)
            info.job = self._spec_to_msg(JobSpec.from_dict(p.job))
            info.estimate = _estimate_from_dict(p.estimate)
            info.map_changed = p.map_checksum != checksum
            response.plans.append(info)
        self._report_store_problems(store)
        return response

    def _on_delete_plan(self, request, response):
        try:
            store = self._store()
        except StoreError as exc:
            response.success, response.message = False, str(exc)
            return response
        if store.delete_plan(request.name):
            response.success, response.message = True, f"deleted '{request.name.strip()}'"
        else:
            response.success, response.message = False, f"there is no saved plan named '{request.name.strip()}'"
        return response

    # ================================================================== history
    def _on_history(self, request, response):
        try:
            store = self._store()
        except StoreError as exc:
            self.get_logger().warn(f"cannot list the history: {exc}")
            return response
        for r in store.history(request.max_records or None):
            response.records.append(_record_msg(r))
        t = store.totals()
        response.totals = CleaningTotals(cleans=int(t["cleans"]), area=float(t["area"]), time=float(t["time"]), distance=float(t["distance"]))
        self._report_store_problems(store)
        return response

    def _on_get_record(self, request, response):
        try:
            store = self._store()
        except StoreError as exc:
            response.success, response.message = False, str(exc)
            return response
        found = store.get_record(request.id.strip())
        self._report_store_problems(store)
        if found is None:
            response.success, response.message = False, f"there is no record '{request.id.strip()}'"
            return response
        record, png = found
        response.success, response.message = True, ""
        response.record = _record_msg(record)
        response.picture = png
        return response

    # ================================================================== status out
    def _set_state(self, job: _Job, state: str, message: str) -> None:
        with self._lock:
            job.state, job.message = state, message
        self._publish_status()

    def _where(self, job: _Job) -> str:
        runner = job.runner
        if runner is None:
            return "the start"
        piece = runner.current_piece
        if piece is not None and piece.kind == EDGE:
            return f"edge {runner.current_edge} of {runner.edges_total}"
        if runner.current_row:
            return f"row {runner.current_row} of {runner.rows_total}"
        if runner.edges_total:
            return f"edge {runner.current_edge} of {runner.edges_total}"
        return "the end"

    def _progress_text(self, job: _Job) -> str:
        runner = job.runner
        if runner is None:
            return job.message
        piece = runner.current_piece
        where = self._where(job)
        if piece is not None and piece.kind == TRANSIT:
            return f"on the way to {where}"
        return where

    def _status(self, job: Optional[_Job]) -> CleaningStatus:
        msg = CleaningStatus()
        msg.stamp = self.get_clock().now().to_msg()
        if job is None:
            msg.state, msg.message = CleaningStatus.NONE, ""
            return msg
        with self._lock:
            msg.state = job.state
            if job.state == CleaningStatus.CLEANING:
                job.message = self._progress_text(job)
            msg.message = job.message
            if job.note and time.monotonic() < job.note_until:
                msg.message += f" ({job.note})"
            msg.job_id = job.id
            msg.target = job.spec.target
            msg.job = self._spec_to_msg(job.spec)
            msg.started = float(job.started)
            msg.cleaning_time = float(job.cleaning_time)
            msg.bumps = job.bumps
            rec, runner, plan = job.recorder, job.runner, job.plan
            if plan is not None:
                msg.planned_area = float(plan.estimate.planned_area)
            if rec is not None:
                msg.coverage = float(rec.coverage * 100.0)
                msg.cleaned_area = float(rec.cleaned_area)
                msg.distance = float(rec.distance)
            if runner is not None:
                msg.rows_done, msg.rows_total, msg.rows_skipped = runner.rows_done, runner.rows_total, runner.rows_skipped
                msg.edges_done, msg.edges_total, msg.edges_skipped = runner.edges_done, runner.edges_total, runner.edges_skipped
                msg.pauses = runner.pauses
                if job.state in (CleaningStatus.CLEANING, CleaningStatus.PAUSED):
                    msg.current_row, msg.current_edge = runner.current_row, runner.current_edge
                    msg.piece = runner.index
                    left = runner.remaining()
                    distance, turns = planner.remaining_estimate(left, self._settings)
                    msg.time_left = float(jobs.time_left(distance, turns, rec.distance if rec else 0.0, job.cleaning_time,
                                                         job.turning_time, runner.turns_done, self._settings))
            elif plan is not None:
                msg.rows_total, msg.edges_total = plan.estimate.rows, plan.estimate.edges
                msg.time_left = float(plan.estimate.time)
        return msg

    def _publish_status(self) -> None:
        with self._lock:
            job = self._job or self._last
        self._status_pub.publish(self._status(job))

    def _plan_msg(self, job_id: int, target: str, plan: Optional[Plan]) -> CleaningPlan:
        msg = CleaningPlan()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.job_id = job_id
        msg.target = target
        if plan is not None:
            for p in plan.pieces:
                msg.pieces.append(CleaningPiece(kind=_KINDS[p.kind], x0=float(p.start[0]), y0=float(p.start[1]), x1=float(p.end[0]),
                                                y1=float(p.end[1]), number=p.number, pass_number=p.pass_number, region=p.region))
            msg.estimate = _estimate_msg(plan.estimate)
        return msg

    def _publish_plan(self, job: Optional[_Job]) -> None:
        if job is None or job.plan is None:
            self._plan_pub.publish(self._plan_msg(0, "", None))
            self._markers_pub.publish(_markers([]))
            return
        self._plan_pub.publish(self._plan_msg(job.id, job.spec.target, job.plan))
        self._markers_pub.publish(_markers(job.plan.pieces))

    def _cleaned_msg(self, m: mapdata.MapData, cells: np.ndarray) -> OccupancyGrid:
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = float(m.grid.resolution)
        grid.info.height, grid.info.width = int(cells.shape[0]), int(cells.shape[1])
        grid.info.origin.position.x, grid.info.origin.position.y = float(m.grid.origin[0]), float(m.grid.origin[1])
        grid.info.origin.orientation.w = 1.0
        grid.data = cells.ravel().tolist()
        return grid

    def _publish_cleaned(self, job: _Job, force: bool = False) -> None:
        now = time.monotonic()
        if job.recorder is None or job.map is None:
            return
        if not force and now - self._cleaned_sent < 1.0 / float(self.param("cleaned_rate")):
            return
        self._cleaned_sent = now
        self._cleaned_pub.publish(self._cleaned_msg(job.map, job.recorder.occupancy()))

    def _publish_cleaned_empty(self, m: mapdata.MapData) -> None:
        """A new job starts with nothing cleaned."""
        self._cleaned_pub.publish(self._cleaned_msg(m, np.zeros(m.grid.shape, dtype=np.int8)))

    # ================================================================== ten times a second
    def _record_tick(self) -> None:
        # The timer runs on the steady clock (it must keep running if the simulator freezes), but
        # durations are measured on the robot's clock: simulation time, which runs at about 0.8 of
        # real time, and stops when the simulator does.
        now = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            job = self._job
        last, self._last_tick = self._last_tick, now
        if job is None or job.recorder is None:
            return
        if job.state in (CleaningStatus.CLEANING, CleaningStatus.PAUSED):
            self._cancel_strangers(job, FOREIGN_GOAL_GRACE)
        if job.state != CleaningStatus.CLEANING:
            self._last_odom = None
            return
        dt = 0.0 if last is None else min(max(now - last, 0.0), 0.5)
        here = self.map_pose()
        odom = self._lookup("odom")
        if here is None or odom is None:
            return
        with self._lock:
            job.cleaning_time += dt
            job.recorder.add((here[0], here[1]), (odom[0], odom[1]), sweeping=job.sweeping)
            prev = self._last_odom
            if prev is not None and dt > 0.0:
                turn_rate = abs(math.atan2(math.sin(odom[2] - prev[2]), math.cos(odom[2] - prev[2]))) / dt
                speed = math.dist(odom[:2], prev[:2]) / dt
                if turn_rate > 0.3 and speed < 0.03:  # turning on the spot
                    job.turning_time += dt
            self._last_odom = odom
        self._publish_cleaned(job)


class _RosDriver:
    """The job runner's way to Nav2: straight pieces to the controller server, transits to the
    navigator. Called from the job's thread; the ROS callbacks run on the executor's threads."""

    def __init__(self, node: CleaningNode, job: _Job):
        self.node = node
        self.job = job

    def pose(self):
        return self.node.map_pose()

    def ready(self) -> bool:
        s = self.node._robot_state
        return s is not None and s.state == CLEANING_MODE and not s.transitioning

    def clear_local_costmap(self) -> None:
        client = self.node._clear_costmap
        if not client.wait_for_service(timeout_sec=1.0):
            return
        done = threading.Event()
        client.call_async(ClearEntireCostmap.Request()).add_done_callback(lambda _f: done.set())
        done.wait(3.0)

    def follow_line(self, start, end, abort: threading.Event) -> str:
        goal = FollowPath.Goal()
        goal.path = _straight_path(start, end)
        goal.controller_id = self.node.param("row_controller")
        goal.goal_checker_id = self.node.param("row_goal_checker")
        length = math.dist(start, end)
        timeout = length / (self.node._settings.cleaning_speed / 2.0) + float(self.node.param("piece_timeout_margin"))
        self.job.sweeping = True  # only the controller server's straight pieces clean
        try:
            return self._run(self.node._follow, goal, abort, timeout, watch_reflex=True)
        finally:
            self.job.sweeping = False

    def navigate(self, x: float, y: float, yaw: float, abort: threading.Event) -> str:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"  # stamp zero: Nav2 uses the newest transform
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        return self._run(self.node._navigate, goal, abort, float(self.node.param("transit_timeout")), watch_reflex=False)

    def _run(self, client: ActionClient, goal, abort: threading.Event, timeout: float, watch_reflex: bool) -> str:
        log = self.node.get_logger()
        if abort.is_set():
            return jobs.CANCELLED
        if not client.wait_for_server(timeout_sec=3.0):
            log.warn(f"{client._action_name} is not available")
            return jobs.FAILED
        answered = threading.Event()
        sent = client.send_goal_async(goal)
        sent.add_done_callback(lambda _f: answered.set())
        if not answered.wait(10.0):
            log.warn(f"{client._action_name} did not answer a goal")
            return jobs.FAILED
        handle = sent.result()
        if handle is None or not handle.accepted:
            return jobs.FAILED
        if client is self.node._navigate:
            with self.node._lock:
                self.node._our_goals.add(bytes(bytearray(handle.goal_id.uuid)))
        finished = threading.Event()
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda _f: finished.set())
        if watch_reflex:
            self.node._reflex.clear()
        end = time.monotonic() + timeout
        reason = None
        while not finished.wait(0.05):
            if abort.is_set():
                reason = jobs.CANCELLED
            elif watch_reflex and self.node._reflex.is_set():
                log.warn("the safety gate backed the robot off a bump or a cliff during a row: counted as a failed piece")
                reason = jobs.FAILED
            elif time.monotonic() > end:
                log.warn(f"a piece took longer than {timeout:.0f} s: given up")
                reason = jobs.FAILED
            if reason is not None:
                cancelled = threading.Event()
                handle.cancel_goal_async().add_done_callback(lambda _f: cancelled.set())
                cancelled.wait(2.0)
                finished.wait(2.0)
                return reason
        wrapped = result_future.result()
        return jobs.SUCCEEDED if wrapped is not None and wrapped.status == GoalStatus.STATUS_SUCCEEDED else jobs.FAILED


# ---------------------------------------------------------------------- message helpers
_STATUS_NUMBERS = ("coverage", "cleaned_area", "planned_area", "distance", "cleaning_time", "time_left", "rows_done", "rows_total",
                   "rows_skipped", "current_row", "edges_done", "edges_total", "edges_skipped", "current_edge", "bumps", "pauses")
_ESTIMATE_FIELDS = ("rows", "edges", "turns", "transits", "distance", "planned_area", "left_out_area", "unreachable_area", "time",
                    "planning_time")


def _estimate_msg(e: planner.Estimate) -> CleaningEstimate:
    return CleaningEstimate(rows=e.rows, edges=e.edges, turns=e.turns, transits=e.transits, distance=float(e.distance),
                            planned_area=float(e.planned_area), left_out_area=float(e.left_out_area),
                            unreachable_area=float(e.unreachable_area), time=float(e.time), planning_time=float(e.planning_time))


def _estimate_dict(msg: CleaningEstimate) -> dict:
    return {k: (float(getattr(msg, k)) if isinstance(getattr(msg, k), float) else int(getattr(msg, k))) for k in _ESTIMATE_FIELDS}


def _estimate_from_dict(d: dict) -> CleaningEstimate:
    msg = CleaningEstimate()
    for k in _ESTIMATE_FIELDS:
        if k in d:
            setattr(msg, k, type(getattr(msg, k))(d[k]))
    return msg


def _record_msg(r: dict) -> CleaningRecord:
    msg = CleaningRecord()
    msg.id = str(r.get("id", ""))
    msg.started = float(r.get("started", 0.0))
    msg.ended = float(r.get("ended", 0.0))
    s = msg.status
    s.state = str(r.get("outcome", ""))
    s.message = str(r.get("reason", ""))
    s.job_id = int(r.get("job_id", 0))
    s.target = str(r.get("target", ""))
    s.started = msg.started
    try:
        s.job = CleaningNode._spec_to_msg(JobSpec.from_dict(r.get("job") or {}))
    except Exception:
        pass
    for k, v in (r.get("status") or {}).items():
        if k in _STATUS_NUMBERS:
            setattr(s, k, type(getattr(s, k))(v))
    return msg


def _straight_path(start, end, spacing: float = 0.05) -> Path:
    """A straight line as a path the controller can follow: a pose every 5 cm, all facing along it."""
    path = Path()
    path.header.frame_id = "map"
    yaw = math.atan2(end[1] - start[1], end[0] - start[0])
    n = max(1, int(math.ceil(math.dist(start, end) / spacing)))
    for i in range(n + 1):
        t = i / n
        p = PoseStamped()
        p.header.frame_id = "map"
        p.pose.position.x = start[0] + (end[0] - start[0]) * t
        p.pose.position.y = start[1] + (end[1] - start[1]) * t
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        path.poses.append(p)
    return path


def _markers(pieces) -> MarkerArray:
    """Rows, connectors and edges as lines for RViz (transits are Nav2's to plan, so not drawn)."""
    out = MarkerArray()
    clear = Marker()
    clear.action = Marker.DELETEALL
    out.markers.append(clear)
    colours = {ROW: (0.1, 0.35, 0.8), CONNECTOR: (0.45, 0.6, 0.9), EDGE: (0.1, 0.55, 0.25)}
    for i, kind in enumerate((ROW, CONNECTOR, EDGE)):
        m = Marker()
        m.header.frame_id = "map"
        m.ns, m.id = "cleaning_plan", i
        m.type, m.action = Marker.LINE_LIST, Marker.ADD
        m.scale.x = 0.02 if kind != CONNECTOR else 0.01
        m.color.r, m.color.g, m.color.b = colours[kind]
        m.color.a = 0.9
        m.pose.orientation.w = 1.0
        for p in pieces:
            if p.kind == kind:
                for x, y in (p.start, p.end):
                    pt = PoseStamped().pose.position
                    pt.x, pt.y, pt.z = float(x), float(y), 0.02
                    m.points.append(pt)
        if m.points:
            out.markers.append(m)
    return out


def main():
    rclpy.init()
    node = CleaningNode()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

"""The mode manager: the one node that decides what the robot is doing.

It owns two things:
  * the robot's mode (idle, mapping, navigation, manual, cleaning, exploring, docking, docked,
    failsafe), changed only through the /change_state service, by a docking finishing, or by a
    detected fault;
  * which groups of lifecycle nodes run in each mode, so that slam_toolbox and AMCL are never
    both active (each publishes the map -> odom transform).

It does not command the wheels. The safety gate (vacuum_safety) does, and it reads the mode
published here to decide who may drive: nobody in idle, in failsafe, and during a mode change.

Interfaces:
  /change_state     vacuum_interfaces/srv/ChangeState    request a mode; returns when done
  /save_map         vacuum_interfaces/srv/SaveMap        save the map being built (mapping mode only)
  /set_no_go_zones  vacuum_interfaces/srv/SetNoGoZones   replace the no-go zones of the map in use
  /robot_state      vacuum_interfaces/msg/RobotState     current mode, reason, faults (latched, and twice a second)
  /mode_label       visualization_msgs/msg/MarkerArray   the mode as text above the robot, for RViz

Failsafe is entered when a managed node stops answering or is in the wrong lifecycle state, when
lidar, odometry, cliff or bumper data stops, when the safety gate stops reporting, when the cleaning
node stops reporting in cleaning mode (or the exploration node in exploring mode), when the robot
is off the floor, when a mode change fails, or on request (an emergency stop). It is left by
requesting idle, once every fault has cleared. When a map was being made and is still there, it
can also be left for mapping, with the map kept (see state_machine.resume_mapping_problem).

Cleaning mode runs what navigation runs. It is entered only while the cleaning node
(vacuum_coverage) reports a job that is starting, so the robot is in it exactly while a job runs;
the node asks for it, and for the next mode when the job ends. In cleaning mode this node goes to
failsafe if the cleaning node falls silent or reports no job (a cleaning node restarted mid-clean
has none). Leaving cleaning mode cancels every Nav2 goal first, so no row drives on in the next mode.

Exploring mode is mapping with Nav2 running: slam_toolbox keeps making the map while the explorer,
owned by the exploration node (vacuum_exploration), sends Nav2 its goals. It is entered only from
mapping and only while the exploration node reports a job that is starting, and it ends in mapping,
so the map is kept. A request for idle (Stop) in exploring mode is carried out as a change to
mapping for the same reason. Nav2 is started there without a saved map: with an empty keepout mask,
and with a behaviour tree that never spins or backs the robot up. The exploration node is watched
like the cleaning node, and leaving the mode cancels every Nav2 goal.

No-go zones belong to a saved map (<maps_dir>/<map>_zones.yaml, see map_files). Each time Nav2 is
started, the keepout mask it keeps out of is made from the zones of the map in use. When the zones
are changed (/set_no_go_zones) while Nav2 runs, the mask server is told to load the new mask at
once, and the costmaps take it on their next update: no mode change is needed.
"""

import math
import os
import threading
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.srv import LoadMap
from nav_msgs.msg import OccupancyGrid, Odometry
from opennav_docking_msgs.action import DockRobot
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import SerializePoseGraph
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

from vacuum_interfaces.msg import CleaningStatus, ExplorationStatus, RobotState, SafetyStatus
from vacuum_interfaces.srv import ChangeState, SaveMap, SetNoGoZones

from . import map_files
from . import state_machine as sm
from . import world_check, zone_rules
from .lifecycle_client import ACTIVE, INACTIVE, UNCONFIGURED, ManagedNode, bring_down, bring_up, call_service, stand_by
from .watchdog import JobWatchdog, NodeHealth, TopicWatchdog

SLAM_NODE = "slam_toolbox"
LOCALIZATION_NODES = ["map_server", "amcl"]
NAVIGATION_NODES = [
    "filter_mask_server",
    "costmap_filter_info_server",
    "controller_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
]
DOCKING_NODES = ["docking_server"]
BT_NAVIGATOR = "bt_navigator"
# The tree bt_navigator runs for a goal that names none (the app's Go to, the explorer's goals).
# It reads this setting each time it is configured, and declares it the first time.
TREE_PARAM = "default_nav_to_pose_bt_xml"
# What _nav_map holds while Nav2 runs for exploring, without a saved map. No map can have this
# name, so a mode with a real map always restarts Nav2 (with that map's zones and the normal tree).
EXPLORING_NAV = "<exploring>"
BASE_FRAME = "base_link"  # the robot's own frame: odometry and AMCL use it (the robot has no base_footprint)
# How old the robot's place on the map may be and still count, when a new no-go zone is checked
# against it. AMCL republishes map -> odom with every scan, the simulator odom -> base_link with every frame.
ROBOT_POSE_MAX_AGE = 2.0
LOAD_MASK_TIMEOUT = 5.0  # s for filter_mask_server to load a new mask; it takes milliseconds

# What the docking server reports while it works, and why it gives up, in plain words.
_DOCK_PROGRESS = {
    DockRobot.Feedback.NAV_TO_STAGING_POSE: "docking: driving to the dock",
    DockRobot.Feedback.INITIAL_PERCEPTION: "docking: looking for the dock",
    DockRobot.Feedback.CONTROLLING: "docking: final approach",
    DockRobot.Feedback.WAIT_FOR_CHARGE: "docking: waiting for the charger",
    DockRobot.Feedback.RETRY: "docking: trying again",
}
_DOCK_ERRORS = {
    DockRobot.Result.DOCK_NOT_IN_DB: "this dock is not in the list of docks",
    DockRobot.Result.DOCK_NOT_VALID: "the dock's settings are not valid",
    DockRobot.Result.FAILED_TO_STAGE: "could not reach the spot in front of the dock",
    DockRobot.Result.FAILED_TO_DETECT_DOCK: "could not see the dock",
    DockRobot.Result.FAILED_TO_CONTROL: "could not drive onto the dock",
    DockRobot.Result.FAILED_TO_CHARGE: "reached the dock but it is not charging",
}

# What the safety gate calls an input it has stopped receiving, and what that means as a fault.
_GATE_INPUT_FAULTS = {
    "cliff": "cliff sensor data stopped",
    "bumper": "bumper data stopped",
    "mode": "the safety gate is not receiving the robot state",
}
# Gate reasons that only repeat the mode; not worth a second line in the RViz label.
_PLAIN_GATE_REASONS = {"", sm.IDLE, sm.DOCKED, sm.FAILSAFE, "mode change in progress"}
_LABEL_COLORS = {  # red, green, blue. Dark and strong: the label is read against a light grey map.
    sm.IDLE: (0.15, 0.15, 0.15),
    sm.MAPPING: (0.0, 0.25, 0.8),
    sm.NAVIGATION: (0.0, 0.45, 0.1),
    sm.MANUAL: (0.75, 0.35, 0.0),
    sm.DOCKING: (0.45, 0.1, 0.6),
    sm.DOCKED: (0.0, 0.4, 0.45),
    sm.CLEANING: (0.0, 0.5, 0.5),
    sm.EXPLORING: (0.0, 0.3, 0.6),
    sm.FAILSAFE: (0.85, 0.0, 0.0),
}
_NOTICE_SECONDS = 8.0
# Requests that stop the robot (and a Reset back to mapping, see state_machine.waits_when_busy),
# asked for while another change is in progress, wait for it instead of being refused as busy.
# The wait is longer than the slowest change seen in failure cases (a node refusing three times
# at up to 20 s each, then failsafe: about 100 s).
# What the cleaning node reports while it has a job (CleaningStatus.state).
_CLEANING_JOB_STATES = (CleaningStatus.STARTING, CleaningStatus.CLEANING, CleaningStatus.PAUSED, CleaningStatus.ENDING)
# The same for the exploration node (ExplorationStatus.state).
_EXPLORATION_JOB_STATES = (
    ExplorationStatus.STARTING, ExplorationStatus.EXPLORING, ExplorationStatus.RETURNING, ExplorationStatus.ENDING
)
_STOP_WAIT_SECONDS = 120.0


class ModeManager(Node):
    def __init__(self):
        super().__init__("mode_manager")
        self.declare_parameter("maps_dir", "")
        # The simulator's world these maps belong to; checked against /sim/world ('' = no check).
        self.declare_parameter("world", "")
        self.declare_parameter("default_map", "house")
        # Where the robot stands when docked (x, y, heading on the map): no new no-go zone may come
        # near it. The launch file reads it from docking.yaml, where the docking server gets it too.
        self.declare_parameter("dock_pose", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("start_pose", [0.0, 0.0, 0.0])  # map -> odom when nothing better is known
        self.declare_parameter("scan_timeout", 1.5)
        self.declare_parameter("odom_timeout", 1.0)
        self.declare_parameter("safety_timeout", 1.0)  # s without /safety_status before the gate counts as dead
        self.declare_parameter("sensor_fault_delay", 1.5)  # s the gate must miss an input before it is a fault
        # s without /cleaning/status in cleaning mode before failsafe. Shorter than the launch file's
        # 2 s restart delay, so a cleaning node that dies is noticed before its replacement reports.
        self.declare_parameter("cleaning_timeout", 1.0)
        self.declare_parameter("exploration_timeout", 1.0)  # the same for /exploration/status in exploring mode
        # Nav2's behaviour trees (absolute paths, from the launch file): the normal one, and the one
        # for exploring, which never spins or backs the robot up.
        self.declare_parameter("navigation_tree", "")
        self.declare_parameter("exploring_tree", "")
        self.declare_parameter("startup_grace", 30.0)
        self.declare_parameter("dock_id", "home_dock")  # which dock in the docking server's list
        self.declare_parameter("max_staging_time", 120.0)  # s allowed to reach the spot in front of the dock
        self.declare_parameter("docking_settle_time", 0.5)  # s; must be longer than AMCL's transform_tolerance (0.3)
        self.declare_parameter("health_period", 2.0)

        self._maps_dir = self.get_parameter("maps_dir").value
        if not self._maps_dir:
            raise RuntimeError("parameter maps_dir is required")
        os.makedirs(self._maps_dir, exist_ok=True)
        dock_pose = self.get_parameter("dock_pose").value
        if dock_pose is None or len(dock_pose) != 3:
            raise RuntimeError("parameter dock_pose is required: the docked robot's x, y and heading on the map")
        self._dock_xy = (float(dock_pose[0]), float(dock_pose[1]))
        self._start_pose = tuple(float(v) for v in self.get_parameter("start_pose").value)
        self._navigation_tree = self.get_parameter("navigation_tree").value
        self._exploring_tree = self.get_parameter("exploring_tree").value
        for name, path in (("navigation_tree", self._navigation_tree), ("exploring_tree", self._exploring_tree)):
            if not os.path.isfile(path):
                raise RuntimeError(f"parameter {name} must name a behaviour tree file (got '{path}')")

        # --- state ---------------------------------------------------------------
        self._cleaning_state = ("", 0.0)  # the cleaning node's last reported job state, and when (monotonic)
        self._exploration_state = ("", 0.0)  # the same for the exploration node
        self._sm = sm.build_state_machine(
            self._has_map,
            self._active_faults,
            self._cleaning_job_starting,
            self._exploration_job_starting,
            self._resume_mapping_problem,
        )
        self._change: Optional[Tuple[str, str]] = None  # the mode change in progress: (from, to)
        self._lifted_in_failsafe = False  # was the robot off the floor at any time in this failsafe?
        self._slam_on = False  # slam_toolbox was switched on by this manager and not switched off since
        self._transition_lock = threading.Lock()  # held for the whole of a mode change
        self._data_lock = threading.Lock()  # guards the small fields below
        self._started = False
        self._transitioning = True  # true until start-up has brought the robot to idle
        self._reason = "starting up"
        self._entered = self.get_clock().now()
        self._faults: List[str] = []

        self._current_map = self._load_current_map()
        self._localized_map = ""  # map AMCL is running on, "" when off
        self._nav_map = ""  # map the Nav2 group was brought up for, "" when off
        self._session_map = ""  # map saved during the current mapping session
        self._map_odom_by_map: Dict[str, map_files.Pose2D] = {}  # last known map -> odom per saved map

        self._odom_pose: Optional[map_files.Pose2D] = None
        self._odom_stamp = None
        self._map_odom: Optional[map_files.Pose2D] = None
        self._map_odom_wall = 0.0
        self._odom_base: Optional[map_files.Pose2D] = None  # odom -> base_link, from /tf
        self._odom_base_wall = 0.0
        self._map_msg: Optional[OccupancyGrid] = None
        self._map_wall = 0.0

        self._world = self.get_parameter("world").value
        self._sim_world: Optional[str] = None  # the world the simulator announced, None until heard
        self._safety: Optional[SafetyStatus] = None  # latest report from the safety gate
        self._gate_missing_since: Dict[str, float] = {}  # gate input -> when it was first reported missing
        self._sensor_fault_delay = float(self.get_parameter("sensor_fault_delay").value)
        self._notice = ""  # a short message for the person watching, shown in the RViz label
        self._notice_until = 0.0
        self._notice_state = ""  # the mode the notice was about; it is dropped when the mode changes
        self._label_text = ("", "")

        # --- fault detectors ------------------------------------------------------
        now = time.monotonic()
        grace = float(self.get_parameter("startup_grace").value)
        self._started_wall = now
        self._startup_grace = grace
        self._scan_watchdog = TopicWatchdog("lidar", float(self.get_parameter("scan_timeout").value), grace, now)
        self._odom_watchdog = TopicWatchdog("odometry", float(self.get_parameter("odom_timeout").value), grace, now)
        self._safety_watchdog = TopicWatchdog(
            "safety gate", float(self.get_parameter("safety_timeout").value), grace, now
        )
        # The cleaning node drives the robot in cleaning mode; if it dies there, nothing ends the job.
        self._cleaning_watchdog = JobWatchdog("cleaning node", float(self.get_parameter("cleaning_timeout").value))
        # The exploration node owns the explorer in exploring mode; the same goes for it.
        self._exploration_watchdog = JobWatchdog(
            "exploration node", float(self.get_parameter("exploration_timeout").value)
        )

        # --- ROS plumbing ---------------------------------------------------------
        self._client_group = ReentrantCallbackGroup()
        service_group = ReentrantCallbackGroup()
        data_group = MutuallyExclusiveCallbackGroup()

        self._slam = ManagedNode(self, SLAM_NODE, self._client_group)
        self._localization = [ManagedNode(self, n, self._client_group) for n in LOCALIZATION_NODES]
        self._navigation = [ManagedNode(self, n, self._client_group) for n in NAVIGATION_NODES]
        self._docking = [ManagedNode(self, n, self._client_group) for n in DOCKING_NODES]
        self._map_server, self._amcl = self._localization
        self._filter_mask_server = self._navigation[0]
        self._bt_navigator = next(n for n in self._navigation if n.name == BT_NAVIGATOR)
        # Loads a new keepout mask into the running mask server; the costmaps take it from there.
        self._load_mask = self.create_client(
            LoadMap, f"{self._filter_mask_server.name}/load_map", callback_group=self._client_group
        )
        self._all_nodes = [self._slam] + self._localization + self._navigation + self._docking

        # Docking is run by the docking server; the manager starts it, follows it, and ends it.
        self._dock_client = ActionClient(self, DockRobot, "dock_robot", callback_group=self._client_group)
        self._dock_goal = None  # handle of the docking in progress
        self._dock_token = 0  # identifies the current docking; results of older ones are ignored
        self._dock_ended = threading.Event()  # set when the docking server has finished its last request
        self._dock_ended.set()
        self._health = {n.name: NodeHealth(n.name) for n in self._all_nodes}
        self._serialize = self.create_client(
            SerializePoseGraph, f"{SLAM_NODE}/serialize_map", callback_group=self._client_group
        )

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._state_pub = self.create_publisher(RobotState, "robot_state", latched)
        self._label_pub = self.create_publisher(MarkerArray, "mode_label", latched)
        self._initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 5)

        self.create_subscription(LaserScan, "scan", self._on_scan, qos_profile_sensor_data, callback_group=data_group)
        self.create_subscription(Odometry, "odom", self._on_odom, qos_profile_sensor_data, callback_group=data_group)
        self.create_subscription(TFMessage, "tf", self._on_tf, 100, callback_group=data_group)
        self.create_subscription(OccupancyGrid, "map", self._on_map, latched, callback_group=data_group)
        self.create_subscription(String, "sim/world", self._on_sim_world, latched, callback_group=data_group)
        self.create_subscription(SafetyStatus, "safety_status", self._on_safety, 10, callback_group=data_group)
        # Not latched on purpose: an old copy delivered on subscribing must not count as a sign of life.
        self.create_subscription(CleaningStatus, "cleaning/status", self._on_cleaning_status, 10, callback_group=data_group)
        self.create_subscription(
            ExplorationStatus, "exploration/status", self._on_exploration_status, 10, callback_group=data_group
        )
        # Leaving cleaning or exploring mode: every Nav2 goal is cancelled before the next mode starts.
        self._cancel_nav = [
            self.create_client(CancelGoal, f"{action}/_action/cancel_goal", callback_group=self._client_group)
            for action in ("navigate_to_pose", "follow_path")
        ]
        self.create_subscription(PoseStamped, "goal_pose", self._on_goal_pose, 5, callback_group=data_group)

        self.create_service(ChangeState, "change_state", self._on_change_state, callback_group=service_group)
        self.create_service(SaveMap, "save_map", self._on_save_map, callback_group=service_group)
        self.create_service(SetNoGoZones, "set_no_go_zones", self._on_set_no_go_zones, callback_group=service_group)

        # These timers run on real time, not simulation time. The node uses simulation time for
        # its stamps, but if the simulator freezes its clock stops, and a watchdog driven by that
        # clock would stop with it, exactly when it is needed.
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        # Twice a second: the safety gate lets nobody drive once it has not heard the mode for 1.5 s.
        self.create_timer(0.5, self._publish_state, callback_group=MutuallyExclusiveCallbackGroup(), clock=steady)
        self.create_timer(0.2, self._stream_tick, callback_group=MutuallyExclusiveCallbackGroup(), clock=steady)
        self.create_timer(
            float(self.get_parameter("health_period").value),
            self._health_tick,
            callback_group=MutuallyExclusiveCallbackGroup(),
            clock=steady,
        )
        self._publish_state()
        self.get_logger().info(
            f"mode manager started for world '{self._world or 'any'}'; maps in {self._maps_dir}, "
            f"current map: {self._current_map or 'none'}"
        )

    # ------------------------------------------------------------------ data in
    def _on_scan(self, _msg):
        self._scan_watchdog.beat(time.monotonic())

    def _on_odom(self, msg):
        self._odom_watchdog.beat(time.monotonic())
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        with self._data_lock:
            self._odom_pose = (p.x, p.y, map_files.yaw_from_quaternion(q.x, q.y, q.z, q.w))
            self._odom_stamp = msg.header.stamp

    def _on_tf(self, msg):
        for t in msg.transforms:
            parent, child = t.header.frame_id, t.child_frame_id
            if (parent, child) not in (("map", "odom"), ("odom", BASE_FRAME)):
                continue
            tr, q = t.transform.translation, t.transform.rotation
            pose = (tr.x, tr.y, map_files.yaw_from_quaternion(q.x, q.y, q.z, q.w))
            with self._data_lock:
                if parent == "map":
                    self._map_odom, self._map_odom_wall = pose, time.monotonic()
                else:
                    self._odom_base, self._odom_base_wall = pose, time.monotonic()

    def _on_map(self, msg):
        with self._data_lock:
            self._map_msg = msg
            self._map_wall = time.monotonic()

    def _on_sim_world(self, msg):
        with self._data_lock:
            self._sim_world = msg.data

    def _world_problem(self) -> str:
        with self._data_lock:
            return world_check.world_problem(self._world, self._sim_world, waiting_over=self._started)

    def _on_safety(self, msg):
        now = time.monotonic()
        self._safety_watchdog.beat(now)
        with self._data_lock:
            self._safety = msg
            for name in list(self._gate_missing_since):
                if name not in msg.stale:
                    del self._gate_missing_since[name]
            for name in msg.stale:
                self._gate_missing_since.setdefault(name, now)
        if msg.off_floor and self._sm.state == sm.FAILSAFE:
            self._lifted_in_failsafe = True  # a map being made cannot be gone back to after this
        self._publish_label()  # only sends when the text changed

    def _on_cleaning_status(self, msg):
        now = time.monotonic()
        self._cleaning_watchdog.beat(now, has_job=msg.state in _CLEANING_JOB_STATES)
        self._cleaning_state = (msg.state, now)

    def _cleaning_job_starting(self) -> bool:
        """Does the cleaning node report, right now, a job waiting for cleaning mode?"""
        state, at = self._cleaning_state
        return state == CleaningStatus.STARTING and time.monotonic() - at < 1.5

    def _on_exploration_status(self, msg):
        now = time.monotonic()
        self._exploration_watchdog.beat(now, has_job=msg.state in _EXPLORATION_JOB_STATES)
        self._exploration_state = (msg.state, now)

    def _exploration_job_starting(self) -> bool:
        """Does the exploration node report, right now, a job waiting for exploring mode?"""
        state, at = self._exploration_state
        return state == ExplorationStatus.STARTING and time.monotonic() - at < 1.5

    def _slam_running(self) -> bool:
        """Is slam_toolbox's map -> odom transform still arriving? Only meaningful while a map is
        being made (AMCL publishes the same transform otherwise). Answers from what has been
        received: it is asked twice a second and must never wait on a call."""
        return sm.slam_map_alive(self._slam_on, self._fresh_map_odom() is not None)

    def _resume_mapping_problem(self) -> str:
        """Why failsafe cannot be left for mapping with the map kept, or ""."""
        return sm.resume_mapping_problem(self._sm.previous, self._lifted_in_failsafe, self._slam_running())

    def _making_map(self) -> bool:
        return sm.making_map(self._sm.state, self._sm.previous, self._slam_running())

    def _cancel_nav_goals(self) -> None:
        """Cancel every Nav2 goal (all-zero id: all of them): what the cleaning node or the explorer
        was driving, and anything else. Best effort and quick: the goals also end when Nav2 is
        switched off."""
        for client in self._cancel_nav:
            if not client.service_is_ready():
                continue
            done = threading.Event()
            client.call_async(CancelGoal.Request()).add_done_callback(lambda _f, d=done: d.set())
            done.wait(2.0)

    def _on_goal_pose(self, _msg):
        """A goal clicked in RViz. Nav2 only listens in navigation mode; say so instead of silence."""
        state = self._sm.state
        if state == sm.NAVIGATION and not self._transitioning:
            return
        if state == sm.CLEANING:
            # Nav2 runs and takes the goal; the cleaning node sees a goal it did not send and stops its job.
            self.get_logger().warn("a goal arrived on /goal_pose during cleaning: the cleaning job will stop")
            return
        if state == sm.EXPLORING:
            # Nav2 runs and takes the goal. The explorer does not publish the ids of its own goals,
            # so a stranger's cannot be told apart and cancelled; its next goal replaces this one.
            self.get_logger().warn(
                "a goal arrived on /goal_pose during exploring: Nav2 drives to it until the explorer's next goal"
            )
            return
        self._notice = f"goal ignored in {state} mode"
        self._notice_state = state
        self._notice_until = time.monotonic() + _NOTICE_SECONDS
        self.get_logger().warn(
            f"a goal arrived on /goal_pose but the robot is in {state} mode, so nothing will happen. "
            "Switch first: ros2 run vacuum_mode_manager mode navigation"
        )
        self._publish_label()

    # ------------------------------------------------------------------ maps
    def _map_yaml(self, name: str) -> str:
        return os.path.join(self._maps_dir, name + ".yaml")

    def _zones_file(self, name: str) -> str:
        return map_files.zones_path(self._maps_dir, name)

    def _has_map(self) -> bool:
        return bool(self._current_map) and os.path.isfile(self._map_yaml(self._current_map))

    def _load_current_map(self) -> str:
        """The map to use: the one remembered from last time, else the default if it exists."""
        marker = os.path.join(self._maps_dir, "current_map.txt")
        candidates = []
        if os.path.isfile(marker):
            with open(marker) as f:
                candidates.append(f.read().strip())
        candidates.append(self.get_parameter("default_map").value)
        for name in candidates:
            if name and os.path.isfile(self._map_yaml(name)):
                return name
        return ""

    def _set_current_map(self, name: str) -> None:
        self._current_map = name
        with open(os.path.join(self._maps_dir, "current_map.txt"), "w") as f:
            f.write(name + "\n")

    # ------------------------------------------------------------------ faults
    def _active_faults(self) -> List[str]:
        """Problems that are present right now and must clear before failsafe can be left."""
        now = time.monotonic()
        faults = self._stream_faults(now)
        faults += [f for f in (h.not_responding() for h in self._health.values()) if f]
        return faults

    def _stream_faults(self, now: float) -> List[str]:
        """Faults that can be read off the incoming data, without asking any node anything."""
        watchdogs = (self._scan_watchdog, self._odom_watchdog, self._safety_watchdog)
        faults = [f for f in (w.fault(now) for w in watchdogs) if f] + self._gate_faults(now)
        wrong_world = self._world_problem()  # never clears: the robot stays in failsafe
        if wrong_world:
            faults.append(wrong_world)
        settled = not self._transitioning
        jobs = (
            self._cleaning_watchdog.fault(now, active=self._sm.state == sm.CLEANING and settled),
            self._exploration_watchdog.fault(now, active=self._sm.state == sm.EXPLORING and settled),
        )
        return faults + [f for f in jobs if f]

    def _stream_tick(self):
        """Five times a second: has a sensor or the safety gate gone quiet? Much faster than the
        poll of the managed nodes, so that a safety gate that crashes is always noticed before
        its replacement starts, and the robot does not quietly carry on afterwards."""
        if not self._started or self._sm.state == sm.FAILSAFE:
            return
        faults = self._stream_faults(time.monotonic())
        if faults and self._transition_lock.acquire(blocking=False):
            try:
                if self._sm.state != sm.FAILSAFE:
                    self._faults = faults
                    self._enter_failsafe("fault: " + "; ".join(faults))
            finally:
                self._transition_lock.release()

    def _gate_faults(self, now: float) -> List[str]:
        """Problems the safety gate reports. The gate has already stopped the robot where needed;
        these make the stop visible as a failsafe that someone has to acknowledge."""
        with self._data_lock:
            status = self._safety
            missing = dict(self._gate_missing_since)
        if status is None or self._safety_watchdog.fault(now):
            return []  # nothing recent to go on; the watchdog reports the silence itself
        faults = []
        if status.off_floor:
            faults.append("robot is off the floor (lifted, or fell off an edge)")
        # A short gap (a simulator hiccup) only makes the gate hold forward motion for a moment.
        for name, since in sorted(missing.items()):
            if now - since > self._sensor_fault_delay:
                faults.append(_GATE_INPUT_FAULTS.get(name, f"safety gate input '{name}' stopped"))
        return faults

    def _expected_states(self) -> Dict[str, Optional[str]]:
        """The lifecycle state each managed node should be in for the current mode (None = any)."""
        state = self._sm.state
        if state == sm.FAILSAFE:
            return {n.name: None for n in self._all_nodes}
        res = sm.resources_for(state, self._has_map())
        expected = {self._slam.name: ACTIVE if res[sm.SLAM] else UNCONFIGURED}
        for n in self._localization:
            expected[n.name] = ACTIVE if res[sm.LOCALIZATION] else UNCONFIGURED
        for n in self._navigation:
            expected[n.name] = ACTIVE if res[sm.NAVIGATION_GROUP] else UNCONFIGURED
        for n in self._docking:
            # Switched off means inactive here, not unconfigured: the docking server (0.0.2)
            # cannot be configured a second time after a cleanup, so it is configured once.
            expected[n.name] = ACTIVE if res[sm.DOCKING_GROUP] else INACTIVE
        return expected

    def _health_tick(self):
        if not self._started:
            self._try_startup()
            return
        if self._transition_lock.locked():
            return
        expected = self._expected_states()
        for n in self._all_nodes:
            if self._transition_lock.locked():
                return  # a mode change started mid-poll; states are in flux
            self._health[n.name].record(n.state(timeout=1.0), expected[n.name])
        faults = self._active_faults()
        wrong = [f for f in (self._health[n].wrong_state(expected[n]) for n in expected) if f]
        self._faults = faults + wrong
        if self._faults and self._sm.state != sm.FAILSAFE:
            if self._transition_lock.acquire(blocking=False):
                try:
                    self._enter_failsafe("fault: " + "; ".join(self._faults))
                finally:
                    self._transition_lock.release()

    # ------------------------------------------------------------------ start-up
    def _try_startup(self):
        """Wait for every managed node and the sensors, then bring the robot to idle."""
        missing = [n.name for n in self._all_nodes if n.state(timeout=1.0) is None]
        waiting = [f"node {m}" for m in missing]
        if not self._scan_watchdog.seen:
            waiting.append("lidar data")
        if not self._odom_watchdog.seen:
            waiting.append("odometry data")
        if not self._safety_watchdog.seen:
            waiting.append("the safety gate")
        # Seen once: a stack restarted next to a running simulator got sensor data but never the
        # simulator's clock, and AMCL then failed with a message that pointed nowhere. Name it.
        if self.get_parameter("use_sim_time").value and self.get_clock().now().nanoseconds == 0:
            waiting.append("the simulator clock (/clock)")
        with self._data_lock:
            if self._world and self._sim_world is None:
                waiting.append(world_check.WAITING)
        if waiting:
            self._reason = "starting up: waiting for " + ", ".join(waiting)
            if time.monotonic() - self._started_wall > self._startup_grace:
                with self._transition_lock:
                    self._started = True
                    self._transitioning = False
                    self._enter_failsafe("start-up failed: no answer from " + ", ".join(waiting))
            return
        wrong_world = self._world_problem()
        if wrong_world:  # never bring up localization on another world's map
            with self._transition_lock:
                self._started = True
                self._transitioning = False
                self._faults = [wrong_world]
                self._enter_failsafe("start-up failed: " + wrong_world)
            return
        with self._transition_lock:
            # If this node was restarted while the robot was localized, carry the robot's place
            # on the map over, so localization restarts from where it was and not from the dock.
            if self._has_map() and self._amcl.state() == ACTIVE and self._slam.state() != ACTIVE:
                transform = self._fresh_map_odom()
                if transform is not None:
                    self._map_odom_by_map[self._current_map] = transform
            ok, why = self._apply_mode(sm.IDLE)
            self._started = True
            self._transitioning = False
            if ok:
                self._reason = "started"
                self._entered = self.get_clock().now()
                self.get_logger().info(f"ready in idle; localization {'on' if self._localized_map else 'off (no map)'}")
            else:
                self._enter_failsafe(f"start-up failed: {why}")
        self._publish_state()

    # ------------------------------------------------------------------ state out
    def _publish_state(self):
        msg = RobotState()
        msg.stamp = self._entered.to_msg()
        msg.state = self._sm.state
        msg.previous_state = self._sm.previous
        msg.reason = self._reason
        msg.transitioning = self._transitioning
        msg.allowed_transitions = [] if self._transitioning else self._sm.allowed()
        msg.faults = list(self._faults) if self._started else []
        msg.current_map = self._current_map if self._has_map() else ""
        msg.making_map = self._making_map()
        self._state_pub.publish(msg)
        self._publish_label(force=True)

    def _label(self) -> Tuple[str, str]:
        """The mode, and the one thing most worth knowing right now (may be empty)."""
        state = self._sm.state
        with self._data_lock:
            gate_reason = self._safety.blocked_reason if self._safety is not None else ""
        if time.monotonic() < self._notice_until and self._notice_state == state:
            detail = self._notice
        elif self._transitioning or state == sm.FAILSAFE:
            detail = self._reason
        elif gate_reason not in _PLAIN_GATE_REASONS:
            detail = gate_reason
        else:
            detail = ""
        if len(detail) > 48:
            detail = detail[:45] + "..."
        return state.upper(), detail

    def _publish_label(self, force: bool = False):
        label = self._label()
        if label == self._label_text and not force:
            return
        self._label_text = label
        mode, detail = label
        color = _LABEL_COLORS.get(self._sm.state, (0.15, 0.15, 0.15))
        # The mode in large letters, the detail in small ones between it and the robot.
        self._label_pub.publish(
            MarkerArray(markers=[self._text_marker(0, mode, 0.2, 6, color), self._text_marker(1, detail, 0.1, 7, color)])
        )

    @staticmethod
    def _text_marker(marker_id: int, text: str, letter_height: float, lift: int, color) -> Marker:
        """One line of text that follows the robot in RViz. RViz centres text on its anchor, which
        would put it on top of the robot; `lift` blank lines underneath push the line up instead."""
        marker = Marker()
        marker.header.frame_id = "base_link"  # stamp left at zero: RViz places it with the newest transform
        marker.frame_locked = True  # and keeps it with the robot as the robot moves
        marker.ns, marker.id = "mode", marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD if text else Marker.DELETE
        marker.pose.position.z = 0.5
        marker.pose.orientation.w = 1.0
        marker.scale.z = letter_height  # metres
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 1.0
        # RViz draws an ordinary space as wide as two capital letters; a no-break space is narrower.
        marker.text = "\n".join([text.replace(" ", "\u00a0")] + [" "] * lift)
        return marker

    # ------------------------------------------------------------------ requests
    def _on_change_state(self, request, response):
        target = request.new_state.strip().lower()
        if not self._started:
            response.success, response.message = False, f"not ready yet ({self._reason})"
            return response
        waited_for = None  # the change this request had to wait for, if any
        if not self._transition_lock.acquire(blocking=False):
            if not sm.waits_when_busy(self._sm.state, target):
                response.success, response.message = False, "busy: another state change is in progress"
                return response
            waited_for = self._change
            # A stop (or a Reset) is never refused. The robot is already held still while a change
            # is in progress (the safety gate lets nobody drive), so wait for it to finish, then act.
            self.get_logger().info(f"{target} requested during a mode change: applying it once the change has finished")
            if not self._transition_lock.acquire(timeout=_STOP_WAIT_SECONDS):
                response.success = False
                response.message = f"the mode change in progress had not finished after {_STOP_WAIT_SECONDS:.0f} s"
                return response
        try:
            source = self._sm.state
            if target == source:
                response.success, response.message = True, f"already in {source}"
                return response
            # A stop while exploring keeps the map: it is carried out as a change to mapping. In
            # failsafe with a map that can be gone back to, a stop leaves the robot where it is.
            resumable = source == sm.FAILSAFE and not self._resume_mapping_problem()
            asked, target = target, sm.carried_out_as(source, target, waited_for, resumable)
            kept_map = target != asked
            if target == source:
                # The exploration had just ended when the stop got its turn, or the robot is in failsafe.
                response.success = True
                response.message = f"the robot stands still in {source}: the map being made is kept" + (
                    "; Reset goes back to mapping" if source == sm.FAILSAFE else "")
                return response
            self._faults = self._active_faults() if source == sm.FAILSAFE else self._faults
            ok, why = self._sm.check(asked)
            if ok and source == sm.FAILSAFE and target == sm.MAPPING and self._slam.state() != ACTIVE:
                # The map was still there a moment ago (the check passed on slam_toolbox's last
                # transform). Starting slam_toolbox again would begin an empty map and call it kept.
                ok, why = False, "slam_toolbox is no longer running, and the map being made went with it: request idle"
            if not ok:
                response.success, response.message = False, why
                self.get_logger().warn(f"refused {source} -> {asked}: {why}")
                return response
            if target == sm.FAILSAFE:
                self._enter_failsafe("requested (emergency stop)")
                response.success, response.message = True, "in failsafe; request idle to leave it"
                return response

            change = f"{source} -> {target}" + (f" (asked for {asked}: the map being made is kept)" if kept_map else "")
            self.get_logger().info(f"changing {change}")
            self._transitioning = True
            self._change = (source, target)
            self._reason = f"changing {source} -> {target}"
            self._publish_state()  # the safety gate stops the robot on seeing "transitioning"
            if source == sm.DOCKING:
                self._cancel_docking()
            if source in (sm.CLEANING, sm.EXPLORING):
                # A row, or the explorer's goal, must not drive on in the next mode. For exploring
                # this is also what ends the goal of an explorer that has died.
                self._cancel_nav_goals()
            ok, why = self._apply_mode(target)
            if ok and target == sm.DOCKING:
                ok, why = self._start_docking()
            self._transitioning = False
            if ok:
                self._sm.set(target)
                if target == sm.DOCKING:
                    self._reason = "docking: starting"
                elif kept_map:
                    self._reason = "stopped exploring; the map is kept"
                else:
                    self._reason = "requested"
                self._entered = self.get_clock().now()
                self._faults = []
                for h in self._health.values():
                    h.reset()
                response.success = True
                response.message = f"now in {target}" + (": exploring stopped, the map is kept" if kept_map else "")
                self.get_logger().info(f"now in {target}")
            else:
                self._enter_failsafe(f"change {source} -> {target} failed: {why}")
                response.success, response.message = False, f"failed and entered failsafe: {why}"
            return response
        finally:
            self._change = None
            self._transition_lock.release()
            self._publish_state()

    def _on_save_map(self, request, response):
        name = request.name.strip() or "house"
        if self._sm.state != sm.MAPPING or self._transitioning:
            response.success, response.message = False, f"maps can only be saved in mapping mode (now {self._sm.state})"
            return response
        why = map_files.map_name_problem(name)
        if why:
            response.success, response.message = False, why
            return response
        if os.path.isfile(self._map_yaml(name)) and not request.replace:
            response.success, response.name_taken = False, True
            response.message = f"a map named '{name}' already exists; save with replace to overwrite it"
            return response
        # slam_toolbox republishes the map every couple of seconds; wait for one newer than this request
        asked = time.monotonic()
        while time.monotonic() - asked < 8.0:
            with self._data_lock:
                if self._map_msg is not None and self._map_wall > asked:
                    break
            time.sleep(0.1)
        with self._data_lock:
            grid = self._map_msg
        if grid is None:
            response.success, response.message = False, "no map received from slam_toolbox yet"
            return response
        stem = os.path.join(self._maps_dir, name)
        image = map_files.grid_to_image(grid.data, grid.info.width, grid.info.height)
        origin = (grid.info.origin.position.x, grid.info.origin.position.y)
        response.path = map_files.write_map(stem, image, grid.info.resolution, origin)

        # slam_toolbox's own format, so a map can be continued later. Not needed for navigation.
        graph = call_service(self._serialize, SerializePoseGraph.Request(filename=stem), 15.0)
        note = "" if graph is not None else " (pose graph not saved: slam_toolbox did not answer)"

        self._set_current_map(name)
        self._session_map = name
        response.success = True
        response.message = f"saved {grid.info.width}x{grid.info.height} cells as '{name}'{note}"
        self.get_logger().info(response.message)
        self._publish_state()
        return response

    # ------------------------------------------------------------------ no-go zones
    def _on_set_no_go_zones(self, request, response):
        """Replace the zones of the map in use. Holds the transition lock throughout, so no mode
        change starts while the zones file and the mask are being written, and none is under way."""
        if not self._started:
            response.success, response.message = False, f"not ready yet ({self._reason})"
        elif not self._transition_lock.acquire(blocking=False):
            response.success, response.message = False, "the robot is changing modes: try again in a moment"
        else:
            try:
                response.success, response.message = self._set_no_go_zones(request.map_name.strip(), request.zones)
            except Exception as exc:  # a file that cannot be written, say: answer, do not crash
                response.success, response.message = False, f"could not save the no-go zones: {exc}"
            finally:
                self._transition_lock.release()
        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().warn(f"no-go zones not changed: {response.message}")
        return response

    def _set_no_go_zones(self, map_name: str, zone_msgs) -> Tuple[bool, str]:
        """Check, save, and put in force. Call with the transition lock held."""
        name = self._current_map if self._has_map() else ""
        why = zone_rules.mode_problem(self._sm.state, self._transitioning, name, map_name, self._making_map())
        if why:
            return False, why
        zones = [{"name": z.name, "polygon": [[p.x, p.y] for p in z.polygon.points]} for z in zone_msgs]
        path = self._zones_file(name)
        try:
            saved = map_files.read_zones(path)
        except Exception:  # a broken file: every zone counts as new, which only means more checks
            saved = []
        extent = zone_rules.map_extent(map_files.read_map_meta(self._map_yaml(name)))
        why = zone_rules.check_zones(zones, extent, robot=self._robot_on_map(), dock=self._dock_xy, saved=saved)
        if why:
            return False, why

        map_files.write_zones(path, zones)
        listed = ", ".join(z["name"] for z in zones) or "none"
        done = f"saved the no-go zones of '{name}': {listed}"
        nav2_running = self._nav_map == name
        later = done + "; they apply the next time navigation starts"
        try:
            # From the file just written, exactly as _start_navigation will make it next time.
            mask_yaml = self._write_mask(name, map_files.read_zones(path))
        except Exception as exc:  # the zones are saved: navigation makes the mask again when it starts
            self.get_logger().error(f"could not write the keepout mask for '{name}': {exc}")
            return True, later if nav2_running else done
        if not nav2_running:
            return True, done
        why = self._load_new_mask(mask_yaml)
        if why:
            self.get_logger().error(f"the running navigation did not take the new keepout mask: {why}")
            return True, later
        return True, done + "; the robot keeps out of them from now on"

    def _write_mask(self, name: str, zones) -> str:
        """Nav2's keepout mask for a map, made from these zones. Returns the mask's yaml path."""
        return map_files.write_keepout_mask(self._map_yaml(name), zones, map_files.keepout_stem(self._maps_dir, name))

    def _load_new_mask(self, mask_yaml: str) -> str:
        """Make the running filter_mask_server load the mask again from its file. It then publishes
        it (latched) and the keepout filter of each costmap replaces its copy and applies the whole
        mask on its next update. Returns why not, or ""."""
        response = call_service(self._load_mask, LoadMap.Request(map_url=mask_yaml), LOAD_MASK_TIMEOUT)
        if response is None:
            return f"{self._filter_mask_server.name} did not answer"
        if response.result != LoadMap.Response.RESULT_SUCCESS:
            return f"{self._filter_mask_server.name} could not load {mask_yaml} (result {response.result})"
        return ""

    def _robot_on_map(self) -> Optional[Tuple[float, float]]:
        """Where the robot's centre is on the map, if both transforms are recent; else None."""
        now = time.monotonic()
        with self._data_lock:
            if self._map_odom is None or self._odom_base is None:
                return None
            if now - self._map_odom_wall > ROBOT_POSE_MAX_AGE or now - self._odom_base_wall > ROBOT_POSE_MAX_AGE:
                return None
            x, y, _ = map_files.compose(self._map_odom, self._odom_base)
        return x, y

    # ------------------------------------------------------------------ mode changes
    def _enter_failsafe(self, reason: str) -> None:
        """Announce failsafe (which makes the safety gate stop the robot) and switch Nav2 off.
        Call with the transition lock held."""
        self.get_logger().error(f"FAILSAFE: {reason}")
        with self._data_lock:
            off_floor = self._safety is not None and self._safety.off_floor
        self._lifted_in_failsafe = off_floor or "off the floor" in reason
        self._sm.set(sm.FAILSAFE)
        self._reason = reason
        self._entered = self.get_clock().now()
        self._transitioning = False
        self._publish_state()
        # Best effort and quick: a node that is the cause of the fault may not answer at all.
        # Nav2 first: the docking server cannot be interrupted while Nav2 is driving for it.
        self._cancel_docking(wait=1.0)
        bring_down(self._navigation, step_timeout=3.0, deadline=6.0)
        self._dock_ended.wait(6.0)
        stand_by(self._docking, step_timeout=3.0, deadline=6.0)
        self._nav_map = ""
        for h in self._health.values():
            h.reset()

    def _apply_mode(self, target: str) -> Tuple[bool, str]:
        """Switch node groups to what `target` needs: everything unwanted off first, then on."""
        want = sm.resources_for(target, self._has_map())
        if target in (sm.NAVIGATION, sm.CLEANING, sm.DOCKING) and not self._has_map():
            return False, "no saved map"
        # Navigation goes off before docking: see _turn_off.
        for group in (sm.NAVIGATION_GROUP, sm.DOCKING_GROUP, sm.LOCALIZATION, sm.SLAM):
            if not want[group]:
                ok, why = self._turn_off(group)
                if not ok:
                    return False, why
        for group in (sm.SLAM, sm.LOCALIZATION, sm.NAVIGATION_GROUP, sm.DOCKING_GROUP):
            if want[group]:
                ok, why = self._turn_on(group, target)
                if not ok:
                    return False, why
        return True, ""

    def _fresh_map_odom(self, max_age: float = 1.5) -> Optional[map_files.Pose2D]:
        with self._data_lock:
            if self._map_odom is not None and time.monotonic() - self._map_odom_wall < max_age:
                return self._map_odom
        return None

    def _wait_for_map_odom(self, since: float, timeout: float) -> bool:
        """Wait for a map -> odom transform published after `since` (monotonic seconds)."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            with self._data_lock:
                if self._map_odom_wall > since:
                    return True
            time.sleep(0.05)
        return False

    def _turn_off(self, group: str) -> Tuple[bool, str]:
        if group == sm.DOCKING_GROUP:
            # While Nav2 drives the robot to the dock for it, the docking server (0.0.2) does not
            # notice a cancel, and it stops answering if deactivated with a docking still running.
            # Nav2 is already off at this point, which ends that drive; wait for the docking to
            # end before touching the server.
            if not self._dock_ended.wait(12.0):
                self.get_logger().warn("the docking server is still busy 12 s after being told to stop")
            return stand_by(self._docking)
        if group == sm.NAVIGATION_GROUP:
            self._nav_map = ""
            return bring_down(self._navigation)
        if group == sm.SLAM:
            # Remember where slam_toolbox had the robot, so AMCL can take over from the same spot.
            if self._session_map and self._slam.state() == ACTIVE:
                transform = self._fresh_map_odom()
                if transform is not None:
                    self._map_odom_by_map[self._session_map] = transform
            self._session_map = ""
            self._slam_on = False
            return bring_down([self._slam])
        if self._localized_map and self._amcl.state() == ACTIVE:
            transform = self._fresh_map_odom()
            if transform is not None:
                self._map_odom_by_map[self._localized_map] = transform
        self._localized_map = ""
        return bring_down(self._localization)

    def _turn_on(self, group: str, target: str) -> Tuple[bool, str]:
        if group == sm.SLAM:
            return self._start_slam()
        if group == sm.LOCALIZATION:
            return self._start_localization()
        if group == sm.DOCKING_GROUP:
            ok, why = bring_up(self._docking)
            if not ok:
                return False, why
            # The docking server has only just started listening to transforms, and AMCL stamps
            # the robot's place on the map a little ahead (its transform_tolerance). Until the
            # server has listened for longer than that it cannot place the robot, and a docking
            # request fails at once. Wait that long, measured on the robot's own clock.
            settle = float(self.get_parameter("docking_settle_time").value)
            start, wall_end = self.get_clock().now(), time.monotonic() + 10.0
            while (self.get_clock().now() - start).nanoseconds < settle * 1e9 and time.monotonic() < wall_end:
                time.sleep(0.05)
            return True, ""
        return self._start_navigation(exploring=target == sm.EXPLORING)

    # ------------------------------------------------------------------ docking
    def _start_docking(self) -> Tuple[bool, str]:
        """Ask the docking server to dock. Returns once it has accepted; the result comes later."""
        if not self._dock_client.wait_for_server(timeout_sec=5.0):
            return False, "the docking server's action is not available"
        goal = DockRobot.Goal()
        goal.use_dock_id = True
        goal.dock_id = self.get_parameter("dock_id").value
        goal.navigate_to_staging_pose = True
        goal.max_staging_time = float(self.get_parameter("max_staging_time").value)
        self._dock_token += 1
        token = self._dock_token
        ended = threading.Event()
        answered = threading.Event()
        send = self._dock_client.send_goal_async(
            goal, feedback_callback=lambda msg, token=token: self._on_dock_feedback(msg.feedback, token)
        )
        send.add_done_callback(lambda _f: answered.set())
        if not answered.wait(10.0):
            return False, "the docking server did not answer the docking request"
        handle = send.result()
        if handle is None or not handle.accepted:
            return False, "the docking server refused the docking request"
        self._dock_goal = handle
        self._dock_ended = ended
        handle.get_result_async().add_done_callback(lambda f, token=token: self._on_dock_result(f, token, ended))
        return True, ""

    def _cancel_docking(self, wait: float = 3.0) -> None:
        """Stop a docking in progress. Its result, when it arrives, is ignored."""
        self._dock_token += 1
        handle, self._dock_goal = self._dock_goal, None
        if handle is None:
            return
        done = threading.Event()
        try:
            handle.cancel_goal_async().add_done_callback(lambda _f: done.set())
            done.wait(wait)
        except Exception as exc:  # the server may be gone; switching it off stops the docking anyway
            self.get_logger().warn(f"could not cancel the docking: {exc}")

    def _on_dock_feedback(self, feedback, token: int) -> None:
        if token != self._dock_token or self._sm.state != sm.DOCKING or self._transitioning:
            return
        reason = _DOCK_PROGRESS.get(feedback.state)
        if reason and reason != self._reason:
            self._reason = reason
            self._publish_state()

    def _on_dock_result(self, future, token: int, ended: threading.Event) -> None:
        """The docking server has finished: the robot is docked, or docking failed."""
        ended.set()  # before taking the lock: a mode change holding the lock may be waiting for this
        with self._transition_lock:
            if token != self._dock_token or self._sm.state != sm.DOCKING:
                return  # that docking was cancelled, or the robot has moved on since
            self._dock_goal = None
            try:
                wrapped = future.result()
            except Exception as exc:
                self._finish_docking(sm.IDLE, f"docking failed: no result from the docking server ({exc})")
                return
            result = wrapped.result
            if wrapped.status == GoalStatus.STATUS_SUCCEEDED and result.success:
                retries = f" after {result.num_retries} more tries" if result.num_retries else ""
                self._finish_docking(sm.DOCKED, "docked" + retries)
            else:
                why = _DOCK_ERRORS.get(result.error_code, f"error {result.error_code} from the docking server")
                self._finish_docking(sm.IDLE, f"docking failed: {why}")

    def _finish_docking(self, target: str, reason: str) -> None:
        """Move on from docking by itself: to docked, or to idle after a failure.
        Call with the transition lock held."""
        # Two separate calls: rclpy refuses to log at two different levels from one line of code.
        if target == sm.DOCKED:
            self.get_logger().info(reason)
        else:
            self.get_logger().warn(reason)
        self._transitioning = True
        self._reason = reason
        self._publish_state()
        ok, why = self._apply_mode(target)
        self._transitioning = False
        if ok:
            self._sm.set(target)
            self._entered = self.get_clock().now()
            self._faults = []
            for h in self._health.values():
                h.reset()
        else:
            self._enter_failsafe(f"change docking -> {target} failed: {why}")
        self._publish_state()

    def _start_slam(self) -> Tuple[bool, str]:
        if self._slam.state() == ACTIVE:
            self._slam_on = True
            return True, ""
        asked = time.monotonic()
        ok, why = bring_up([self._slam])
        if not ok:
            return False, why
        if not self._wait_for_map_odom(asked, 12.0):
            return False, "slam_toolbox is active but did not start publishing the map -> odom transform"
        self._slam_on = True
        return True, ""

    def _start_localization(self) -> Tuple[bool, str]:
        name = self._current_map
        if self._localized_map == name and self._amcl.state() == ACTIVE and self._map_server.state() == ACTIVE:
            return True, ""
        yaml_path = self._map_yaml(name)
        if not os.path.isfile(yaml_path):
            return False, f"map file {yaml_path} is missing"
        ok, why = bring_down(self._localization)  # a clean start: no leftovers from another map
        if not ok:
            return False, why
        if not self._map_server.set_string_parameter("yaml_filename", yaml_path):
            return False, "could not tell map_server which map to load"
        ok, why = bring_up(self._localization)
        if not ok:
            return False, why

        # Tell AMCL where the robot is: the last known map -> odom for this map, carried forward
        # through whatever the wheels did since, or the start pose if this map was never used.
        with self._data_lock:
            odom_pose = self._odom_pose
            stamp = self._odom_stamp
        if odom_pose is None:
            return False, "no odometry yet, cannot give AMCL a starting pose"
        pose = map_files.compose(self._map_odom_by_map.get(name, self._start_pose), odom_pose)
        for _ in range(6):
            asked = time.monotonic()
            self._publish_initial_pose(pose, stamp)
            if self._wait_for_map_odom(asked, 2.5):
                self._localized_map = name
                return True, ""
            with self._data_lock:
                stamp = self._odom_stamp
        return False, "AMCL is active but did not start localizing after being given a pose"

    def _publish_initial_pose(self, pose: map_files.Pose2D, stamp) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = stamp
        msg.pose.pose.position.x, msg.pose.pose.position.y = pose[0], pose[1]
        msg.pose.pose.orientation.z = math.sin(pose[2] / 2.0)
        msg.pose.pose.orientation.w = math.cos(pose[2] / 2.0)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.15**2  # 15 cm
        cov[35] = 0.1**2  # about 6 degrees
        msg.pose.covariance = cov
        self._initialpose_pub.publish(msg)

    def _start_navigation(self, exploring: bool = False) -> Tuple[bool, str]:
        """Start Nav2 for the saved map in use, with its no-go zones and the normal behaviour tree;
        or, for exploring, for the map being made: with no zones (the saved map in use is another,
        older one) and the tree that never spins or backs up."""
        name = EXPLORING_NAV if exploring else self._current_map
        if self._nav_map == name and all(n.state() == ACTIVE for n in self._navigation):
            return True, ""
        ok, why = bring_down(self._navigation)
        if not ok:
            return False, why
        try:
            if exploring:
                mask_yaml = map_files.write_empty_keepout_mask(self._maps_dir)
            else:
                mask_yaml = self._write_mask(name, map_files.read_zones(self._zones_file(name)))
        except Exception as exc:  # a bad zones file must not leave the robot half started
            return False, f"could not build the no-go zone mask: {exc}"
        if not self._filter_mask_server.set_string_parameter("yaml_filename", mask_yaml):
            return False, "could not tell filter_mask_server which mask to load"
        ok, why = self._set_tree(self._exploring_tree if exploring else self._navigation_tree, must=exploring)
        if not ok:
            return False, why
        ok, why = bring_up(self._navigation)
        if not ok:
            return False, why
        self._nav_map = name
        return True, ""

    def _set_tree(self, tree: str, must: bool) -> Tuple[bool, str]:
        """Tell bt_navigator which tree to run for goals that name none. Call with Nav2 switched
        off: the setting is read when bt_navigator is configured. It is set on every Nav2 start,
        because it stays as it was left.

        bt_navigator declares the setting the first time it is configured, with the launch file's
        value (the normal tree); before that it cannot be set. For the normal tree that changes
        nothing. For exploring (`must`) the group is configured once so that the setting exists,
        then bt_navigator alone is cleaned up and given the tree."""
        if self._bt_navigator.set_string_parameter(TREE_PARAM, tree):
            return True, ""
        if not must:
            return True, ""
        for n in self._navigation:  # in order: bt_navigator's tree needs the other servers' actions
            ok, why = n.ensure(INACTIVE)
            if not ok:
                return False, why
        if not self._bt_navigator.set_string_parameter(TREE_PARAM, tree):
            return False, "could not tell bt_navigator which behaviour tree to use"
        return self._bt_navigator.ensure(UNCONFIGURED)


def main():
    rclpy.init()
    node = ModeManager()
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

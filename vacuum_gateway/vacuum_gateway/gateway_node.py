"""The gateway between the robot and the Android app.

It translates and does nothing else: every ability the app uses is a topic, service or action of
the robot, and the gateway holds no robot state apart from the latest copy of each stream, which
it gives to a phone that connects late. The robot works the same whether or not it is running.

    /robot_state     vacuum_interfaces/msg/RobotState    -> RobotState     (on change, 2 a second)
    /safety_status   vacuum_interfaces/msg/SafetyStatus  -> SafetyStatus   (on change, otherwise 2 a second)
    /map             nav_msgs/msg/OccupancyGrid          -> Map            (on change, at most 1 a second)
    TF map -> base_link                                  -> RobotPose      (10 a second, when TF has news)
    /plan            nav_msgs/msg/Path                   -> Route          (when Nav2 plans, at most 2 a second)
    navigate_to_pose status + feedback                   -> NavigationStatus (the newest goal, whoever sent it)
    the zones file of the map in use (see zones.py)      -> NoGoZones      (when the map or the file changes)
    ChangeMode       -> /change_state (vacuum_interfaces/srv/ChangeState) -> Reply
    SaveMap          -> /save_map (vacuum_interfaces/srv/SaveMap)        -> Reply
    SetNoGoZones     -> /set_no_go_zones (vacuum_interfaces/srv/SetNoGoZones) -> Reply; once saved,
                        every phone gets the new NoGoZones at once
    GoTo             -> compute_path_to_pose (is there a route?), then
                        navigate_to_pose (nav2_msgs/action/NavigateToPose)  -> Reply with the goal's id
    CancelGoal       -> navigate_to_pose/_action/cancel_goal               -> Reply
    Drive            -> /cmd_vel (geometry_msgs/msg/Twist), one phone at a time -> Reply at once
    /cleaning/status   vacuum_interfaces/msg/CleaningStatus -> CleaningStatus (on change, 2 a second)
    /cleaning/plan     vacuum_interfaces/msg/CleaningPlan   -> CleaningPlan   (on change)
    /cleaning/cleaned  nav_msgs/msg/OccupancyGrid           -> CleanedArea    (at most 1 a second)
    StartCleaning    -> /cleaning/start (vacuum_interfaces/srv/StartCleaning): Reply once the robot has
                        checked and planned the job, with its number, or with why not
    PauseCleaning    -> /cleaning/pause or /cleaning/resume (std_srvs/srv/Trigger)
    PreviewCleaning, SaveCleaningPlan, ListCleaningPlans, DeleteCleaningPlan, ListCleaningHistory,
    GetCleaningRecord -> the cleaning node's services of the same names, under /cleaning/
    /exploration/status    vacuum_interfaces/msg/ExplorationStatus -> ExplorationStatus (on change, 2 a second)
    /exploration/frontiers geometry_msgs/msg/PoseArray             -> Frontiers         (on change, at most 2 a second)
    StartExploration -> /exploration/start (vacuum_interfaces/srv/StartExploration): Reply at once, with
                        the exploration's number, or with why not. Stop is ChangeMode to idle.

Phones connect to ws://<this PC>:9090. The messages are in proto/vacuum.proto.

Two threads: the ROS executor thread runs subscriptions and every service call; the asyncio
event loop in the main thread runs the WebSocket server. Messages and results cross between
them through loop.call_soon_threadsafe and executor.create_task, both safe from any thread.
The one exception is publishing the Drive speeds, done straight from the event loop: rclpy's
publish() may be called from any thread, and a hop through the executor would only add delay.
"""

import asyncio
import signal
import sys
import threading

import math

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Point32, PoseArray, Polygon, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from vacuum_interfaces.msg import CleaningJob, CleaningPlan, CleaningStatus, ExplorationStatus, NoGoZone, RobotState, SafetyStatus
from vacuum_interfaces.srv import (ChangeState, DeleteCleaningPlan, GetCleaningRecord, ListCleaningHistory, ListCleaningPlans,
                                   PreviewCleaning, SaveCleaningPlan, SaveMap, SetNoGoZones, StartCleaning, StartExploration)

# Imported before anything generated: say clearly what is wrong if the pip packages are missing.
from . import requirements

requirements.check_or_exit()

from . import __version__, convert, drive, geometry, navigation, zones  # noqa: E402
from . import vacuum_pb2 as pb  # noqa: E402
from .server import Command, GatewayServer  # noqa: E402
from .streams import Stream  # noqa: E402

Result = pb.Reply.Result

# Longer than the slowest mode change seen in failure cases (about 100 s) plus the up to 120 s a
# stop waits for a change already in progress (mode manager, _STOP_WAIT_SECONDS).
CHANGE_MODE_TIMEOUT = 240.0
# The route check takes well under a second on this house; Nav2 then accepts or refuses at once.
# It is the driving that takes time, and the Reply does not wait for it.
ROUTE_CHECK_TIMEOUT = 5.0
GO_TO_TIMEOUT = 12.0
CANCEL_TIMEOUT = 10.0
DRIVE_TIMEOUT = 2.0  # publishing does not wait for anything
SAVE_MAP_TIMEOUT = 60.0  # the mode manager waits up to 8 s for a fresh map, then up to 15 s for slam_toolbox
# Writing the zones and the mask takes well under a second; the mask server then has up to 5 s to load it.
SET_ZONES_TIMEOUT = 20.0
# The robot answers a start once it has checked and planned the job (well under a second of
# planning), before it switches modes.
START_CLEANING_TIMEOUT = 20.0
CLEANING_CALL_TIMEOUT = 20.0  # preview, save, list, delete, history, one record: planning is the slowest, under 1 s
START_EXPLORATION_TIMEOUT = 10.0  # the exploration node answers at once, before the robot changes mode


POSE_PERIOD = 0.1  # s between robot positions (computer's clock)
ZONES_CHECK_PERIOD = 2.0  # s between looks at the zones file of the map in use


def make_streams():
    """The streams the phones receive, and their timing (seconds).

    The robot state arrives twice a second; repeating at 0.4 s lets every copy through even
    when they arrive a little early. The safety status arrives ten times a second; unchanged
    copies go out twice a second. A copy older than 2 s is not given to a phone that connects
    late: by then the node that publishes it has stopped, and the phone must not show it as live.

    The map goes out when it changes, at most once a second, and is never repeated; a phone that
    connects late always gets the current one. Positions all differ (each carries its
    stamp), so every one goes out; a late phone gets one only if under 1 s old.
    """
    return {
        pb.DATA_ROBOT_STATE: Stream("the robot state", repeat_interval=0.4, replay_max_age=2.0),
        pb.DATA_SAFETY_STATUS: Stream("the safety status", repeat_interval=0.5, replay_max_age=2.0),
        pb.DATA_MAP: Stream("the map", repeat_interval=None, replay_max_age=None, min_interval=1.0),
        pb.DATA_ROBOT_POSE: Stream("the robot's position", repeat_interval=0.0, replay_max_age=1.0),
        # The route goes out when Nav2 plans; the navigation status on every change, at most
        # twice a second, and again every 2 s. A late phone gets both: the app shows them only
        # while the robot is navigating.
        pb.DATA_ROUTE: Stream("the route", repeat_interval=None, replay_max_age=None, min_interval=0.5),
        pb.DATA_NAVIGATION: Stream("the navigation status", repeat_interval=2.0, replay_max_age=None, min_interval=0.5),
        # The no-go zones go out when the map in use or its zones file changes; a late phone always gets them.
        pb.DATA_NO_GO_ZONES: Stream("the no-go zones", repeat_interval=None, replay_max_age=None),
        # Cleaning: the status like the robot state (the cleaning node publishes it twice a second,
        # job or not); the rows on change; the cleaned area at most once a second. A late phone gets
        # all three, so a phone that connects in the middle of a clean sees it at once.
        pb.DATA_CLEANING_STATUS: Stream("the cleaning status", repeat_interval=0.4, replay_max_age=2.0),
        pb.DATA_CLEANING_PLAN: Stream("the cleaning plan", repeat_interval=None, replay_max_age=None, min_interval=0.5),
        pb.DATA_CLEANED_AREA: Stream("the cleaned area", repeat_interval=None, replay_max_age=None, min_interval=1.0),
        # Exploring: the status like the cleaning status; the places still to look at on change, at
        # most twice a second. A late phone gets both, so it sees an exploration under way at once.
        pb.DATA_EXPLORATION_STATUS: Stream("the exploration status", repeat_interval=0.4, replay_max_age=2.0),
        pb.DATA_FRONTIERS: Stream("the places still to look at", repeat_interval=None, replay_max_age=None, min_interval=0.5),
    }


def ros_call(executor, client, request, loop) -> asyncio.Future:
    """Call a ROS service from the event loop. The call itself runs on the ROS executor's thread,
    the only thread that touches ROS; the result comes back to the event loop as a future."""
    return ros_run(executor, lambda: client.call_async(request), loop)


def ros_run(executor, start, loop) -> asyncio.Future:
    """Run `start()` on the ROS executor's thread and await what it returns (a service call, a
    goal sent to an action). The result comes back to the event loop as a future."""
    answer = loop.create_future()

    def deliver(response, error):
        if not answer.done():  # the caller may have given up (time limit) meanwhile
            if error is not None:
                answer.set_exception(error)
            else:
                answer.set_result(response)

    async def call():
        try:
            response = await start()
        except Exception as exc:
            loop.call_soon_threadsafe(deliver, None, exc)
        else:
            loop.call_soon_threadsafe(deliver, response, None)

    executor.create_task(call)
    return answer


def make_commands(node, executor, loop, driver: drive.DriverLock):
    """The commands phones can send, besides ping (which the server answers itself)."""

    async def change_mode(message):
        if not message.mode.strip():
            return Result.RESULT_BAD_REQUEST, "no mode given"
        if not node.change_state.service_is_ready():
            return Result.RESULT_NOT_AVAILABLE, "the mode manager is not running (no /change_state service)"
        response = await ros_call(executor, node.change_state, ChangeState.Request(new_state=message.mode), loop)
        return (Result.RESULT_OK if response.success else Result.RESULT_REFUSED), response.message

    async def save_map(message):
        if not node.save_map.service_is_ready():
            return Result.RESULT_NOT_AVAILABLE, "the mode manager is not running (no /save_map service)"
        request = SaveMap.Request(name=message.name.strip(), replace=message.replace)
        response = await ros_call(executor, node.save_map, request, loop)
        if response.name_taken:
            return Result.RESULT_ALREADY_EXISTS, response.message
        return (Result.RESULT_OK if response.success else Result.RESULT_REFUSED), response.message

    async def save_zones(map_name, zone_list):
        if not node.set_zones.service_is_ready():
            return None
        request = SetNoGoZones.Request(map_name=map_name.strip())
        request.zones = [
            NoGoZone(name=name, polygon=Polygon(points=[Point32(x=x, y=y, z=0.0) for x, y in corners]))
            for name, corners in zone_list
        ]
        response = await ros_call(executor, node.set_zones, request, loop)
        return response.success, response.message

    async def send_zones_now():
        async def on_ros_thread():  # the zones are read and sent from the ROS thread, as on every look
            node.send_zones(force=True)

        await ros_run(executor, on_ros_thread, loop)

    async def route_exists(pose) -> bool:
        """Ask Nav2's planner for a route from the robot to `pose`. It plans on the same costmap
        as navigation: walls, unexplored space and no-go zones block a route."""
        check = ComputePathToPose.Goal(goal=pose, use_start=False)  # start: where the robot is

        async def plan():
            handle = await node.plan_route.send_goal_async(check)
            if not handle.accepted:
                return False
            outcome = await handle.get_result_async()
            return outcome.status == GoalStatus.STATUS_SUCCEEDED and len(outcome.result.path.poses) > 0

        return await asyncio.wait_for(ros_run(executor, plan, loop), ROUTE_CHECK_TIMEOUT)

    async def go_to(message):
        if not all(math.isfinite(v) for v in (message.x, message.y, message.heading)):
            return Result.RESULT_BAD_REQUEST, "the goal is not a number"
        # Nav2 runs only in navigation mode (and while docking or cleaning); otherwise its action is absent.
        if not node.navigate.server_is_ready():
            return Result.RESULT_NOT_AVAILABLE, "navigation is not running: switch the robot to navigation first"
        if node.mode != "navigation":
            # Nav2 runs for a clean or a docking: a goal of the phone's would fight it (the cleaning
            # node would cancel it anyway).
            return Result.RESULT_REFUSED, f"Go to works in navigation mode, and the robot is {node.mode or 'starting up'}"
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"  # stamp left at zero: Nav2 uses the newest transform
        goal.pose.pose.position.x, goal.pose.pose.position.y = message.x, message.y
        goal.pose.pose.orientation.z = math.sin(message.heading / 2.0)
        goal.pose.pose.orientation.w = math.cos(message.heading / 2.0)
        # Without a route, Nav2 would accept the goal, try its recovery moves for about 20 s and
        # then give up without a reason. Asking first gives the user an answer at once.
        if node.plan_route.server_is_ready():
            try:
                if not await route_exists(goal.pose):
                    return Result.RESULT_REFUSED, "no route to that spot: it is blocked, unexplored or in a no-go zone"
            except asyncio.TimeoutError:
                node.get_logger().warning("the route check took too long; sending the goal without it")
        handle = await ros_run(executor, lambda: node.navigate.send_goal_async(goal), loop)
        if not handle.accepted:
            return Result.RESULT_REFUSED, "navigation refused the goal"
        return Result.RESULT_OK, "on the way", {"goal_id": bytes(bytearray(handle.goal_id.uuid))}

    async def cancel_goal(message):
        if len(message.goal_id) not in (0, 16):
            return Result.RESULT_BAD_REQUEST, "a goal id has 16 bytes"
        if not node.cancel_nav.service_is_ready():
            return Result.RESULT_NOT_AVAILABLE, "navigation is not running"
        request = CancelGoal.Request()  # an all-zero id and stamp mean "every goal"
        if message.goal_id:
            request.goal_info.goal_id.uuid = list(message.goal_id)
        response = await ros_call(executor, node.cancel_nav, request, loop)
        code = response.return_code
        if code == CancelGoal.Response.ERROR_NONE:
            return Result.RESULT_OK, "cancelling"
        if code == CancelGoal.Response.ERROR_GOAL_TERMINATED:
            return Result.RESULT_REFUSED, "that trip had already ended"
        if code == CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID:
            return Result.RESULT_REFUSED, "navigation does not know that trip"
        return Result.RESULT_REFUSED, "navigation refused to cancel"

    async def drive_robot(message, phone):
        speeds = drive.clamp_speeds(message.forward, message.turn)
        if speeds is None:
            return Result.RESULT_BAD_REQUEST, "the speeds are not numbers"
        if not driver.claim(phone, loop.time()):
            return Result.RESULT_REFUSED, "another phone is driving the robot"
        node.publish_drive(*speeds)
        return Result.RESULT_OK, ""

    async def start_exploration(message):
        if not math.isfinite(message.time_limit) or message.time_limit < 0:
            return Result.RESULT_BAD_REQUEST, "the time limit is not a number of seconds"
        if not node.start_exploration.service_is_ready():
            return Result.RESULT_NOT_AVAILABLE, "the exploration node is not running (no /exploration/start service)"
        response = await ros_call(executor, node.start_exploration, StartExploration.Request(time_limit=message.time_limit), loop)
        if not response.accepted:
            return Result.RESULT_REFUSED, response.message
        return Result.RESULT_OK, response.message, {"job_id": response.job_id}

    cleaning = make_cleaning_commands(node, executor, loop)
    return {
        **cleaning,
        pb.COMMAND_START_EXPLORATION: Command("start exploration", pb.StartExploration, START_EXPLORATION_TIMEOUT, start_exploration),
        pb.COMMAND_CHANGE_MODE: Command("change mode", pb.ChangeMode, CHANGE_MODE_TIMEOUT, change_mode),
        pb.COMMAND_SAVE_MAP: Command("save map", pb.SaveMap, SAVE_MAP_TIMEOUT, save_map),
        pb.COMMAND_SET_NO_GO_ZONES: Command("set no-go zones", pb.SetNoGoZones, SET_ZONES_TIMEOUT,
                                            zones.set_zones_command(save_zones, send_zones_now)),
        pb.COMMAND_DRIVE: Command("drive", pb.Drive, DRIVE_TIMEOUT, drive_robot, with_phone=True),
        pb.COMMAND_GO_TO: Command("go to", pb.GoTo, GO_TO_TIMEOUT, go_to),
        pb.COMMAND_CANCEL_GOAL: Command("cancel goal", pb.CancelGoal, CANCEL_TIMEOUT, cancel_goal),
    }


def make_cleaning_commands(node, executor, loop):
    """The cleaning commands. The robot's cleaning node does all the work; these only pass requests on."""
    not_running = "the cleaning node is not running"

    def ros_job(message) -> CleaningJob:
        job = CleaningJob()
        for name, value in convert.cleaning_job_fields(message).items():
            setattr(job, name, value)
        return job

    async def call(client, request):
        if not client.service_is_ready():
            return None
        return await ros_call(executor, client, request, loop)

    async def start_cleaning(message):
        problem = convert.cleaning_job_problem(message.job)
        if problem:
            return Result.RESULT_BAD_REQUEST, problem
        response = await call(node.start_cleaning, StartCleaning.Request(job=ros_job(message.job)))
        if response is None:
            return Result.RESULT_NOT_AVAILABLE, f"{not_running} (no /cleaning/start service)"
        if not response.accepted:
            return Result.RESULT_REFUSED, response.message
        return Result.RESULT_OK, response.message, {"job_id": response.job_id, "estimate": convert.cleaning_estimate(response.estimate)}

    async def pause_cleaning(message):
        client = node.resume_cleaning if message.resume else node.pause_cleaning
        response = await call(client, Trigger.Request())
        if response is None:
            return Result.RESULT_NOT_AVAILABLE, not_running
        return (Result.RESULT_OK if response.success else Result.RESULT_REFUSED), response.message

    async def preview(message):
        problem = convert.cleaning_job_problem(message.job)
        if problem:
            return Result.RESULT_BAD_REQUEST, problem
        response = await call(node.preview_cleaning, PreviewCleaning.Request(job=ros_job(message.job)))
        if response is None:
            return Result.RESULT_NOT_AVAILABLE, not_running
        if not response.success:
            return Result.RESULT_REFUSED, response.message
        return Result.RESULT_OK, response.message, {
            "estimate": convert.cleaning_estimate(response.estimate), "replanned": response.replanned}

    async def save_plan(message):
        problem = convert.cleaning_job_problem(message.job)
        if problem:
            return Result.RESULT_BAD_REQUEST, problem
        request = SaveCleaningPlan.Request(name=message.name.strip(), job=ros_job(message.job), replace=message.replace)
        response = await call(node.save_cleaning_plan, request)
        if response is None:
            return Result.RESULT_NOT_AVAILABLE, not_running
        if response.name_taken:
            return Result.RESULT_ALREADY_EXISTS, response.message
        if not response.success:
            return Result.RESULT_REFUSED, response.message
        return Result.RESULT_OK, response.message, {"estimate": convert.cleaning_estimate(response.estimate)}

    async def list_plans(_message):
        response = await call(node.list_cleaning_plans, ListCleaningPlans.Request())
        if response is None:
            return Result.RESULT_NOT_AVAILABLE, not_running
        return Result.RESULT_OK, "", {"plans": [convert.cleaning_plan_info(p) for p in response.plans]}

    async def delete_plan(message):
        response = await call(node.delete_cleaning_plan, DeleteCleaningPlan.Request(name=message.name))
        if response is None:
            return Result.RESULT_NOT_AVAILABLE, not_running
        return (Result.RESULT_OK if response.success else Result.RESULT_REFUSED), response.message

    async def history(message):
        response = await call(node.cleaning_history, ListCleaningHistory.Request(max_records=message.max_records))
        if response is None:
            return Result.RESULT_NOT_AVAILABLE, not_running
        return Result.RESULT_OK, "", {"history": convert.cleaning_history(response.records, response.totals)}

    async def get_record(message):
        response = await call(node.cleaning_record, GetCleaningRecord.Request(id=message.id))
        if response is None:
            return Result.RESULT_NOT_AVAILABLE, not_running
        if not response.success:
            return Result.RESULT_REFUSED, response.message
        return Result.RESULT_OK, "", {"record": convert.cleaning_record(response.record, bytes(response.picture))}

    return {
        pb.COMMAND_START_CLEANING: Command("start cleaning", pb.StartCleaning, START_CLEANING_TIMEOUT, start_cleaning),
        pb.COMMAND_PAUSE_CLEANING: Command("pause cleaning", pb.PauseCleaning, CLEANING_CALL_TIMEOUT, pause_cleaning),
        pb.COMMAND_PREVIEW_CLEANING: Command("preview cleaning", pb.PreviewCleaning, CLEANING_CALL_TIMEOUT, preview),
        pb.COMMAND_SAVE_CLEANING_PLAN: Command("save cleaning plan", pb.SaveCleaningPlan, CLEANING_CALL_TIMEOUT, save_plan),
        pb.COMMAND_LIST_CLEANING_PLANS: Command("list cleaning plans", pb.ListCleaningPlans, CLEANING_CALL_TIMEOUT, list_plans),
        pb.COMMAND_DELETE_CLEANING_PLAN: Command("delete cleaning plan", pb.DeleteCleaningPlan, CLEANING_CALL_TIMEOUT, delete_plan),
        pb.COMMAND_LIST_CLEANING_HISTORY: Command("list cleaning history", pb.ListCleaningHistory, CLEANING_CALL_TIMEOUT, history),
        pb.COMMAND_GET_CLEANING_RECORD: Command("get cleaning record", pb.GetCleaningRecord, CLEANING_CALL_TIMEOUT, get_record),
    }


class RobotSide(Node):
    """Subscribes to the robot and hands each message, converted, to `forward(type, bytes)`.
    Holds the service clients the commands use."""

    def __init__(self):
        super().__init__("vacuum_gateway")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 9090)
        self.declare_parameter("robot_name", "MOTEbot")
        self.declare_parameter("heartbeat_interval", 1.0)
        self.declare_parameter("max_message_bytes", 4 * 1024 * 1024)
        self.declare_parameter("maps_dir", "")  # where the saved maps and their zones files are
        self.forward = None
        self.mode = ""  # the robot's mode, from /robot_state
        self._heard = set()
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(RobotState, "robot_state", self._on_robot_state, latched)
        self.create_subscription(SafetyStatus, "safety_status", self._on_safety_status, 10)
        self.create_subscription(OccupancyGrid, "map", self._on_map, latched)
        self.change_state = self.create_client(ChangeState, "change_state")
        self.save_map = self.create_client(SaveMap, "save_map")
        self.set_zones = self.create_client(SetNoGoZones, "set_no_go_zones")
        # Driving by hand: the same input the keyboard uses. The safety gate decides whether it moves the robot.
        self._drive_pub = self.create_publisher(Twist, "cmd_vel", 1)
        # Navigation: any goal's progress (status, feedback, route), and the means to send and cancel.
        self.navigate = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.plan_route = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.cancel_nav = self.create_client(CancelGoal, "navigate_to_pose/_action/cancel_goal")
        self._nav = navigation.NavigationTracker()
        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(GoalStatusArray, "navigate_to_pose/_action/status", self._on_nav_status, status_qos)
        self.create_subscription(NavigateToPose.Impl.FeedbackMessage, "navigate_to_pose/_action/feedback", self._on_nav_feedback, 10)
        self.create_subscription(Path, "plan", self._on_plan, 1)
        # Cleaning: the cleaning node's reports, and the means to ask it things.
        self._cleaning_job = 0
        self.create_subscription(CleaningStatus, "cleaning/status", self._on_cleaning_status, latched)
        self.create_subscription(CleaningPlan, "cleaning/plan", self._on_cleaning_plan, latched)
        self.create_subscription(OccupancyGrid, "cleaning/cleaned", self._on_cleaned, latched)
        self.start_cleaning = self.create_client(StartCleaning, "cleaning/start")
        self.pause_cleaning = self.create_client(Trigger, "cleaning/pause")
        self.resume_cleaning = self.create_client(Trigger, "cleaning/resume")
        self.preview_cleaning = self.create_client(PreviewCleaning, "cleaning/preview")
        self.save_cleaning_plan = self.create_client(SaveCleaningPlan, "cleaning/save_plan")
        self.list_cleaning_plans = self.create_client(ListCleaningPlans, "cleaning/list_plans")
        self.delete_cleaning_plan = self.create_client(DeleteCleaningPlan, "cleaning/delete_plan")
        self.cleaning_history = self.create_client(ListCleaningHistory, "cleaning/history")
        self.cleaning_record = self.create_client(GetCleaningRecord, "cleaning/get_record")
        # Exploring: the exploration node's reports, and the means to start one.
        self._exploration_job = 0
        self.create_subscription(ExplorationStatus, "exploration/status", self._on_exploration_status, latched)
        self.create_subscription(PoseArray, "exploration/frontiers", self._on_frontiers, latched)
        self.start_exploration = self.create_client(StartExploration, "exploration/start")

        self.tf = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self.tf, self)
        self._map_content = None
        self._map_version = 0
        self._pose_stamp = 0.0
        # A steady clock: the simulator's clock stops when it freezes, and so would this timer.
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(POSE_PERIOD, self._pose_tick, clock=steady)
        # The zones of the map in use, which /robot_state names.
        self._map_zones = zones.MapZones(self.param("maps_dir")) if self.param("maps_dir") else None
        if self._map_zones is None:
            self.get_logger().error("parameter maps_dir is not set: the phones will not get the no-go zones")
        self._zones = None  # the latest converted zones, sent again once the app side is ready
        self._zones_error = None
        self.create_timer(ZONES_CHECK_PERIOD, self.send_zones, clock=steady)

    def param(self, name):
        return self.get_parameter(name).value

    def publish_drive(self, forward: float, turn: float) -> None:
        msg = Twist()
        msg.linear.x, msg.angular.z = forward, turn
        self._drive_pub.publish(msg)

    def _first(self, topic):
        """Log when a topic is heard for the first time. After a restart that takes a few
        seconds (ROS has to discover this node), and phones see "waiting for the robot" until then."""
        if topic not in self._heard:
            self._heard.add(topic)
            self.get_logger().info(f"receiving /{topic}")

    def _on_robot_state(self, msg):
        self._first("robot_state")
        self.mode = msg.state
        if self.forward is not None:
            self.forward(pb.DATA_ROBOT_STATE, convert.robot_state(msg).SerializeToString())
        # While a map is being made (mapping, exploring), it is not a saved map, whatever
        # current_map still names.
        in_use = "" if msg.making_map else msg.current_map
        if self._map_zones is not None and self._map_zones.follow(in_use):
            self.send_zones()  # the new map's zones at once, not at the next look

    def _on_safety_status(self, msg):
        self._first("safety_status")
        if self.forward is not None:
            self.forward(pb.DATA_SAFETY_STATUS, convert.safety_status(msg).SerializeToString())

    def _on_map(self, msg):
        self._first("map")
        content = convert.map_content(msg)
        if content != self._map_content:
            self._map_content = content
            self._map_version += 1
        if self.forward is not None:
            self.forward(pb.DATA_MAP, convert.map_message(msg, self._map_version).SerializeToString())

    def _on_cleaning_status(self, msg):
        self._first("cleaning/status")
        self._cleaning_job = msg.job_id
        if self.forward is not None:
            self.forward(pb.DATA_CLEANING_STATUS, convert.cleaning_status(msg).SerializeToString())

    def _on_exploration_status(self, msg):
        self._first("exploration/status")
        self._exploration_job = msg.job_id
        if self.forward is not None:
            self.forward(pb.DATA_EXPLORATION_STATUS, convert.exploration_status(msg).SerializeToString())

    def _on_frontiers(self, msg):
        if msg.header.frame_id not in ("map", ""):
            self._first(f"exploration/frontiers in frame '{msg.header.frame_id}', which the app cannot place")
            return
        job = self._exploration_job if msg.poses else 0
        if self.forward is not None:
            self.forward(pb.DATA_FRONTIERS, convert.frontiers(msg, job).SerializeToString())

    def _on_cleaning_plan(self, msg):
        if msg.header.frame_id not in ("map", ""):
            self._first(f"cleaning/plan in frame '{msg.header.frame_id}', which the app cannot place")
            return
        if self.forward is not None:
            self.forward(pb.DATA_CLEANING_PLAN, convert.cleaning_plan(msg).SerializeToString())

    def _on_cleaned(self, msg):
        if self.forward is not None:
            self.forward(pb.DATA_CLEANED_AREA, convert.cleaned_area(msg, self._cleaning_job).SerializeToString())

    def _forward_navigation(self):
        message = self._nav.message()
        if message is not None and self.forward is not None:
            self.forward(pb.DATA_NAVIGATION, message.SerializeToString())

    def _on_nav_status(self, msg):
        if self._nav.on_status(convert.goal_statuses(msg)):
            self._forward_navigation()

    def _on_nav_feedback(self, msg):
        if self._nav.on_feedback(bytes(bytearray(msg.goal_id.uuid)), msg.feedback.distance_remaining):
            self._forward_navigation()

    def _on_plan(self, msg):
        if self.forward is None or not msg.poses:
            return
        if msg.header.frame_id not in ("map", ""):
            self._first(f"plan in frame '{msg.header.frame_id}', which the app cannot place")
            return
        points = navigation.thin_path([(p.pose.position.x, p.pose.position.y) for p in msg.poses])
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.forward(pb.DATA_ROUTE, convert.route(stamp, points).SerializeToString())

    def send_zones(self, force: bool = False):
        """Send the no-go zones of the map in use when they have changed (and the first time). With
        `force`, the file is read again even if it looks unchanged. Runs on the ROS thread."""
        if self._map_zones is None:
            return
        try:
            found = self._map_zones.changed(force)
        except (OSError, ValueError) as exc:
            if str(exc) != self._zones_error:
                self._zones_error = str(exc)
                self.get_logger().error(f"cannot read the no-go zones of the map '{self._map_zones.map_name}': {exc}")
            return
        if found is not None:
            self._zones_error = None
            map_name, zone_list = found
            self._zones = convert.no_go_zones(zone_list, map_name).SerializeToString()
            names = ", ".join(name for name, _ in zone_list) or "none"
            self.get_logger().info(f"no-go zones of {repr(map_name) if map_name else 'no saved map'}: {names}")
        if self._zones is not None and self.forward is not None:
            self.forward(pb.DATA_NO_GO_ZONES, self._zones)  # unchanged copies are skipped by the stream

    def _pose_tick(self):
        """Where the robot is on the map, if TF has news since the last time. When nothing places
        the robot on a map (no map, or localization off), TF stops advancing and nothing is sent."""
        try:
            t = self.tf.lookup_transform("map", "base_link", Time())
        except TransformException:
            return
        stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
        if stamp <= self._pose_stamp or self.forward is None:
            return
        self._pose_stamp = stamp
        tr, q = t.transform.translation, t.transform.rotation
        heading = geometry.yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.forward(pb.DATA_ROBOT_POSE, convert.robot_pose(stamp, tr.x, tr.y, heading).SerializeToString())


def main():
    # The asyncio loop handles Ctrl-C and the launch system's SIGTERM, then shuts ROS down itself.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = RobotSide()
    log = node.get_logger()
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    driver = drive.DriverLock()

    def phone_left(phone):
        # Courtesy only: the safety gate stops the robot 0.6 s after the last command anyway.
        if driver.release(phone):
            node.publish_drive(0.0, 0.0)
            log.info(f"phone {phone} left while driving: sent zero speed")

    server = GatewayServer(
        robot_name=node.param("robot_name"),
        gateway_version=__version__,
        streams=make_streams(),
        commands=make_commands(node, executor, loop, driver),
        on_phone_left=phone_left,
        logger=log,
        host=node.param("host"),
        port=node.param("port"),
        heartbeat_interval=node.param("heartbeat_interval"),
        max_payload=node.param("max_message_bytes"),
    )

    def forward(msg_type, payload):  # called on the ROS thread
        try:
            loop.call_soon_threadsafe(server.publish, msg_type, payload)
        except RuntimeError:
            pass  # the loop has closed: shutting down

    stop = asyncio.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set)

    exit_code = 0
    try:
        port = loop.run_until_complete(server.start())
    except OSError as exc:
        # Most likely another gateway still has the port. The launch file starts this one again.
        log.error(f"cannot listen on {node.param('host')}:{node.param('port')}: {exc}")
        exit_code = 1
    else:
        node.forward = forward

        def spin():
            try:
                executor.spin()
            except ExternalShutdownException:
                pass

        threading.Thread(target=spin, daemon=True).start()
        log.info(f"listening for the app on ws://{node.param('host')}:{port}")
        loop.run_until_complete(stop.wait())
        log.info("stopping")
        loop.run_until_complete(server.stop())
    finally:
        node.forward = None
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        loop.close()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()

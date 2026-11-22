from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from collections.abc import Iterable as _Iterable, Mapping as _Mapping
from typing import ClassVar as _ClassVar, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class Protocol(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    PROTOCOL_UNSPECIFIED: _ClassVar[Protocol]
    PROTOCOL_VERSION: _ClassVar[Protocol]

class Channel(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    CHANNEL_COMMAND: _ClassVar[Channel]
    CHANNEL_DATA: _ClassVar[Channel]

class CommandType(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    COMMAND_UNSPECIFIED: _ClassVar[CommandType]
    COMMAND_PING: _ClassVar[CommandType]
    COMMAND_CHANGE_MODE: _ClassVar[CommandType]
    COMMAND_SAVE_MAP: _ClassVar[CommandType]
    COMMAND_SET_NO_GO_ZONES: _ClassVar[CommandType]
    COMMAND_SET_ROBOT_SETTINGS: _ClassVar[CommandType]
    COMMAND_FIND_ROOMS: _ClassVar[CommandType]
    COMMAND_SET_ROOMS: _ClassVar[CommandType]
    COMMAND_GO_TO: _ClassVar[CommandType]
    COMMAND_CANCEL_GOAL: _ClassVar[CommandType]
    COMMAND_DRIVE: _ClassVar[CommandType]
    COMMAND_START_EXPLORATION: _ClassVar[CommandType]
    COMMAND_START_CLEANING: _ClassVar[CommandType]
    COMMAND_PAUSE_CLEANING: _ClassVar[CommandType]
    COMMAND_PREVIEW_CLEANING: _ClassVar[CommandType]
    COMMAND_SAVE_CLEANING_PLAN: _ClassVar[CommandType]
    COMMAND_LIST_CLEANING_PLANS: _ClassVar[CommandType]
    COMMAND_DELETE_CLEANING_PLAN: _ClassVar[CommandType]
    COMMAND_LIST_CLEANING_HISTORY: _ClassVar[CommandType]
    COMMAND_GET_CLEANING_RECORD: _ClassVar[CommandType]
    COMMAND_ASK_ASSISTANT: _ClassVar[CommandType]
    COMMAND_NAME_ROOMS: _ClassVar[CommandType]

class DataType(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    DATA_UNSPECIFIED: _ClassVar[DataType]
    DATA_HELLO: _ClassVar[DataType]
    DATA_HEARTBEAT: _ClassVar[DataType]
    DATA_ROBOT_STATE: _ClassVar[DataType]
    DATA_SAFETY_STATUS: _ClassVar[DataType]
    DATA_ROBOT_SETTINGS: _ClassVar[DataType]
    DATA_ROOMS: _ClassVar[DataType]
    DATA_MAP: _ClassVar[DataType]
    DATA_ROBOT_POSE: _ClassVar[DataType]
    DATA_ROUTE: _ClassVar[DataType]
    DATA_NAVIGATION: _ClassVar[DataType]
    DATA_NO_GO_ZONES: _ClassVar[DataType]
    DATA_EXPLORATION_STATUS: _ClassVar[DataType]
    DATA_FRONTIERS: _ClassVar[DataType]
    DATA_CLEANING_STATUS: _ClassVar[DataType]
    DATA_CLEANING_PLAN: _ClassVar[DataType]
    DATA_CLEANED_AREA: _ClassVar[DataType]
    DATA_ASSISTANT_STATUS: _ClassVar[DataType]
    DATA_REPLY: _ClassVar[DataType]
PROTOCOL_UNSPECIFIED: Protocol
PROTOCOL_VERSION: Protocol
CHANNEL_COMMAND: Channel
CHANNEL_DATA: Channel
COMMAND_UNSPECIFIED: CommandType
COMMAND_PING: CommandType
COMMAND_CHANGE_MODE: CommandType
COMMAND_SAVE_MAP: CommandType
COMMAND_SET_NO_GO_ZONES: CommandType
COMMAND_SET_ROBOT_SETTINGS: CommandType
COMMAND_FIND_ROOMS: CommandType
COMMAND_SET_ROOMS: CommandType
COMMAND_GO_TO: CommandType
COMMAND_CANCEL_GOAL: CommandType
COMMAND_DRIVE: CommandType
COMMAND_START_EXPLORATION: CommandType
COMMAND_START_CLEANING: CommandType
COMMAND_PAUSE_CLEANING: CommandType
COMMAND_PREVIEW_CLEANING: CommandType
COMMAND_SAVE_CLEANING_PLAN: CommandType
COMMAND_LIST_CLEANING_PLANS: CommandType
COMMAND_DELETE_CLEANING_PLAN: CommandType
COMMAND_LIST_CLEANING_HISTORY: CommandType
COMMAND_GET_CLEANING_RECORD: CommandType
COMMAND_ASK_ASSISTANT: CommandType
COMMAND_NAME_ROOMS: CommandType
DATA_UNSPECIFIED: DataType
DATA_HELLO: DataType
DATA_HEARTBEAT: DataType
DATA_ROBOT_STATE: DataType
DATA_SAFETY_STATUS: DataType
DATA_ROBOT_SETTINGS: DataType
DATA_ROOMS: DataType
DATA_MAP: DataType
DATA_ROBOT_POSE: DataType
DATA_ROUTE: DataType
DATA_NAVIGATION: DataType
DATA_NO_GO_ZONES: DataType
DATA_EXPLORATION_STATUS: DataType
DATA_FRONTIERS: DataType
DATA_CLEANING_STATUS: DataType
DATA_CLEANING_PLAN: DataType
DATA_CLEANED_AREA: DataType
DATA_ASSISTANT_STATUS: DataType
DATA_REPLY: DataType

class Hello(_message.Message):
    __slots__ = ("protocol_version", "robot_name", "gateway_version")
    PROTOCOL_VERSION_FIELD_NUMBER: _ClassVar[int]
    ROBOT_NAME_FIELD_NUMBER: _ClassVar[int]
    GATEWAY_VERSION_FIELD_NUMBER: _ClassVar[int]
    protocol_version: int
    robot_name: str
    gateway_version: str
    def __init__(self, protocol_version: _Optional[int] = ..., robot_name: _Optional[str] = ..., gateway_version: _Optional[str] = ...) -> None: ...

class Heartbeat(_message.Message):
    __slots__ = ("sequence",)
    SEQUENCE_FIELD_NUMBER: _ClassVar[int]
    sequence: int
    def __init__(self, sequence: _Optional[int] = ...) -> None: ...

class RobotState(_message.Message):
    __slots__ = ("state", "previous_state", "reason", "transitioning", "allowed_transitions", "faults", "current_map", "entered_time", "making_map")
    STATE_FIELD_NUMBER: _ClassVar[int]
    PREVIOUS_STATE_FIELD_NUMBER: _ClassVar[int]
    REASON_FIELD_NUMBER: _ClassVar[int]
    TRANSITIONING_FIELD_NUMBER: _ClassVar[int]
    ALLOWED_TRANSITIONS_FIELD_NUMBER: _ClassVar[int]
    FAULTS_FIELD_NUMBER: _ClassVar[int]
    CURRENT_MAP_FIELD_NUMBER: _ClassVar[int]
    ENTERED_TIME_FIELD_NUMBER: _ClassVar[int]
    MAKING_MAP_FIELD_NUMBER: _ClassVar[int]
    state: str
    previous_state: str
    reason: str
    transitioning: bool
    allowed_transitions: _containers.RepeatedScalarFieldContainer[str]
    faults: _containers.RepeatedScalarFieldContainer[str]
    current_map: str
    entered_time: float
    making_map: bool
    def __init__(self, state: _Optional[str] = ..., previous_state: _Optional[str] = ..., reason: _Optional[str] = ..., transitioning: bool = ..., allowed_transitions: _Optional[_Iterable[str]] = ..., faults: _Optional[_Iterable[str]] = ..., current_map: _Optional[str] = ..., entered_time: _Optional[float] = ..., making_map: bool = ...) -> None: ...

class Map(_message.Message):
    __slots__ = ("version", "width", "height", "resolution", "origin_x", "origin_y", "origin_yaw", "cells_rle")
    VERSION_FIELD_NUMBER: _ClassVar[int]
    WIDTH_FIELD_NUMBER: _ClassVar[int]
    HEIGHT_FIELD_NUMBER: _ClassVar[int]
    RESOLUTION_FIELD_NUMBER: _ClassVar[int]
    ORIGIN_X_FIELD_NUMBER: _ClassVar[int]
    ORIGIN_Y_FIELD_NUMBER: _ClassVar[int]
    ORIGIN_YAW_FIELD_NUMBER: _ClassVar[int]
    CELLS_RLE_FIELD_NUMBER: _ClassVar[int]
    version: int
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    cells_rle: bytes
    def __init__(self, version: _Optional[int] = ..., width: _Optional[int] = ..., height: _Optional[int] = ..., resolution: _Optional[float] = ..., origin_x: _Optional[float] = ..., origin_y: _Optional[float] = ..., origin_yaw: _Optional[float] = ..., cells_rle: _Optional[bytes] = ...) -> None: ...

class RobotPose(_message.Message):
    __slots__ = ("stamp", "x", "y", "heading")
    STAMP_FIELD_NUMBER: _ClassVar[int]
    X_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    HEADING_FIELD_NUMBER: _ClassVar[int]
    stamp: float
    x: float
    y: float
    heading: float
    def __init__(self, stamp: _Optional[float] = ..., x: _Optional[float] = ..., y: _Optional[float] = ..., heading: _Optional[float] = ...) -> None: ...

class Route(_message.Message):
    __slots__ = ("stamp", "points_cm")
    STAMP_FIELD_NUMBER: _ClassVar[int]
    POINTS_CM_FIELD_NUMBER: _ClassVar[int]
    stamp: float
    points_cm: _containers.RepeatedScalarFieldContainer[int]
    def __init__(self, stamp: _Optional[float] = ..., points_cm: _Optional[_Iterable[int]] = ...) -> None: ...

class NavigationStatus(_message.Message):
    __slots__ = ("goal_id", "state", "distance_remaining")
    class State(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        STATE_NONE: _ClassVar[NavigationStatus.State]
        STATE_ACCEPTED: _ClassVar[NavigationStatus.State]
        STATE_ON_THE_WAY: _ClassVar[NavigationStatus.State]
        STATE_CANCELLING: _ClassVar[NavigationStatus.State]
        STATE_REACHED: _ClassVar[NavigationStatus.State]
        STATE_FAILED: _ClassVar[NavigationStatus.State]
        STATE_CANCELLED: _ClassVar[NavigationStatus.State]
    STATE_NONE: NavigationStatus.State
    STATE_ACCEPTED: NavigationStatus.State
    STATE_ON_THE_WAY: NavigationStatus.State
    STATE_CANCELLING: NavigationStatus.State
    STATE_REACHED: NavigationStatus.State
    STATE_FAILED: NavigationStatus.State
    STATE_CANCELLED: NavigationStatus.State
    GOAL_ID_FIELD_NUMBER: _ClassVar[int]
    STATE_FIELD_NUMBER: _ClassVar[int]
    DISTANCE_REMAINING_FIELD_NUMBER: _ClassVar[int]
    goal_id: bytes
    state: NavigationStatus.State
    distance_remaining: float
    def __init__(self, goal_id: _Optional[bytes] = ..., state: _Optional[_Union[NavigationStatus.State, str]] = ..., distance_remaining: _Optional[float] = ...) -> None: ...

class NoGoZones(_message.Message):
    __slots__ = ("zones", "map_name")
    class Zone(_message.Message):
        __slots__ = ("name", "points_cm")
        NAME_FIELD_NUMBER: _ClassVar[int]
        POINTS_CM_FIELD_NUMBER: _ClassVar[int]
        name: str
        points_cm: _containers.RepeatedScalarFieldContainer[int]
        def __init__(self, name: _Optional[str] = ..., points_cm: _Optional[_Iterable[int]] = ...) -> None: ...
    ZONES_FIELD_NUMBER: _ClassVar[int]
    MAP_NAME_FIELD_NUMBER: _ClassVar[int]
    zones: _containers.RepeatedCompositeFieldContainer[NoGoZones.Zone]
    map_name: str
    def __init__(self, zones: _Optional[_Iterable[_Union[NoGoZones.Zone, _Mapping]]] = ..., map_name: _Optional[str] = ...) -> None: ...

class RobotSettings(_message.Message):
    __slots__ = ("cleaning_speed", "manual_speed", "row_spacing", "row_angle_auto", "row_angle", "limits", "manual_turn_speed")
    class Limits(_message.Message):
        __slots__ = ("cleaning_speed_min", "cleaning_speed_max", "manual_speed_min", "manual_speed_max", "row_spacing_min", "row_spacing_max", "row_spacing_step")
        CLEANING_SPEED_MIN_FIELD_NUMBER: _ClassVar[int]
        CLEANING_SPEED_MAX_FIELD_NUMBER: _ClassVar[int]
        MANUAL_SPEED_MIN_FIELD_NUMBER: _ClassVar[int]
        MANUAL_SPEED_MAX_FIELD_NUMBER: _ClassVar[int]
        ROW_SPACING_MIN_FIELD_NUMBER: _ClassVar[int]
        ROW_SPACING_MAX_FIELD_NUMBER: _ClassVar[int]
        ROW_SPACING_STEP_FIELD_NUMBER: _ClassVar[int]
        cleaning_speed_min: float
        cleaning_speed_max: float
        manual_speed_min: float
        manual_speed_max: float
        row_spacing_min: float
        row_spacing_max: float
        row_spacing_step: float
        def __init__(self, cleaning_speed_min: _Optional[float] = ..., cleaning_speed_max: _Optional[float] = ..., manual_speed_min: _Optional[float] = ..., manual_speed_max: _Optional[float] = ..., row_spacing_min: _Optional[float] = ..., row_spacing_max: _Optional[float] = ..., row_spacing_step: _Optional[float] = ...) -> None: ...
    CLEANING_SPEED_FIELD_NUMBER: _ClassVar[int]
    MANUAL_SPEED_FIELD_NUMBER: _ClassVar[int]
    ROW_SPACING_FIELD_NUMBER: _ClassVar[int]
    ROW_ANGLE_AUTO_FIELD_NUMBER: _ClassVar[int]
    ROW_ANGLE_FIELD_NUMBER: _ClassVar[int]
    LIMITS_FIELD_NUMBER: _ClassVar[int]
    MANUAL_TURN_SPEED_FIELD_NUMBER: _ClassVar[int]
    cleaning_speed: float
    manual_speed: float
    row_spacing: float
    row_angle_auto: bool
    row_angle: float
    limits: RobotSettings.Limits
    manual_turn_speed: float
    def __init__(self, cleaning_speed: _Optional[float] = ..., manual_speed: _Optional[float] = ..., row_spacing: _Optional[float] = ..., row_angle_auto: bool = ..., row_angle: _Optional[float] = ..., limits: _Optional[_Union[RobotSettings.Limits, _Mapping]] = ..., manual_turn_speed: _Optional[float] = ...) -> None: ...

class Rooms(_message.Message):
    __slots__ = ("rooms", "map_name")
    class Room(_message.Message):
        __slots__ = ("name", "points_cm", "x_cm", "y_cm", "area")
        NAME_FIELD_NUMBER: _ClassVar[int]
        POINTS_CM_FIELD_NUMBER: _ClassVar[int]
        X_CM_FIELD_NUMBER: _ClassVar[int]
        Y_CM_FIELD_NUMBER: _ClassVar[int]
        AREA_FIELD_NUMBER: _ClassVar[int]
        name: str
        points_cm: _containers.RepeatedScalarFieldContainer[int]
        x_cm: int
        y_cm: int
        area: float
        def __init__(self, name: _Optional[str] = ..., points_cm: _Optional[_Iterable[int]] = ..., x_cm: _Optional[int] = ..., y_cm: _Optional[int] = ..., area: _Optional[float] = ...) -> None: ...
    ROOMS_FIELD_NUMBER: _ClassVar[int]
    MAP_NAME_FIELD_NUMBER: _ClassVar[int]
    rooms: _containers.RepeatedCompositeFieldContainer[Rooms.Room]
    map_name: str
    def __init__(self, rooms: _Optional[_Iterable[_Union[Rooms.Room, _Mapping]]] = ..., map_name: _Optional[str] = ...) -> None: ...

class AssistantStatus(_message.Message):
    __slots__ = ("busy", "conversation", "doing")
    BUSY_FIELD_NUMBER: _ClassVar[int]
    CONVERSATION_FIELD_NUMBER: _ClassVar[int]
    DOING_FIELD_NUMBER: _ClassVar[int]
    busy: bool
    conversation: str
    doing: str
    def __init__(self, busy: bool = ..., conversation: _Optional[str] = ..., doing: _Optional[str] = ...) -> None: ...

class AssistantAnswer(_message.Message):
    __slots__ = ("text", "tools_used", "picture_jpeg", "mark_cm")
    TEXT_FIELD_NUMBER: _ClassVar[int]
    TOOLS_USED_FIELD_NUMBER: _ClassVar[int]
    PICTURE_JPEG_FIELD_NUMBER: _ClassVar[int]
    MARK_CM_FIELD_NUMBER: _ClassVar[int]
    text: str
    tools_used: _containers.RepeatedScalarFieldContainer[str]
    picture_jpeg: bytes
    mark_cm: _containers.RepeatedScalarFieldContainer[int]
    def __init__(self, text: _Optional[str] = ..., tools_used: _Optional[_Iterable[str]] = ..., picture_jpeg: _Optional[bytes] = ..., mark_cm: _Optional[_Iterable[int]] = ...) -> None: ...

class RequestId(_message.Message):
    __slots__ = ("request_id",)
    REQUEST_ID_FIELD_NUMBER: _ClassVar[int]
    request_id: int
    def __init__(self, request_id: _Optional[int] = ...) -> None: ...

class Ping(_message.Message):
    __slots__ = ("request_id",)
    REQUEST_ID_FIELD_NUMBER: _ClassVar[int]
    request_id: int
    def __init__(self, request_id: _Optional[int] = ...) -> None: ...

class ChangeMode(_message.Message):
    __slots__ = ("request_id", "mode")
    REQUEST_ID_FIELD_NUMBER: _ClassVar[int]
    MODE_FIELD_NUMBER: _ClassVar[int]
    request_id: int
    mode: str
    def __init__(self, request_id: _Optional[int] = ..., mode: _Optional[str] = ...) -> None: ...

class SaveMap(_message.Message):
    __slots__ = ("request_id", "name", "replace")
    REQUEST_ID_FIELD_NUMBER: _ClassVar[int]
    NAME_FIELD_NUMBER: _ClassVar[int]
    REPLACE_FIELD_NUMBER: _ClassVar[int]
    request_id: int
    name: str
    replace: bool
    def __init__(self, request_id: _Optional[int] = ..., name: _Optional[str] = ..., replace: bool = ...) -> None: ...

class SetNoGoZones(_message.Message):
    __slots__ = ("request_id", "map_name", "zones")
    REQUEST_ID_FIELD_NUMBER: _ClassVar[int]
    MAP_NAME_FIELD_NUMBER: _ClassVar[int]
    ZONES_FIELD_NUMBER: _ClassVar[int]
    request_id: int
    map_name: str
    zones: _containers.RepeatedCompositeFieldContainer[NoGoZones.Zone]
    def __init__(self, request_id: _Optional[int] = ..., map_name: _Optional[str] = ..., zones: _Optional[_Iterable[_Union[NoGoZones.Zone, _Mapping]]] = ...) -> None: ...

class SetRobotSettings(_message.Message):
    __slots__ = ("request_id", "settings")
    REQUEST_ID_FIELD_NUMBER: _ClassVar[int]
    SETTINGS_FIELD_NUMBER: _ClassVar[int]
    request_id: int
    settings: RobotSettings
    def __init__(self, request_id: _Optional[int] = ..., settings: _Optional[_Union[RobotSettings, _Mapping]] = ...) -> None: ...

class FindRooms(_message.Message):
    __slots__ = ("request_id",)
    REQUEST_ID_FIELD_NUMBER: _ClassVar[int]
    request_id: int
    def __init__(self, request_id: _Optional[int] = ...) -> None: ...

class SetRooms(_message.Message):
    __slots__ = ("request_id", "map_name", "rooms")
    REQUEST_ID_FIELD_NUMBER: _ClassVar[int]
    MAP_NAME_FIELD_NUMBER: _ClassVar[int]
    ROOMS_FIELD_NUMBER: _ClassVar[int]
    request_id: int
    map_name: str
    rooms: _containers.RepeatedCompositeFieldContainer[Rooms.Room]
    def __init__(self, request_id: _Optional[int] = ..., map_name: _Optional[str] = ..., rooms: _Optional[_Iterable[_Union[Rooms.Room, _Mapping]]] = ...) -> None: ...

class AskAssistant(_message.Message):
    __slots__ = ("request_id", "conversation", "text")
    REQUEST_ID_FIELD_NUMBER: _ClassVar[int]
    CONVERSATION_FIELD_NUMBER: _ClassVar[int]
    TEXT_FIELD_NUMBER: _ClassVar[int]
    request_id: int
    conversation: str
    text: str
    def __init__(self, request_id: _Optional[int] = ..., conversation: _Optional[str] = ..., text: _Optional[str] = ...) -> None: ...

class NameRooms(_message.Message):
    __slots__ = ("request_id", "rooms")
    REQUEST_ID_FIELD_NUMBER: _ClassVar[int]
    ROOMS_FIELD_NUMBER: _ClassVar[int]
    request_id: int
    rooms: _containers.RepeatedCompositeFieldContainer[Rooms.Room]
    def __init__(self, request_id: _Optional[int] = ..., rooms: _Optional[_Iterable[_Union[Rooms.Room, _Mapping]]] = ...) -> None: ...

class GoTo(_message.Message):
    __slots__ = ("request_id", "x", "y", "heading")
    REQUEST_ID_FIELD_NUMBER: _ClassVar[int]
    X_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    HEADING_FIELD_NUMBER: _ClassVar[int]
    request_id: int
    x: float
    y: float
    heading: float
    def __init__(self, request_id: _Optional[int] = ..., x: _Optional[float] = ..., y: _Optional[float] = ..., heading: _Optional[float] = ...) -> None: ...

class CancelGoal(_message.Message):
    __slots__ = ("request_id", "goal_id")
    REQUEST_ID_FIELD_NUMBER: _ClassVar[int]
    GOAL_ID_FIELD_NUMBER: _ClassVar[int]
    request_id: int
    goal_id: bytes
    def __init__(self, request_id: _Optional[int] = ..., goal_id: _Optional[bytes] = ...) -> None: ...

class Drive(_message.Message):
    __slots__ = ("request_id", "forward", "turn")
    REQUEST_ID_FIELD_NUMBER: _ClassVar[int]
    FORWARD_FIELD_NUMBER: _ClassVar[int]
    TURN_FIELD_NUMBER: _ClassVar[int]
    request_id: int
    forward: float
    turn: float
    def __init__(self, request_id: _Optional[int] = ..., forward: _Optional[float] = ..., turn: _Optional[float] = ...) -> None: ...

class Reply(_message.Message):
    __slots__ = ("request_id", "result", "message", "goal_id", "estimate", "plans", "history", "record", "replanned", "job_id", "answer", "rooms")
    class Result(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        RESULT_UNSPECIFIED: _ClassVar[Reply.Result]
        RESULT_OK: _ClassVar[Reply.Result]
        RESULT_REFUSED: _ClassVar[Reply.Result]
        RESULT_NOT_AVAILABLE: _ClassVar[Reply.Result]
        RESULT_TIMED_OUT: _ClassVar[Reply.Result]
        RESULT_BAD_REQUEST: _ClassVar[Reply.Result]
        RESULT_BUSY: _ClassVar[Reply.Result]
        RESULT_ERROR: _ClassVar[Reply.Result]
        RESULT_ALREADY_EXISTS: _ClassVar[Reply.Result]
    RESULT_UNSPECIFIED: Reply.Result
    RESULT_OK: Reply.Result
    RESULT_REFUSED: Reply.Result
    RESULT_NOT_AVAILABLE: Reply.Result
    RESULT_TIMED_OUT: Reply.Result
    RESULT_BAD_REQUEST: Reply.Result
    RESULT_BUSY: Reply.Result
    RESULT_ERROR: Reply.Result
    RESULT_ALREADY_EXISTS: Reply.Result
    REQUEST_ID_FIELD_NUMBER: _ClassVar[int]
    RESULT_FIELD_NUMBER: _ClassVar[int]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    GOAL_ID_FIELD_NUMBER: _ClassVar[int]
    ESTIMATE_FIELD_NUMBER: _ClassVar[int]
    PLANS_FIELD_NUMBER: _ClassVar[int]
    HISTORY_FIELD_NUMBER: _ClassVar[int]
    RECORD_FIELD_NUMBER: _ClassVar[int]
    REPLANNED_FIELD_NUMBER: _ClassVar[int]
    JOB_ID_FIELD_NUMBER: _ClassVar[int]
    ANSWER_FIELD_NUMBER: _ClassVar[int]
    ROOMS_FIELD_NUMBER: _ClassVar[int]
    request_id: int
    result: Reply.Result
    message: str
    goal_id: bytes
    estimate: CleaningEstimate
    plans: _containers.RepeatedCompositeFieldContainer[CleaningPlanInfo]
    history: CleaningHistory
    record: CleaningRecord
    replanned: bool
    job_id: int
    answer: AssistantAnswer
    rooms: Rooms
    def __init__(self, request_id: _Optional[int] = ..., result: _Optional[_Union[Reply.Result, str]] = ..., message: _Optional[str] = ..., goal_id: _Optional[bytes] = ..., estimate: _Optional[_Union[CleaningEstimate, _Mapping]] = ..., plans: _Optional[_Iterable[_Union[CleaningPlanInfo, _Mapping]]] = ..., history: _Optional[_Union[CleaningHistory, _Mapping]] = ..., record: _Optional[_Union[CleaningRecord, _Mapping]] = ..., replanned: bool = ..., job_id: _Optional[int] = ..., answer: _Optional[_Union[AssistantAnswer, _Mapping]] = ..., rooms: _Optional[_Union[Rooms, _Mapping]] = ...) -> None: ...

class SafetyStatus(_message.Message):
    __slots__ = ("source", "blocked_reason", "cliff", "bumper", "off_floor", "reflex_active", "stale")
    SOURCE_FIELD_NUMBER: _ClassVar[int]
    BLOCKED_REASON_FIELD_NUMBER: _ClassVar[int]
    CLIFF_FIELD_NUMBER: _ClassVar[int]
    BUMPER_FIELD_NUMBER: _ClassVar[int]
    OFF_FLOOR_FIELD_NUMBER: _ClassVar[int]
    REFLEX_ACTIVE_FIELD_NUMBER: _ClassVar[int]
    STALE_FIELD_NUMBER: _ClassVar[int]
    source: str
    blocked_reason: str
    cliff: bool
    bumper: bool
    off_floor: bool
    reflex_active: bool
    stale: _containers.RepeatedScalarFieldContainer[str]
    def __init__(self, source: _Optional[str] = ..., blocked_reason: _Optional[str] = ..., cliff: bool = ..., bumper: bool = ..., off_floor: bool = ..., reflex_active: bool = ..., stale: _Optional[_Iterable[str]] = ...) -> None: ...

class CleaningJob(_message.Message):
    __slots__ = ("what", "area", "spot_x", "spot_y", "plan_name", "direction", "passes", "edges", "return_to_dock", "room")
    class What(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        WHAT_EVERYTHING: _ClassVar[CleaningJob.What]
        WHAT_AREA: _ClassVar[CleaningJob.What]
        WHAT_SPOT: _ClassVar[CleaningJob.What]
        WHAT_PLAN: _ClassVar[CleaningJob.What]
        WHAT_ROOM: _ClassVar[CleaningJob.What]
    WHAT_EVERYTHING: CleaningJob.What
    WHAT_AREA: CleaningJob.What
    WHAT_SPOT: CleaningJob.What
    WHAT_PLAN: CleaningJob.What
    WHAT_ROOM: CleaningJob.What
    class Option(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        OPTION_DEFAULT: _ClassVar[CleaningJob.Option]
        OPTION_OFF: _ClassVar[CleaningJob.Option]
        OPTION_ON: _ClassVar[CleaningJob.Option]
    OPTION_DEFAULT: CleaningJob.Option
    OPTION_OFF: CleaningJob.Option
    OPTION_ON: CleaningJob.Option
    WHAT_FIELD_NUMBER: _ClassVar[int]
    AREA_FIELD_NUMBER: _ClassVar[int]
    SPOT_X_FIELD_NUMBER: _ClassVar[int]
    SPOT_Y_FIELD_NUMBER: _ClassVar[int]
    PLAN_NAME_FIELD_NUMBER: _ClassVar[int]
    DIRECTION_FIELD_NUMBER: _ClassVar[int]
    PASSES_FIELD_NUMBER: _ClassVar[int]
    EDGES_FIELD_NUMBER: _ClassVar[int]
    RETURN_TO_DOCK_FIELD_NUMBER: _ClassVar[int]
    ROOM_FIELD_NUMBER: _ClassVar[int]
    what: CleaningJob.What
    area: _containers.RepeatedScalarFieldContainer[float]
    spot_x: float
    spot_y: float
    plan_name: str
    direction: str
    passes: int
    edges: CleaningJob.Option
    return_to_dock: CleaningJob.Option
    room: str
    def __init__(self, what: _Optional[_Union[CleaningJob.What, str]] = ..., area: _Optional[_Iterable[float]] = ..., spot_x: _Optional[float] = ..., spot_y: _Optional[float] = ..., plan_name: _Optional[str] = ..., direction: _Optional[str] = ..., passes: _Optional[int] = ..., edges: _Optional[_Union[CleaningJob.Option, str]] = ..., return_to_dock: _Optional[_Union[CleaningJob.Option, str]] = ..., room: _Optional[str] = ...) -> None: ...

class CleaningEstimate(_message.Message):
    __slots__ = ("rows", "edges", "turns", "transits", "distance", "planned_area", "left_out_area", "unreachable_area", "time")
    ROWS_FIELD_NUMBER: _ClassVar[int]
    EDGES_FIELD_NUMBER: _ClassVar[int]
    TURNS_FIELD_NUMBER: _ClassVar[int]
    TRANSITS_FIELD_NUMBER: _ClassVar[int]
    DISTANCE_FIELD_NUMBER: _ClassVar[int]
    PLANNED_AREA_FIELD_NUMBER: _ClassVar[int]
    LEFT_OUT_AREA_FIELD_NUMBER: _ClassVar[int]
    UNREACHABLE_AREA_FIELD_NUMBER: _ClassVar[int]
    TIME_FIELD_NUMBER: _ClassVar[int]
    rows: int
    edges: int
    turns: int
    transits: int
    distance: float
    planned_area: float
    left_out_area: float
    unreachable_area: float
    time: float
    def __init__(self, rows: _Optional[int] = ..., edges: _Optional[int] = ..., turns: _Optional[int] = ..., transits: _Optional[int] = ..., distance: _Optional[float] = ..., planned_area: _Optional[float] = ..., left_out_area: _Optional[float] = ..., unreachable_area: _Optional[float] = ..., time: _Optional[float] = ...) -> None: ...

class CleaningStatus(_message.Message):
    __slots__ = ("state", "message", "job_id", "target", "job", "started", "coverage", "cleaned_area", "planned_area", "distance", "cleaning_time", "time_left", "rows_done", "rows_total", "rows_skipped", "current_row", "edges_done", "edges_total", "edges_skipped", "current_edge", "bumps", "pauses", "piece")
    class State(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        STATE_NONE: _ClassVar[CleaningStatus.State]
        STATE_STARTING: _ClassVar[CleaningStatus.State]
        STATE_CLEANING: _ClassVar[CleaningStatus.State]
        STATE_PAUSED: _ClassVar[CleaningStatus.State]
        STATE_FINISHED: _ClassVar[CleaningStatus.State]
        STATE_STOPPED: _ClassVar[CleaningStatus.State]
        STATE_FAILED: _ClassVar[CleaningStatus.State]
        STATE_ENDING: _ClassVar[CleaningStatus.State]
    STATE_NONE: CleaningStatus.State
    STATE_STARTING: CleaningStatus.State
    STATE_CLEANING: CleaningStatus.State
    STATE_PAUSED: CleaningStatus.State
    STATE_FINISHED: CleaningStatus.State
    STATE_STOPPED: CleaningStatus.State
    STATE_FAILED: CleaningStatus.State
    STATE_ENDING: CleaningStatus.State
    STATE_FIELD_NUMBER: _ClassVar[int]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    JOB_ID_FIELD_NUMBER: _ClassVar[int]
    TARGET_FIELD_NUMBER: _ClassVar[int]
    JOB_FIELD_NUMBER: _ClassVar[int]
    STARTED_FIELD_NUMBER: _ClassVar[int]
    COVERAGE_FIELD_NUMBER: _ClassVar[int]
    CLEANED_AREA_FIELD_NUMBER: _ClassVar[int]
    PLANNED_AREA_FIELD_NUMBER: _ClassVar[int]
    DISTANCE_FIELD_NUMBER: _ClassVar[int]
    CLEANING_TIME_FIELD_NUMBER: _ClassVar[int]
    TIME_LEFT_FIELD_NUMBER: _ClassVar[int]
    ROWS_DONE_FIELD_NUMBER: _ClassVar[int]
    ROWS_TOTAL_FIELD_NUMBER: _ClassVar[int]
    ROWS_SKIPPED_FIELD_NUMBER: _ClassVar[int]
    CURRENT_ROW_FIELD_NUMBER: _ClassVar[int]
    EDGES_DONE_FIELD_NUMBER: _ClassVar[int]
    EDGES_TOTAL_FIELD_NUMBER: _ClassVar[int]
    EDGES_SKIPPED_FIELD_NUMBER: _ClassVar[int]
    CURRENT_EDGE_FIELD_NUMBER: _ClassVar[int]
    BUMPS_FIELD_NUMBER: _ClassVar[int]
    PAUSES_FIELD_NUMBER: _ClassVar[int]
    PIECE_FIELD_NUMBER: _ClassVar[int]
    state: CleaningStatus.State
    message: str
    job_id: int
    target: str
    job: CleaningJob
    started: float
    coverage: float
    cleaned_area: float
    planned_area: float
    distance: float
    cleaning_time: float
    time_left: float
    rows_done: int
    rows_total: int
    rows_skipped: int
    current_row: int
    edges_done: int
    edges_total: int
    edges_skipped: int
    current_edge: int
    bumps: int
    pauses: int
    piece: int
    def __init__(self, state: _Optional[_Union[CleaningStatus.State, str]] = ..., message: _Optional[str] = ..., job_id: _Optional[int] = ..., target: _Optional[str] = ..., job: _Optional[_Union[CleaningJob, _Mapping]] = ..., started: _Optional[float] = ..., coverage: _Optional[float] = ..., cleaned_area: _Optional[float] = ..., planned_area: _Optional[float] = ..., distance: _Optional[float] = ..., cleaning_time: _Optional[float] = ..., time_left: _Optional[float] = ..., rows_done: _Optional[int] = ..., rows_total: _Optional[int] = ..., rows_skipped: _Optional[int] = ..., current_row: _Optional[int] = ..., edges_done: _Optional[int] = ..., edges_total: _Optional[int] = ..., edges_skipped: _Optional[int] = ..., current_edge: _Optional[int] = ..., bumps: _Optional[int] = ..., pauses: _Optional[int] = ..., piece: _Optional[int] = ...) -> None: ...

class CleaningPlan(_message.Message):
    __slots__ = ("job_id", "target", "pieces", "estimate")
    class Kind(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        KIND_ROW: _ClassVar[CleaningPlan.Kind]
        KIND_CONNECTOR: _ClassVar[CleaningPlan.Kind]
        KIND_TRANSIT: _ClassVar[CleaningPlan.Kind]
        KIND_EDGE: _ClassVar[CleaningPlan.Kind]
    KIND_ROW: CleaningPlan.Kind
    KIND_CONNECTOR: CleaningPlan.Kind
    KIND_TRANSIT: CleaningPlan.Kind
    KIND_EDGE: CleaningPlan.Kind
    class Piece(_message.Message):
        __slots__ = ("kind", "x0_cm", "y0_cm", "x1_cm", "y1_cm", "number", "pass_number")
        KIND_FIELD_NUMBER: _ClassVar[int]
        X0_CM_FIELD_NUMBER: _ClassVar[int]
        Y0_CM_FIELD_NUMBER: _ClassVar[int]
        X1_CM_FIELD_NUMBER: _ClassVar[int]
        Y1_CM_FIELD_NUMBER: _ClassVar[int]
        NUMBER_FIELD_NUMBER: _ClassVar[int]
        PASS_NUMBER_FIELD_NUMBER: _ClassVar[int]
        kind: CleaningPlan.Kind
        x0_cm: int
        y0_cm: int
        x1_cm: int
        y1_cm: int
        number: int
        pass_number: int
        def __init__(self, kind: _Optional[_Union[CleaningPlan.Kind, str]] = ..., x0_cm: _Optional[int] = ..., y0_cm: _Optional[int] = ..., x1_cm: _Optional[int] = ..., y1_cm: _Optional[int] = ..., number: _Optional[int] = ..., pass_number: _Optional[int] = ...) -> None: ...
    JOB_ID_FIELD_NUMBER: _ClassVar[int]
    TARGET_FIELD_NUMBER: _ClassVar[int]
    PIECES_FIELD_NUMBER: _ClassVar[int]
    ESTIMATE_FIELD_NUMBER: _ClassVar[int]
    job_id: int
    target: str
    pieces: _containers.RepeatedCompositeFieldContainer[CleaningPlan.Piece]
    estimate: CleaningEstimate
    def __init__(self, job_id: _Optional[int] = ..., target: _Optional[str] = ..., pieces: _Optional[_Iterable[_Union[CleaningPlan.Piece, _Mapping]]] = ..., estimate: _Optional[_Union[CleaningEstimate, _Mapping]] = ...) -> None: ...

class CleanedArea(_message.Message):
    __slots__ = ("job_id", "width", "height", "resolution", "origin_x", "origin_y", "cells_rle")
    JOB_ID_FIELD_NUMBER: _ClassVar[int]
    WIDTH_FIELD_NUMBER: _ClassVar[int]
    HEIGHT_FIELD_NUMBER: _ClassVar[int]
    RESOLUTION_FIELD_NUMBER: _ClassVar[int]
    ORIGIN_X_FIELD_NUMBER: _ClassVar[int]
    ORIGIN_Y_FIELD_NUMBER: _ClassVar[int]
    CELLS_RLE_FIELD_NUMBER: _ClassVar[int]
    job_id: int
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    cells_rle: bytes
    def __init__(self, job_id: _Optional[int] = ..., width: _Optional[int] = ..., height: _Optional[int] = ..., resolution: _Optional[float] = ..., origin_x: _Optional[float] = ..., origin_y: _Optional[float] = ..., cells_rle: _Optional[bytes] = ...) -> None: ...

class CleaningPlanInfo(_message.Message):
    __slots__ = ("name", "job", "estimate", "map_changed", "saved")
    NAME_FIELD_NUMBER: _ClassVar[int]
    JOB_FIELD_NUMBER: _ClassVar[int]
    ESTIMATE_FIELD_NUMBER: _ClassVar[int]
    MAP_CHANGED_FIELD_NUMBER: _ClassVar[int]
    SAVED_FIELD_NUMBER: _ClassVar[int]
    name: str
    job: CleaningJob
    estimate: CleaningEstimate
    map_changed: bool
    saved: float
    def __init__(self, name: _Optional[str] = ..., job: _Optional[_Union[CleaningJob, _Mapping]] = ..., estimate: _Optional[_Union[CleaningEstimate, _Mapping]] = ..., map_changed: bool = ..., saved: _Optional[float] = ...) -> None: ...

class CleaningRecord(_message.Message):
    __slots__ = ("id", "started", "ended", "status", "picture_png")
    ID_FIELD_NUMBER: _ClassVar[int]
    STARTED_FIELD_NUMBER: _ClassVar[int]
    ENDED_FIELD_NUMBER: _ClassVar[int]
    STATUS_FIELD_NUMBER: _ClassVar[int]
    PICTURE_PNG_FIELD_NUMBER: _ClassVar[int]
    id: str
    started: float
    ended: float
    status: CleaningStatus
    picture_png: bytes
    def __init__(self, id: _Optional[str] = ..., started: _Optional[float] = ..., ended: _Optional[float] = ..., status: _Optional[_Union[CleaningStatus, _Mapping]] = ..., picture_png: _Optional[bytes] = ...) -> None: ...

class CleaningTotals(_message.Message):
    __slots__ = ("cleans", "area", "time", "distance")
    CLEANS_FIELD_NUMBER: _ClassVar[int]
    AREA_FIELD_NUMBER: _ClassVar[int]
    TIME_FIELD_NUMBER: _ClassVar[int]
    DISTANCE_FIELD_NUMBER: _ClassVar[int]
    cleans: int
    area: float
    time: float
    distance: float
    def __init__(self, cleans: _Optional[int] = ..., area: _Optional[float] = ..., time: _Optional[float] = ..., distance: _Optional[float] = ...) -> None: ...

class CleaningHistory(_message.Message):
    __slots__ = ("records", "totals")
    RECORDS_FIELD_NUMBER: _ClassVar[int]
    TOTALS_FIELD_NUMBER: _ClassVar[int]
    records: _containers.RepeatedCompositeFieldContainer[CleaningRecord]
    totals: CleaningTotals
    def __init__(self, records: _Optional[_Iterable[_Union[CleaningRecord, _Mapping]]] = ..., totals: _Optional[_Union[CleaningTotals, _Mapping]] = ...) -> None: ...

class StartCleaning(_message.Message):
    __slots__ = ("request_id", "job")
    REQUEST_ID_FIELD_NUMBER: _ClassVar[int]
    JOB_FIELD_NUMBER: _ClassVar[int]
    request_id: int
    job: CleaningJob
    def __init__(self, request_id: _Optional[int] = ..., job: _Optional[_Union[CleaningJob, _Mapping]] = ...) -> None: ...

class PauseCleaning(_message.Message):
    __slots__ = ("request_id", "resume")
    REQUEST_ID_FIELD_NUMBER: _ClassVar[int]
    RESUME_FIELD_NUMBER: _ClassVar[int]
    request_id: int
    resume: bool
    def __init__(self, request_id: _Optional[int] = ..., resume: bool = ...) -> None: ...

class PreviewCleaning(_message.Message):
    __slots__ = ("request_id", "job")
    REQUEST_ID_FIELD_NUMBER: _ClassVar[int]
    JOB_FIELD_NUMBER: _ClassVar[int]
    request_id: int
    job: CleaningJob
    def __init__(self, request_id: _Optional[int] = ..., job: _Optional[_Union[CleaningJob, _Mapping]] = ...) -> None: ...

class SaveCleaningPlan(_message.Message):
    __slots__ = ("request_id", "name", "job", "replace")
    REQUEST_ID_FIELD_NUMBER: _ClassVar[int]
    NAME_FIELD_NUMBER: _ClassVar[int]
    JOB_FIELD_NUMBER: _ClassVar[int]
    REPLACE_FIELD_NUMBER: _ClassVar[int]
    request_id: int
    name: str
    job: CleaningJob
    replace: bool
    def __init__(self, request_id: _Optional[int] = ..., name: _Optional[str] = ..., job: _Optional[_Union[CleaningJob, _Mapping]] = ..., replace: bool = ...) -> None: ...

class ListCleaningPlans(_message.Message):
    __slots__ = ("request_id",)
    REQUEST_ID_FIELD_NUMBER: _ClassVar[int]
    request_id: int
    def __init__(self, request_id: _Optional[int] = ...) -> None: ...

class DeleteCleaningPlan(_message.Message):
    __slots__ = ("request_id", "name")
    REQUEST_ID_FIELD_NUMBER: _ClassVar[int]
    NAME_FIELD_NUMBER: _ClassVar[int]
    request_id: int
    name: str
    def __init__(self, request_id: _Optional[int] = ..., name: _Optional[str] = ...) -> None: ...

class ListCleaningHistory(_message.Message):
    __slots__ = ("request_id", "max_records")
    REQUEST_ID_FIELD_NUMBER: _ClassVar[int]
    MAX_RECORDS_FIELD_NUMBER: _ClassVar[int]
    request_id: int
    max_records: int
    def __init__(self, request_id: _Optional[int] = ..., max_records: _Optional[int] = ...) -> None: ...

class GetCleaningRecord(_message.Message):
    __slots__ = ("request_id", "id")
    REQUEST_ID_FIELD_NUMBER: _ClassVar[int]
    ID_FIELD_NUMBER: _ClassVar[int]
    request_id: int
    id: str
    def __init__(self, request_id: _Optional[int] = ..., id: _Optional[str] = ...) -> None: ...

class ExplorationStatus(_message.Message):
    __slots__ = ("state", "message", "job_id", "started", "places_left", "mapped_area", "exploring_time", "time_limit", "distance", "goals_sent", "goals_failed")
    class State(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        STATE_NONE: _ClassVar[ExplorationStatus.State]
        STATE_STARTING: _ClassVar[ExplorationStatus.State]
        STATE_EXPLORING: _ClassVar[ExplorationStatus.State]
        STATE_RETURNING: _ClassVar[ExplorationStatus.State]
        STATE_ENDING: _ClassVar[ExplorationStatus.State]
        STATE_FINISHED: _ClassVar[ExplorationStatus.State]
        STATE_STOPPED: _ClassVar[ExplorationStatus.State]
        STATE_FAILED: _ClassVar[ExplorationStatus.State]
    STATE_NONE: ExplorationStatus.State
    STATE_STARTING: ExplorationStatus.State
    STATE_EXPLORING: ExplorationStatus.State
    STATE_RETURNING: ExplorationStatus.State
    STATE_ENDING: ExplorationStatus.State
    STATE_FINISHED: ExplorationStatus.State
    STATE_STOPPED: ExplorationStatus.State
    STATE_FAILED: ExplorationStatus.State
    STATE_FIELD_NUMBER: _ClassVar[int]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    JOB_ID_FIELD_NUMBER: _ClassVar[int]
    STARTED_FIELD_NUMBER: _ClassVar[int]
    PLACES_LEFT_FIELD_NUMBER: _ClassVar[int]
    MAPPED_AREA_FIELD_NUMBER: _ClassVar[int]
    EXPLORING_TIME_FIELD_NUMBER: _ClassVar[int]
    TIME_LIMIT_FIELD_NUMBER: _ClassVar[int]
    DISTANCE_FIELD_NUMBER: _ClassVar[int]
    GOALS_SENT_FIELD_NUMBER: _ClassVar[int]
    GOALS_FAILED_FIELD_NUMBER: _ClassVar[int]
    state: ExplorationStatus.State
    message: str
    job_id: int
    started: float
    places_left: int
    mapped_area: float
    exploring_time: float
    time_limit: float
    distance: float
    goals_sent: int
    goals_failed: int
    def __init__(self, state: _Optional[_Union[ExplorationStatus.State, str]] = ..., message: _Optional[str] = ..., job_id: _Optional[int] = ..., started: _Optional[float] = ..., places_left: _Optional[int] = ..., mapped_area: _Optional[float] = ..., exploring_time: _Optional[float] = ..., time_limit: _Optional[float] = ..., distance: _Optional[float] = ..., goals_sent: _Optional[int] = ..., goals_failed: _Optional[int] = ...) -> None: ...

class Frontiers(_message.Message):
    __slots__ = ("job_id", "points_cm")
    JOB_ID_FIELD_NUMBER: _ClassVar[int]
    POINTS_CM_FIELD_NUMBER: _ClassVar[int]
    job_id: int
    points_cm: _containers.RepeatedScalarFieldContainer[int]
    def __init__(self, job_id: _Optional[int] = ..., points_cm: _Optional[_Iterable[int]] = ...) -> None: ...

class StartExploration(_message.Message):
    __slots__ = ("request_id", "time_limit")
    REQUEST_ID_FIELD_NUMBER: _ClassVar[int]
    TIME_LIMIT_FIELD_NUMBER: _ClassVar[int]
    request_id: int
    time_limit: float
    def __init__(self, request_id: _Optional[int] = ..., time_limit: _Optional[float] = ...) -> None: ...

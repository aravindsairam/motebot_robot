cmake_minimum_required(VERSION 3.10)
project(vacuum_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotState.msg"
  "msg/SafetyStatus.msg"
  "msg/CleaningJob.msg"
  "msg/CleaningEstimate.msg"
  "msg/CleaningStatus.msg"
  "msg/CleaningPiece.msg"
  "msg/CleaningPlan.msg"
  "msg/CleaningPlanInfo.msg"
  "msg/CleaningRecord.msg"
  "msg/CleaningTotals.msg"
  "msg/NoGoZone.msg"
  "msg/ExplorationStatus.msg"
  "msg/RobotSettings.msg"
  "msg/Room.msg"
  "msg/Rooms.msg"
  "msg/RoomCleaning.msg"
  "msg/AssistantStatus.msg"
  "msg/Detection.msg"
  "srv/ChangeState.srv"
  "srv/SaveMap.srv"
  "srv/SetNoGoZones.srv"
  "srv/SetRobotSettings.srv"
  "srv/AskAssistant.srv"
  "srv/FindRooms.srv"
  "srv/SetRooms.srv"
  "srv/RoomReport.srv"
  "srv/NameRooms.srv"
  "srv/FindObjects.srv"
  "srv/StartCleaning.srv"
  "srv/PreviewCleaning.srv"
  "srv/SaveCleaningPlan.srv"
  "srv/ListCleaningPlans.srv"
  "srv/DeleteCleaningPlan.srv"
  "srv/ListCleaningHistory.srv"
  "srv/GetCleaningRecord.srv"
  "srv/StartExploration.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

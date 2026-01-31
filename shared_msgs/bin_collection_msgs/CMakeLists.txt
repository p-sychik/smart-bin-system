cmake_minimum_required(VERSION 3.8)
project(bin_collection_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PathPoint.msg"
  "msg/RecordedPath.msg"
  "msg/PathStatus.msg"
  "msg/RFIDDetection.msg"
  "msg/BinInfo.msg"
  "msg/UltrasonicArray.msg"
  "msg/AlignmentStatus.msg"
  "msg/HookStatus.msg"
  "msg/MissionStatus.msg"

  "srv/StartRecording.srv"
  "srv/StopRecording.srv"
  "srv/GetPath.srv"
  "srv/LookupBin.srv"
  "srv/RegisterBin.srv"
  "srv/EngageHook.srv"
  "srv/DisengageHook.srv"
  "srv/StartMission.srv"
  "srv/AbortMission.srv"

  "action/ExecutePath.action"
  "action/ApproachBin.action"
  "action/AlignToHook.action"
  "action/DockToBin.action"
  "action/CollectBin.action"

  DEPENDENCIES geometry_msgs builtin_interfaces std_msgs
)

ament_package()

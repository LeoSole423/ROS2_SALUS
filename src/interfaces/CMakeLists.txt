cmake_minimum_required(VERSION 3.8)
project(interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BatteryMissionGuard.msg"
  "msg/CmdVelFinal.msg"
  "msg/DriveTelemetry.msg"
  "msg/NoGoPoint.msg"
  "msg/NoGoZone.msg"
  "msg/NavSnapshotLayers.msg"
  "msg/NavTelemetry.msg"
  "msg/NavEvent.msg"
  "srv/SetZonesGeoJson.srv"
  "srv/GetZonesState.srv"
  "srv/SetKeepoutZones.srv"
  "srv/GetKeepoutState.srv"
  "srv/SetNavGoalLL.srv"
  "srv/CancelNavGoal.srv"
  "srv/BrakeNav.srv"
  "srv/SetManualMode.srv"
  "srv/SetManualCmd.srv"
  "srv/GetNavState.srv"
  "srv/GetNavSnapshot.srv"
  "srv/SetRouteMissionLL.srv"
  "srv/CancelRouteMission.srv"
  "srv/GetRouteMissionState.srv"
  "srv/SetDatum.srv"
  "srv/GetDatum.srv"
  "srv/CameraPan.srv"
  "srv/CameraPreset.srv"
  "srv/CameraSavePreset.srv"
  "srv/CameraPtz.srv"
  "srv/CameraPtzState.srv"
  "srv/CameraStatus.srv"
  DEPENDENCIES builtin_interfaces diagnostic_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

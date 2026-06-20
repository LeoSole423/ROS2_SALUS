cmake_minimum_required(VERSION 3.8)
project(navegacion_gps_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(nav2_is_path_clearance_valid_condition_bt_node SHARED
  src/is_path_clearance_valid_condition.cpp
)

add_library(nav2_trace_replan_decorator_bt_node SHARED
  src/trace_replan_decorator.cpp
)

target_compile_definitions(nav2_is_path_clearance_valid_condition_bt_node
  PRIVATE BT_PLUGIN_EXPORT
)

target_compile_definitions(nav2_trace_replan_decorator_bt_node
  PRIVATE BT_PLUGIN_EXPORT
)

target_include_directories(nav2_is_path_clearance_valid_condition_bt_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(nav2_trace_replan_decorator_bt_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(nav2_is_path_clearance_valid_condition_bt_node
  behaviortree_cpp_v3
  nav2_behavior_tree
  nav2_msgs
  nav_msgs
  rclcpp
)

ament_target_dependencies(nav2_trace_replan_decorator_bt_node
  behaviortree_cpp_v3
  diagnostic_msgs
  geometry_msgs
  interfaces
  nav_msgs
  rclcpp
)

install(TARGETS nav2_is_path_clearance_valid_condition_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS nav2_trace_replan_decorator_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()

#!/usr/bin/env bash
set -euo pipefail

CYCLONEDDS_WIFI_URI="file:///ros2_ws/src/navegacion_gps/config/cyclonedds_wifi.xml"
REAL_WIFI_LAUNCH="/ros2_ws/src/navegacion_gps/launch/real_global_v2_wifi.launch.py"

./tools/exec.sh "source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export ROS_DOMAIN_ID=0; export ROS_LOCALHOST_ONLY=0; export CYCLONEDDS_URI=${CYCLONEDDS_WIFI_URI}; ros2 launch ${REAL_WIFI_LAUNCH}"

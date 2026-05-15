#!/usr/bin/env bash
set -euo pipefail

# Perfil recomendado para operacion remota del robot por WiFi.
CYCLONEDDS_WIFI_URI="file:///ros2_ws/src/navegacion_gps/config/cyclonedds_wifi.xml"
REAL_WIFI_LAUNCH="/ros2_ws/src/navegacion_gps/launch/real_global_v2_wifi.launch.py"
REAL_BASE_LAUNCH="/ros2_ws/src/navegacion_gps/launch/real_global_v2.launch.py"

./tools/exec.sh "source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export ROS_DOMAIN_ID=0; export ROS_LOCALHOST_ONLY=0; export CYCLONEDDS_URI=${CYCLONEDDS_WIFI_URI}; if [ -f ${REAL_WIFI_LAUNCH} ]; then ros2 launch ${REAL_WIFI_LAUNCH} $*; elif [ -f ${REAL_BASE_LAUNCH} ]; then ros2 launch ${REAL_BASE_LAUNCH} use_rviz:=False enable_scan_wifi_debug:=True scan_wifi_debug_topic:=/scan_wifi_debug scan_wifi_debug_publish_hz:=2.0 scan_wifi_debug_beam_stride:=4 scan_wifi_debug_range_max_m:=12.0 use_keepout:=False launch_web_app:=True $*; else ros2 launch navegacion_gps real_global_v2.launch.py use_rviz:=False enable_scan_wifi_debug:=True scan_wifi_debug_topic:=/scan_wifi_debug scan_wifi_debug_publish_hz:=2.0 scan_wifi_debug_beam_stride:=4 scan_wifi_debug_range_max_m:=12.0 use_keepout:=False launch_web_app:=True $*; fi"

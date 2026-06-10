#!/usr/bin/env bash
set -euo pipefail

# Perfil real base: mantiene el launch no-WiFi para compatibilidad y pruebas
# locales. Para operacion remota del robot por WiFi usar:
#   ./tools/launch_real_global_v2_wifi.sh

CYCLONEDDS_WIFI_URI="file:///ros2_ws/src/navegacion_gps/config/cyclonedds_wifi.xml"

./tools/exec.sh "source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export ROS_DOMAIN_ID=0; export ROS_LOCALHOST_ONLY=0; export CYCLONEDDS_URI=${CYCLONEDDS_WIFI_URI}; ros2 launch navegacion_gps real_global_v2.launch.py"

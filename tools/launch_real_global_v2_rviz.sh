#!/usr/bin/env bash
set -euo pipefail

# Local RViz launcher for the real global-nav v2 stack.
#
# This script is meant to be run on the operator PC, not on the Raspberry Pi.
# The robot is expected to already be running `real_global_v2.launch.py`
# headless, while this helper starts only RViz inside the local Docker
# workspace so the PC can attach to the live ROS 2 graph over DDS.
#
# It does not launch sensors, controllers, or navigation on the robot.
# It only opens the RViz profile for `real_global_v2`.
#
# Cyclone DDS, ROS_DOMAIN_ID y el perfil XML se exportan explícitamente para
# priorizar discovery liviano y payloads más chicos cuando el operador se une
# por Wi‑Fi a la navegación real del robot.
CYCLONEDDS_WIFI_URI="file:///ros2_ws/src/navegacion_gps/config/cyclonedds_wifi.xml"
DISPLAY_VALUE="${DISPLAY:-:0}"

prepare_x11_for_docker_rviz() {
  if [[ -z "${DISPLAY_VALUE}" ]]; then
    return
  fi
  if command -v xhost >/dev/null 2>&1; then
    xhost +local: >/dev/null 2>&1 || true
  fi
}

prepare_x11_for_docker_rviz

./tools/exec.sh "export DISPLAY=${DISPLAY_VALUE}; export QT_X11_NO_MITSHM=1; source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export ROS_DOMAIN_ID=0; export ROS_LOCALHOST_ONLY=0; export CYCLONEDDS_URI=${CYCLONEDDS_WIFI_URI}; ros2 launch navegacion_gps rviz_real_global_v2.launch.py custom_urdf:=/ros2_ws/src/navegacion_gps/models/cuatri_real.urdf rviz_config:=/ros2_ws/src/navegacion_gps/config/rviz_global_v2.rviz"

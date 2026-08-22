#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/docker_ros_env.sh"

CONTAINER="${ROS2_CONTAINER_NAME:-ros2_salus}"
RMW_IMPLEMENTATION_VALUE="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
LAUNCH_RVIZ="${LAUNCH_RVIZ:-true}"
DISPLAY_VALUE="${DISPLAY:-:0}"

"${SCRIPT_DIR}/stop_sim_global_v2.sh" >/dev/null 2>&1 || true

if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
  echo "El contenedor ${CONTAINER} no esta corriendo."
  exit 1
fi

ros2_prepare_gui

ros2_docker_exec "${CONTAINER}" bash -lc "
  mkdir -p /ros2_ws/logs
  nohup bash -lc 'export DISPLAY=${DISPLAY_VALUE}; export QT_X11_NO_MITSHM=1; export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION_VALUE}; source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; ros2 launch navegacion_gps sim_global_v2.launch.py gps_profile:=f9p_rtk launch_web_app:=True use_keepout:=True' \
    </dev/null >/ros2_ws/logs/sim_global_v2.log 2>&1 &
"

sleep 5

echo "Web app sim_global_v2 disponible en ws://localhost:8766"
echo "Abrir: src/map_tools/web/index.html"

if [[ "${LAUNCH_RVIZ,,}" == "false" || "${LAUNCH_RVIZ}" == "0" ]]; then
  exit 0
fi

"${SCRIPT_DIR}/exec.sh" "export DISPLAY=${DISPLAY_VALUE}; export QT_X11_NO_MITSHM=1; source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; ros2 launch navegacion_gps rviz_sim_global_v2.launch.py"

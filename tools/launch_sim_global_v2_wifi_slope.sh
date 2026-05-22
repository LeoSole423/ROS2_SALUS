#!/usr/bin/env bash
set -euo pipefail

CONTAINER="${ROS2_CONTAINER_NAME:-ros2_salus}"
RMW_IMPLEMENTATION_VALUE="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
CYCLONEDDS_WIFI_URI="file:///ros2_ws/src/navegacion_gps/config/cyclonedds_wifi.xml"
SIM_WIFI_LAUNCH="/ros2_ws/src/navegacion_gps/launch/sim_global_v2_wifi.launch.py"
RVIZ_WIFI_LAUNCH="/ros2_ws/src/navegacion_gps/launch/rviz_sim_global_v2_wifi.launch.py"
SLOPE_WORLD="/ros2_ws/src/navegacion_gps/worlds/slope_lidar.world"
LAUNCH_RVIZ="${LAUNCH_RVIZ:-true}"
DISPLAY_VALUE="${DISPLAY:-:0}"

prepare_x11_for_docker_rviz() {
  if [[ -z "${DISPLAY_VALUE}" ]]; then
    return
  fi
  if command -v xhost >/dev/null 2>&1; then
    xhost +local: >/dev/null 2>&1 || true
  fi
}

./tools/stop_sim_global_v2.sh >/dev/null 2>&1 || true

if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
  echo "El contenedor ${CONTAINER} no esta corriendo."
  exit 1
fi

prepare_x11_for_docker_rviz
docker exec "${CONTAINER}" bash -lc "
  mkdir -p /ros2_ws/logs
  nohup bash -lc 'export DISPLAY=${DISPLAY_VALUE}; export QT_X11_NO_MITSHM=1; export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION_VALUE}; export ROS_DOMAIN_ID=0; export ROS_LOCALHOST_ONLY=0; export CYCLONEDDS_URI=${CYCLONEDDS_WIFI_URI}; source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; ros2 launch ${SIM_WIFI_LAUNCH} gps_profile:=f9p_rtk launch_web_app:=True use_keepout:=False world:=${SLOPE_WORLD} world_name:=slope_lidar' \
    </dev/null >/ros2_ws/logs/sim_global_v2_wifi_slope.log 2>&1 &
"

sleep 5

echo "Web app sim_global_v2_wifi slope disponible en ws://localhost:8766"
echo "World: ${SLOPE_WORLD}"
echo "Abrir: src/map_tools/web/index.html"

if [[ "${LAUNCH_RVIZ,,}" == "false" || "${LAUNCH_RVIZ}" == "0" ]]; then
  exit 0
fi

./tools/exec.sh "export DISPLAY=${DISPLAY_VALUE}; export QT_X11_NO_MITSHM=1; source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION_VALUE}; export ROS_DOMAIN_ID=0; export ROS_LOCALHOST_ONLY=0; export CYCLONEDDS_URI=${CYCLONEDDS_WIFI_URI}; ros2 launch ${RVIZ_WIFI_LAUNCH}"

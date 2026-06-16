#!/usr/bin/env bash
set -euo pipefail

CONTAINER="${ROS2_CONTAINER_NAME:-ros2_salus}"
RMW_IMPLEMENTATION_VALUE="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
CYCLONEDDS_WIFI_URI="file:///ros2_ws/src/navegacion_gps/config/cyclonedds_wifi.xml"
RVIZ_WIFI_LAUNCH="/ros2_ws/src/navegacion_gps/launch/rviz_sim_global_v2_wifi.launch.py"
DISPLAY_VALUE="${DISPLAY:-:0}"
URDF_PATH="/ros2_ws/src/navegacion_gps/models/cuatri_real_v2.urdf"
LIDAR_TO_SCAN_PARAMS="/ros2_ws/src/navegacion_gps/config/pointcloud_to_laserscan_tilted_lidar_sim.yaml"
MODEL_NAME="cuatri_real_v2"

prepare_x11_for_docker_rviz() {
  if [[ -z "${DISPLAY_VALUE}" ]]; then
    return
  fi
  if command -v xhost >/dev/null 2>&1; then
    xhost +local: >/dev/null 2>&1 || true
  fi
}

prepare_x11_for_docker_rviz

LAUNCH_RVIZ=false ./tools/launch_sim_global_v2_wifi.sh \
  custom_urdf:="${URDF_PATH}" \
  lidar_to_scan_params_file:="${LIDAR_TO_SCAN_PARAMS}" \
  model_name:="${MODEL_NAME}" \
  enable_sim_compass:=true \
  enable_compass_initial_guess:=true \
  "$@"

if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
  echo "El contenedor ${CONTAINER} no esta corriendo."
  exit 1
fi

docker exec "${CONTAINER}" bash -lc "
  export DISPLAY=${DISPLAY_VALUE}
  export QT_X11_NO_MITSHM=1
  export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION_VALUE}
  export ROS_DOMAIN_ID=0
  export ROS_LOCALHOST_ONLY=0
  export CYCLONEDDS_URI=${CYCLONEDDS_WIFI_URI}
  source /opt/ros/humble/setup.bash
  source /ros2_ws/install/setup.bash
  ros2 launch ${RVIZ_WIFI_LAUNCH} custom_urdf:=${URDF_PATH}
"

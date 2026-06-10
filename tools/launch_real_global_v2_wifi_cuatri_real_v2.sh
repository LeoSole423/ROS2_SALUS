#!/usr/bin/env bash
set -euo pipefail

# Perfil real WiFi para validar el URDF realista V2 con un recorte mas alto
# del LaserScan 3D->2D.
URDF_PATH="/ros2_ws/src/navegacion_gps/models/cuatri_real_v2.urdf"
LIDAR_TO_SCAN_PARAMS="/ros2_ws/src/navegacion_gps/config/pointcloud_to_laserscan_real_cuatri_real_v2.yaml"

./tools/launch_real_global_v2_wifi.sh \
  custom_urdf:="${URDF_PATH}" \
  lidar_to_scan_params_file:="${LIDAR_TO_SCAN_PARAMS}" \
  "$@"

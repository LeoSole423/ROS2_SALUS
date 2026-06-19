#!/usr/bin/env bash
set -euo pipefail

# RViz local para el perfil real global v2 Wi-Fi, con el display extra de la
# salida 3D del scan_ground_filter (/scan_3d/no_ground) ACTIVO.
#
# Igual que tools/launch_real_global_v2_wifi_rviz.sh pero usando el .rviz
# dedicado rviz_global_v2_wifi_scan_ground.rviz (no modifica el original).
# Pensado para confirmar a ojo, en el robot real, que el nodo saca el piso:
# ademas del LaserScan 2D liviano (/scan_wifi_debug) se ve la nube 3D sin piso.
#
# OJO wifi: /scan_3d/no_ground es la nube completa y pesa. Si satura el enlace,
# apaga el display "ScanGround NoGround" en el panel de Displays y segui con el
# 2D. Corre en la PC del operador, no en la Raspberry. El robot tiene que estar
# corriendo real_global_v2_wifi.launch.py con enable_scan_ground_filter:=True.
CYCLONEDDS_WIFI_URI="file:///ros2_ws/src/navegacion_gps/config/cyclonedds_wifi.xml"
RVIZ_WIFI_LAUNCH="/ros2_ws/src/navegacion_gps/launch/rviz_real_global_v2_wifi.launch.py"
RVIZ_CONFIG="/ros2_ws/src/navegacion_gps/config/rviz_global_v2_wifi_scan_ground.rviz"
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

./tools/exec.sh "export DISPLAY=${DISPLAY_VALUE}; export QT_X11_NO_MITSHM=1; source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export ROS_DOMAIN_ID=0; export ROS_LOCALHOST_ONLY=0; export CYCLONEDDS_URI=${CYCLONEDDS_WIFI_URI}; ros2 launch ${RVIZ_WIFI_LAUNCH} custom_urdf:=/ros2_ws/src/navegacion_gps/models/cuatri_real_v2.urdf rviz_config:=${RVIZ_CONFIG}"

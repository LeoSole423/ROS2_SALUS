#!/usr/bin/env bash
set -euo pipefail

# RViz LIVIANO 2D para el perfil real global v2 Wi-Fi, pensado para operacion
# remota por wifi donde el ancho de banda importa.
#
# Diferencia con los otros viewers wifi:
#   - launch_real_global_v2_wifi_rviz.sh        -> trae el Global Costmap
#       (300x300 m @ 0.25 = ~1.4 MB por publicacion, con always_send_full_costmap)
#   - launch_real_global_v2_wifi_rviz_scan_ground.sh -> encima la nube 3D
#       /scan_3d/no_ground (PointCloud2 completa, pesadisima por wifi)
#
# Este usa rviz_global_v2_wifi_2d.rviz, que es el perfil wifi SIN el Global
# Costmap y sin nube 3D: solo lo 2D esencial y chico ->
#   TF, RobotModel, LaserScan /scan_wifi_debug (2D decimado, 2 Hz),
#   Local Costmap (30x30 m @ 0.1 = ~90 KB), odom, plan/ruta y zonas.
# El /scan_wifi_debug ya refleja el /scan final (con scan_ground_filter activo
# sale de /scan_3d/no_ground), asi que igual ves el efecto del filtro en 2D.
#
# Corre en la PC del operador, no en la Raspberry. El robot tiene que estar
# corriendo real_global_v2_wifi.launch.py (idealmente con
# enable_scan_ground_filter:=True).
CYCLONEDDS_WIFI_URI="file:///ros2_ws/src/navegacion_gps/config/cyclonedds_wifi.xml"
RVIZ_WIFI_LAUNCH="/ros2_ws/src/navegacion_gps/launch/rviz_real_global_v2_wifi.launch.py"
RVIZ_CONFIG="/ros2_ws/src/navegacion_gps/config/rviz_global_v2_wifi_2d.rviz"
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

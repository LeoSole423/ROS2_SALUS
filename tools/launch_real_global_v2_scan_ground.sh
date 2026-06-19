#!/usr/bin/env bash
set -euo pipefail

# Perfil real global con el scan_ground_filter (segmentacion de suelo estilo
# Autoware) ACTIVO, para validar/operar el filtro en el robot real.
#
# Toma de referencia ./tools/launch_real_global_v2.sh y NO modifica
# real_global_v2.launch.py: solo pasa launch args. El URDF default ya es
# cuatri_real_v2.urdf (RS16 pitcheado 10°), necesario para que el filtro nivele
# bien la nube (target_frame base_footprint).
#
# Override de cualquier arg pasandolo al final (gana el ultimo), p.ej.:
#   ./tools/launch_real_global_v2_scan_ground.sh use_rviz:=True
#   ./tools/launch_real_global_v2_scan_ground.sh enable_scan_ground_filter:=False  # baseline A/B
#
# Para medir KPIs (FP en costmap + frenos falsos) en OTRA terminal, con el stack
# ya corriendo:
#   ./tools/exec.sh "ros2 launch navegacion_gps validate_scan_ground_real.launch.py \
#     label:=real_filtered output_path:=/tmp/real_filtered.json duration_s:=60"

# DDS fijado a la LAN cableada (eth0 en la raspi). Necesario para que el PC
# operador (eno1) descubra el robot por el cable; ver config/cyclonedds_lan.xml.
CYCLONEDDS_LAN_URI="file:///ros2_ws/src/navegacion_gps/config/cyclonedds_lan.xml"

./tools/exec.sh "source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export ROS_DOMAIN_ID=0; export ROS_LOCALHOST_ONLY=0; export CYCLONEDDS_URI=${CYCLONEDDS_LAN_URI}; ros2 launch navegacion_gps real_global_v2.launch.py enable_scan_ground_filter:=True scan_ground_min_height:=0.10 scan_ground_max_height:=2.50 $*"

#!/usr/bin/env bash
set -euo pipefail

# Arranca el escenario de rampa para depurar el scan_ground_filter:
# - Gazebo server + GUI
# - Nav2 local + collision_monitor
# - scan_ground_filter habilitado
# - RViz mostrando /scan_3d/no_ground

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/docker_ros_env.sh"

CONTAINER="${ROS2_CONTAINER_NAME:-ros2_salus}"
RMW_IMPLEMENTATION_VALUE="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
DISPLAY_VALUE="${DISPLAY:-:0}"
DURATION_S="${DURATION_S:-3600}"
LOG_FILE="/ros2_ws/logs/scan_ground_ramp_debug.log"
RVIZ_CONFIG="/tmp/rviz_scan_ground_no_ground.rviz"

if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
  echo "El contenedor ${CONTAINER} no esta corriendo."
  echo "Levantalo con: ./tools/up-salus.sh ros2"
  exit 1
fi

ros2_prepare_gui

"${SCRIPT_DIR}/stop_sim_local_v2.sh" >/dev/null 2>&1 || true
"${SCRIPT_DIR}/stop_sim_global_v2.sh" >/dev/null 2>&1 || true

docker exec "${CONTAINER}" bash -lc "
  set -eo pipefail
  mkdir -p /ros2_ws/logs
  source /opt/ros/humble/setup.bash
  source /ros2_ws/install/setup.bash
  python3 - <<'PY'
from pathlib import Path
src = Path('/ros2_ws/src/navegacion_gps/config/rviz_local_v2.rviz')
dst = Path('${RVIZ_CONFIG}')
text = src.read_text()
text = text.replace('Value: /scan_3d', 'Value: /scan_3d/no_ground')
text = text.replace('Name: Lidar Points', 'Name: Lidar No Ground')
dst.write_text(text)
PY
"

docker exec -d \
  -e "DISPLAY=${DISPLAY_VALUE}" \
  -e "QT_X11_NO_MITSHM=1" \
  -e "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION_VALUE}" \
  "${CONTAINER}" bash -lc "
    source /opt/ros/humble/setup.bash
    source /ros2_ws/install/setup.bash
    ros2 launch navegacion_gps validate_scan_ground.launch.py \
      enable_scan_ground_filter:=True \
      scan_ground_min_height:=0.10 \
      scan_ground_max_height:=2.50 \
      collision_backup_recovery_enabled:=True \
      collision_backup_distance_m:=0.50 \
      collision_backup_speed_mps:=0.25 \
      collision_backup_cooldown_s:=8.0 \
      use_rviz:=False \
      duration_s:=${DURATION_S} \
      label:=manual_rviz \
      output_path:=/tmp/sgf_manual.json \
      > ${LOG_FILE} 2>&1
  "

echo "Esperando topicos de LiDAR filtrado..."
for _ in $(seq 1 20); do
  if docker exec "${CONTAINER}" bash -lc "
    source /opt/ros/humble/setup.bash
    source /ros2_ws/install/setup.bash
    ros2 topic list | grep -qx /scan_3d/no_ground
  "; then
    break
  fi
  sleep 1
done

docker exec -d \
  -e "DISPLAY=${DISPLAY_VALUE}" \
  -e "QT_X11_NO_MITSHM=1" \
  "${CONTAINER}" bash -lc \
  "ign gazebo -g --force-version 6 >/ros2_ws/logs/gazebo_gui.log 2>&1"

docker exec -d \
  -e "DISPLAY=${DISPLAY_VALUE}" \
  -e "QT_X11_NO_MITSHM=1" \
  -e "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION_VALUE}" \
  "${CONTAINER}" bash -lc \
  "source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; rviz2 -d ${RVIZ_CONFIG} >/ros2_ws/logs/rviz_scan_ground_ramp.log 2>&1"

echo "Listo: Gazebo GUI + RViz abiertos."
echo "RViz muestra /scan_3d/no_ground. Para comparar, agrega /scan_3d como PointCloud2."
echo "Zonas collision_monitor documentadas:"
echo "  slow_zone:          x=5.35 m, slowdown_ratio=0.75"
echo "  critical_slow_zone: x=3.50 m, slowdown_ratio=0.4375"
echo "  stop_zone:          x=2.05 m, action=approach, ttc=2.0s"
echo "  footprint:          x=1.05 m, action=stop"
echo "Recovery activo: STOP -> cancela goal -> Nav2 BackUp 0.50 m -> reenvia el goal."
echo "Log: ${LOG_FILE}"

#!/usr/bin/env bash
set -euo pipefail

CONTAINER="${ROS2_CONTAINER_NAME:-ros2_salus}"
PATTERN='ign gazebo|ros_gz_bridge|ros_gz_sim/create|sim_global_v2.launch.py|rviz_sim_global_v2.launch.py|nav_global_v2.launch.py|localization_global_v2.launch.py|sim_v2_base.launch.py|sim_sensor_normalizer_v2|sim_compass_hdg|vehicle_controller_server|controller_server_node|nav_command_server|ackermann_odometry|ekf_filter_node_local_v2|ekf_filter_node_map|navsat_transform|datum_setter|map_gps_absolute_measurement|global_odom_stationary_gate|global_imu_stationary_gate|global_yaw_stationary_hold|gps_course_heading|compass_heading_gate|lifecycle_manager_global_navigation_v2|collision_monitor_lifecycle_manager_global_v2|planner_server|controller_server|smoother_server|bt_navigator|behavior_server|waypoint_follower|collision_monitor|keepout_filter_mask_server|keepout_costmap_filter_info_server|stop_zone_republisher|map_server|zones_manager|nav_snapshot_server|web_zone_server|rviz2'

if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
  echo "El contenedor ${CONTAINER} no esta corriendo."
  exit 1
fi

docker exec "${CONTAINER}" bash -lc "
  pids=\$(ps -eo pid=,args= | grep -E \"${PATTERN}\" | grep -v grep | awk '{print \$1}')
  if [ -n \"\${pids}\" ]; then
    kill \${pids} || true
    sleep 2
  fi
  remaining=\$(ps -eo pid=,args= | grep -E \"${PATTERN}\" | grep -v grep || true)
  if [ -n \"\${remaining}\" ]; then
    remaining_pids=\$(printf '%s\n' \"\${remaining}\" | awk '{print \$1}')
    kill -9 \${remaining_pids} || true
    sleep 1
    remaining=\$(ps -eo pid=,args= | grep -E \"${PATTERN}\" | grep -v grep || true)
    if [ -n \"\${remaining}\" ]; then
      echo \"Aun quedan procesos:\" >&2
      echo \"\${remaining}\" >&2
      exit 2
    fi
  fi
"

host_pids=$(ps -eo pid=,args= | grep -E "${PATTERN}|gz sim" | grep -v grep | awk '{print $1}' || true)
if [ -n "${host_pids}" ]; then
  kill ${host_pids} || true
  sleep 2
fi

remaining_host=$(ps -eo pid=,args= | grep -E "${PATTERN}|gz sim" | grep -v grep || true)
if [ -n "${remaining_host}" ]; then
  remaining_host_pids=$(printf '%s\n' "${remaining_host}" | awk '{print $1}')
  kill -9 ${remaining_host_pids} || true
  sleep 1
  remaining_host=$(ps -eo pid=,args= | grep -E "${PATTERN}|gz sim" | grep -v grep || true)
  if [ -n "${remaining_host}" ]; then
    echo "Aun quedan procesos de simulacion global en host:" >&2
    echo "${remaining_host}" >&2
    exit 3
  fi
fi

echo "Simulacion global v2 detenida."

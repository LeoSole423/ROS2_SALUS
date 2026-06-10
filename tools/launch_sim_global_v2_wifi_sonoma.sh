#!/usr/bin/env bash
set -euo pipefail

SONOMA_WORLD="/ros2_ws/src/navegacion_gps/worlds/sonoma_salus.world"

exec ./tools/launch_sim_global_v2_wifi.sh \
  world:="${SONOMA_WORLD}" \
  world_name:=sonoma_salus \
  spawn_x:=278.08 \
  spawn_y:=-134.22 \
  spawn_z:=3.1 \
  spawn_yaw:=0.97 \
  datum_lat:=38.1606 \
  datum_lon:=-122.4540 \
  datum_yaw_deg:=0.0 \
  "$@"

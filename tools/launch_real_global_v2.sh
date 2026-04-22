#!/usr/bin/env bash
set -euo pipefail

# Perfil real base: mantiene el launch no-WiFi para compatibilidad y pruebas
# locales. Para operacion remota del robot por WiFi usar:
#   ./tools/launch_real_global_v2_wifi.sh

./tools/exec.sh "source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; ros2 launch navegacion_gps real_global_v2.launch.py"

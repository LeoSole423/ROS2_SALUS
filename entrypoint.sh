#!/usr/bin/env bash
set -e

if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

# Planificador agricola de cobertura (modo Campo). Va antes del workspace de
# SALUS para que este pueda superponerlo si alguna vez se compila local.
if [ -f /opt/salus_coverage_ws/install/setup.bash ]; then
  source /opt/salus_coverage_ws/install/setup.bash
fi

if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
fi

exec "$@"

#!/usr/bin/env bash
set -euo pipefail

# Ejecuta un comando dentro del contenedor ya levantado.
# Uso:
#   ./tools/exec.sh                # abre una shell interactiva
#   ./tools/exec.sh <cmd> [args]   # ejecuta el comando dentro del contenedor

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/docker_ros_env.sh"

CONTAINER="${ROS2_CONTAINER_NAME:-ros2_salus}"
RMW_IMPLEMENTATION_VALUE="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
DOCKER_TTY_ARGS=()

if [[ -n "${DISPLAY:-}" ]]; then
  ros2_prepare_gui || true
fi

if [[ -t 0 && -t 1 ]]; then
  DOCKER_TTY_ARGS=(-it)
fi

if [[ $# -eq 0 ]]; then
  ros2_docker_exec "${DOCKER_TTY_ARGS[@]}" "${CONTAINER}" bash -lc "export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION_VALUE}; exec bash"
  exit 0
fi

ros2_docker_exec "${DOCKER_TTY_ARGS[@]}" "${CONTAINER}" bash -lc "export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION_VALUE}; $*"

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/docker_ros_env.sh"

CONTAINER="${ROS2_CONTAINER_NAME:-ros2_salus}"
RMW_IMPLEMENTATION_VALUE="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
DOCKER_TTY_ARGS=()

if [[ -t 0 && -t 1 ]]; then
  DOCKER_TTY_ARGS=(-it)
fi

usage() {
  cat <<'EOF'
Uso:
  ./tools/sim_battery.sh preset <full|under_load|watching|return_home_rest|return_home_load|stale|suspect|unavailable>
  ./tools/sim_battery.sh set <recovered_v> <loaded_v> --traction <on|off> [--ready <on|off>] [--fresh <on|off>] [--suspect <on|off>]

Ejemplos:
  ./tools/sim_battery.sh preset full
  ./tools/sim_battery.sh preset under_load
  ./tools/sim_battery.sh set 57.0 56.2 --traction on
  ./tools/sim_battery.sh set 60.0 59.8 --traction off --ready on --fresh on --suspect off
EOF
}

require_container() {
  if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
    echo "El contenedor ${CONTAINER} no esta corriendo."
    exit 1
  fi
}

bool_value() {
  case "${1,,}" in
    on|true|1|yes) echo "true" ;;
    off|false|0|no) echo "false" ;;
    *)
      echo "Valor booleano invalido: ${1}" >&2
      exit 1
      ;;
  esac
}

run_ros2() {
  local command="$1"
  ros2_docker_exec "${DOCKER_TTY_ARGS[@]}" "${CONTAINER}" bash -lc "\
    export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION_VALUE}; \
    source /opt/ros/humble/setup.bash; \
    source /ros2_ws/install/setup.bash; \
    ${command}"
}

if [[ $# -lt 2 ]]; then
  usage
  exit 1
fi

require_container

mode="$1"
shift

case "${mode}" in
  preset)
    preset_name="${1:-}"
    if [[ -z "${preset_name}" ]]; then
      usage
      exit 1
    fi
    echo "Aplicando preset de bateria simulada: ${preset_name}"
    run_ros2 "ros2 service call /sim_battery/set_preset interfaces/srv/SetSimBatteryPreset \"{preset: '${preset_name}'}\""
    ;;
  set)
    if [[ $# -lt 4 ]]; then
      usage
      exit 1
    fi
    recovered_v="$1"
    loaded_v="$2"
    shift 2

    traction_value=""
    ready_value="true"
    fresh_value="true"
    suspect_value="false"

    while [[ $# -gt 0 ]]; do
      case "$1" in
        --traction)
          traction_value="$(bool_value "${2:-}")"
          shift 2
          ;;
        --ready)
          ready_value="$(bool_value "${2:-}")"
          shift 2
          ;;
        --fresh)
          fresh_value="$(bool_value "${2:-}")"
          shift 2
          ;;
        --suspect)
          suspect_value="$(bool_value "${2:-}")"
          shift 2
          ;;
        *)
          echo "Argumento desconocido: $1" >&2
          usage
          exit 1
          ;;
      esac
    done

    if [[ -z "${traction_value}" ]]; then
      echo "Falta --traction <on|off>" >&2
      usage
      exit 1
    fi

    echo "Aplicando bateria simulada: recovered=${recovered_v}V loaded=${loaded_v}V traction=${traction_value}"
    run_ros2 "ros2 service call /sim_battery/set_state interfaces/srv/SetSimBatteryState \"{recovered_voltage_v: ${recovered_v}, loaded_voltage_v: ${loaded_v}, traction_active: ${traction_value}, ready: ${ready_value}, fresh: ${fresh_value}, suspect: ${suspect_value}}\""
    ;;
  *)
    usage
    exit 1
    ;;
esac

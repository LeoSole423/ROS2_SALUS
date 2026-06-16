#!/usr/bin/env bash
set -euo pipefail

CONTAINER="${ROS2_CONTAINER_NAME:-ros2_salus}"
WS="/ros2_ws"
LABEL="${1:-compass_run}"
DURATION_S="${2:-60}"
RMW_IMPLEMENTATION_VALUE="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${WS}/artifacts/compass_calibration"
OUT_FILE="${OUT_DIR}/${LABEL}_${STAMP}.json"

if [[ $# -gt 0 ]]; then
  shift
fi
if [[ $# -gt 0 ]]; then
  shift
fi

EXTRA_ARGS=("$@")
LABEL_Q="$(printf '%q' "${LABEL}")"
DURATION_Q="$(printf '%q' "${DURATION_S}")"
OUT_FILE_Q="$(printf '%q' "${OUT_FILE}")"
EXTRA_QUOTED=""
DOCKER_TTY_ARGS=()

if [[ -t 0 && -t 1 ]]; then
  DOCKER_TTY_ARGS=(-it)
fi

if [[ ${#EXTRA_ARGS[@]} -gt 0 ]]; then
  printf -v EXTRA_QUOTED '%q ' "${EXTRA_ARGS[@]}"
fi

echo "Grabando calibracion compass label='${LABEL}' duration_s='${DURATION_S}'"
echo "Salida: ${OUT_FILE}"

docker exec "${DOCKER_TTY_ARGS[@]}" "${CONTAINER}" bash -lc "\
  set -eo pipefail && \
  export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION_VALUE} && \
  source /opt/ros/\${ROS_DISTRO:-humble}/setup.bash && \
  if [ -f ${WS}/install/setup.bash ]; then source ${WS}/install/setup.bash; fi && \
  mkdir -p ${OUT_DIR} && \
  cd ${WS} && \
  ros2 run navegacion_gps compass_calibration_recorder \
    --label ${LABEL_Q} \
    --duration-s ${DURATION_Q} \
    --output ${OUT_FILE_Q} \
    ${EXTRA_QUOTED}"

echo "Calibracion guardada en ${OUT_FILE}"

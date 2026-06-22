#!/usr/bin/env bash
set -euo pipefail

# Compila uno o varios paquetes dentro del contenedor ya levantado.
# Uso:
#   ./tools/compile-ros.sh               # compila todo el workspace
#   ./tools/compile-ros.sh pkg1 pkg2 ... # compila solo esos paquetes

CONTAINER="${ROS2_CONTAINER_NAME:-ros2_salus}"
WS="/ros2_ws"
RMW_IMPLEMENTATION_VALUE="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
DOCKER_TTY_ARGS=()

if [[ -t 0 && -t 1 ]]; then
  DOCKER_TTY_ARGS=(-it)
fi

if [[ $# -gt 0 ]]; then
  PKG_LIST=("$@")
  BUILD_CMD="colcon build --packages-select ${PKG_LIST[*]} --symlink-install"
  CLEAN_CMD="for pkg in ${PKG_LIST[*]}; do rm -rf ${WS}/build/\$pkg ${WS}/install/\$pkg; done"
  TARGET_MSG="${PKG_LIST[*]}"
else
  BUILD_CMD="colcon build --symlink-install"
  CLEAN_CMD=":"
  TARGET_MSG="todo el workspace"
fi

echo "Compilando ${TARGET_MSG} dentro del contenedor '${CONTAINER}'..."

docker exec "${DOCKER_TTY_ARGS[@]}" "${CONTAINER}" bash -lc "\
  # Dentro del contenedor evitamos '-u' porque los setup.bash de ROS usan vars no definidas
  set -eo pipefail && \
  export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION_VALUE} && \
  source /opt/ros/\${ROS_DISTRO:-humble}/setup.bash && \
  if [ -f ${WS}/install/setup.bash ]; then source ${WS}/install/setup.bash; fi && \
  cd ${WS} && \
  ${CLEAN_CMD} && \
  ${BUILD_CMD} && \
  echo 'Build finalizado. Para usar:' && \
  echo '  source ${WS}/install/setup.bash'"

echo "Listo."

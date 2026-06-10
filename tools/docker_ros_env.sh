#!/usr/bin/env bash

# Helpers compartidos para ejecutar ROS 2 dentro del contenedor sin depender
# del entorno grafico que tenia Docker cuando se creo el contenedor.

ros2_x11_socket_for_display() {
  local display="${1:-}"
  local display_id

  if [[ -z "${display}" ]]; then
    return 1
  fi

  display_id="${display%%.*}"
  display_id="${display_id##*:}"

  if [[ ! "${display_id}" =~ ^[0-9]+$ ]]; then
    return 1
  fi

  printf '/tmp/.X11-unix/X%s\n' "${display_id}"
}

ros2_prepare_gui() {
  local display="${DISPLAY:-}"
  local x11_socket
  local local_user

  if [[ -z "${display}" ]]; then
    echo "DISPLAY no esta definido. Abrir RViz/Gazebo dentro de Docker requiere una sesion grafica local." >&2
    return 1
  fi

  if x11_socket="$(ros2_x11_socket_for_display "${display}")"; then
    if [[ ! -S "${x11_socket}" ]]; then
      echo "DISPLAY=${display}, pero no existe ${x11_socket} en el host." >&2
      echo "Si la sesion grafica cambio, reinicia el launch desde una terminal del escritorio actual." >&2
      return 1
    fi
  fi

  if command -v xhost >/dev/null 2>&1; then
    local_user="$(id -un)"
    xhost "+SI:localuser:${local_user}" >/dev/null 2>&1 || true
  fi
}

ros2_docker_exec_env=()

ros2_build_docker_exec_env() {
  ros2_docker_exec_env=(
    -e "DISPLAY=${DISPLAY:-}"
    -e "XAUTHORITY=/home/ros/.Xauthority"
    -e "QT_X11_NO_MITSHM=1"
  )
}

ros2_docker_exec() {
  ros2_build_docker_exec_env
  docker exec "${ros2_docker_exec_env[@]}" "$@"
}

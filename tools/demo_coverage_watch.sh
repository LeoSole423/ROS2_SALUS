#!/usr/bin/env bash
# Sigue el avance de la mision de cobertura hasta que termina.
set -euo pipefail

CONTAINER="${ROS2_CONTAINER_NAME:-ros2_salus}"
INTERVAL_S="${INTERVAL_S:-5}"

if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
  echo "El contenedor ${CONTAINER} no esta corriendo." >&2
  exit 1
fi

echo "hora      pasada   xte[m]  estado            (Ctrl-C para salir)"

docker exec -i "${CONTAINER}" bash -lc "
  source /opt/ros/humble/setup.bash >/dev/null 2>&1
  source /ros2_ws/install/setup.bash >/dev/null 2>&1
  export RMW_IMPLEMENTATION=\"${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}\"
  while true; do
    state=\$(ros2 service call /route_executor/get_state interfaces/srv/GetRouteMissionState '{}' 2>/dev/null)
    target=\$(printf '%s' \"\${state}\" | grep -o 'current_target_index=[0-9]*' | head -1 | cut -d= -f2)
    xte=\$(printf '%s' \"\${state}\" | grep -o 'cross_track_error_m=[0-9.e+-]*' | head -1 | cut -d= -f2)
    status=\$(printf '%s' \"\${state}\" | grep -o \"status='[^']*'\" | head -1 | cut -d= -f2- | tr -d \\\")
    if [ -z \"\${target}\" ]; then
      echo \"\$(date +%H:%M:%S)  sin mision activa\"
    else
      printf '%s  %6s  %7.2f  %s\n' \"\$(date +%H:%M:%S)\" \"\${target}\" \"\${xte:-0}\" \"\${status}\"
    fi
    case \"\${status}\" in
      *completed*) echo 'Mision completada.'; break ;;
      *failed*)    echo 'Mision fallida.'; break ;;
    esac
    sleep ${INTERVAL_S}
  done
"

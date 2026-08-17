#!/usr/bin/env bash
# Levanta todo lo necesario para ver una cobertura tipo cortadora y la lanza:
# contenedor -> simulacion sim_global_v2 -> cockpit -> ruta de cobertura.
#
# Es idempotente: lo que ya este corriendo no se toca. Para forzar una
# simulacion nueva usar --restart-sim.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
COCKPIT_DIR="${WORKSPACE_DIR}/cockpit"
CONTAINER="${ROS2_CONTAINER_NAME:-ros2_salus}"
COCKPIT_PORT="${COCKPIT_PORT:-5173}"
COCKPIT_LOG="${COCKPIT_LOG:-/tmp/cockpit-dev.log}"
BOOT_TIMEOUT_S="${BOOT_TIMEOUT_S:-120}"

FIELD_LENGTH_M="20"
FIELD_WIDTH_M="20"
CUTTER_WIDTH_M="2"
OVERLAP_RATIO="0.15"
# El Smac de sim_global_v2 traza a 2.9 m: pedir mas grande achica el
# recorrido nominal a pura cabecera sin que el vehiculo lo necesite.
MIN_TURNING_RADIUS_M="2.9"
RESTART_SIM="false"
START_COCKPIT="true"
OPEN_BROWSER="true"
ASSUME_YES="false"
DRY_RUN="false"

usage() {
  cat <<'EOF'
Uso: ./tools/demo_coverage.sh [opciones]

  --field-length-m N        Largo de cada pasada        (default 20)
  --field-width-m N         Ancho total a cubrir        (default 20)
  --cutter-width-m N        Ancho util de corte         (default 2)
  --overlap-ratio N         Solape entre pasadas        (default 0.15)
  --min-turning-radius-m N  Radio minimo Ackermann      (default 2.9)

  --restart-sim             Reinicia la simulacion aunque ya este corriendo
  --no-cockpit              No levanta el dev server del cockpit
  --no-browser              No abre el navegador
  --dry-run                 Calcula y muestra el plan, no mueve el vehiculo
  -y, --yes                 No pide confirmacion antes de lanzar la ruta
  -h, --help                Esta ayuda

Para que los giros salgan en U limpia el lote tiene que ser mas ancho que
2 * radio + ancho de corte. Con radio 4 y corte 2 eso son 10 m.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --field-length-m) FIELD_LENGTH_M="$2"; shift 2 ;;
    --field-width-m) FIELD_WIDTH_M="$2"; shift 2 ;;
    --cutter-width-m) CUTTER_WIDTH_M="$2"; shift 2 ;;
    --overlap-ratio) OVERLAP_RATIO="$2"; shift 2 ;;
    --min-turning-radius-m) MIN_TURNING_RADIUS_M="$2"; shift 2 ;;
    --restart-sim) RESTART_SIM="true"; shift ;;
    --no-cockpit) START_COCKPIT="false"; shift ;;
    --no-browser) OPEN_BROWSER="false"; shift ;;
    --dry-run) DRY_RUN="true"; shift ;;
    -y|--yes) ASSUME_YES="true"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Opcion desconocida: $1" >&2; usage >&2; exit 2 ;;
  esac
done

step() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
ok()   { printf '    \033[32mOK\033[0m  %s\n' "$*"; }
warn() { printf '    \033[33m!\033[0m   %s\n' "$*"; }
die()  { printf '\n\033[31mError:\033[0m %s\n' "$*" >&2; exit 1; }

in_container() {
  docker exec -i "${CONTAINER}" bash -lc "
    source /opt/ros/humble/setup.bash >/dev/null 2>&1
    source /ros2_ws/install/setup.bash >/dev/null 2>&1
    export RMW_IMPLEMENTATION=\"${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}\"
    $1"
}

sim_is_up() {
  in_container 'ros2 node list 2>/dev/null' | grep -qx '/route_executor'
}

# ---------------------------------------------------------------- contenedor
step "Contenedor ${CONTAINER}"
if docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
  ok "ya esta corriendo"
else
  warn "no esta corriendo, levantandolo"
  "${SCRIPT_DIR}/up-salus.sh" >/dev/null || die "no se pudo levantar el contenedor"
  ok "levantado"
fi

# --------------------------------------------------------------- simulacion
step "Simulacion sim_global_v2"
if [[ "${RESTART_SIM}" == "true" ]]; then
  warn "--restart-sim: bajando la simulacion actual"
  "${SCRIPT_DIR}/stop_sim_global_v2.sh" >/dev/null 2>&1 || true
fi

if sim_is_up && [[ "${RESTART_SIM}" == "false" ]]; then
  ok "ya esta corriendo (no la toco)"
else
  warn "arrancando (Gazebo tarda ~30 s)"
  LAUNCH_RVIZ=false "${SCRIPT_DIR}/launch_sim_global_v2.sh" >/dev/null 2>&1 || \
    die "fallo el launch; revisar 'docker exec ${CONTAINER} cat /ros2_ws/logs/sim_global_v2.log'"
  deadline=$(( SECONDS + BOOT_TIMEOUT_S ))
  until sim_is_up; do
    (( SECONDS < deadline )) || die "la simulacion no levanto en ${BOOT_TIMEOUT_S}s"
    sleep 3
  done
  ok "route_executor arriba"
fi

step "Esperando fix GPS y odometria"
deadline=$(( SECONDS + BOOT_TIMEOUT_S ))
until in_container 'ros2 topic list 2>/dev/null' | grep -qx '/odometry/global'; do
  (( SECONDS < deadline )) || die "no aparecio /odometry/global"
  sleep 2
done
ok "/gps/fix y /odometry/global publicando"

# ------------------------------------------------------------------ cockpit
if [[ "${START_COCKPIT}" == "true" ]]; then
  step "Cockpit (puerto ${COCKPIT_PORT})"
  if ss -ltn 2>/dev/null | grep -q ":${COCKPIT_PORT}[[:space:]]"; then
    ok "ya esta escuchando"
  elif [[ ! -d "${COCKPIT_DIR}/node_modules" ]]; then
    warn "faltan node_modules en ${COCKPIT_DIR}; correr 'npm install' ahi. Sigo sin cockpit."
    START_COCKPIT="false"
  else
    warn "arrancando dev server (log: ${COCKPIT_LOG})"
    ( cd "${COCKPIT_DIR}" && nohup npm run dev -- --host 0.0.0.0 --port "${COCKPIT_PORT}" \
        >"${COCKPIT_LOG}" 2>&1 & )
    deadline=$(( SECONDS + 60 ))
    until ss -ltn 2>/dev/null | grep -q ":${COCKPIT_PORT}[[:space:]]"; do
      (( SECONDS < deadline )) || die "el cockpit no levanto; ver ${COCKPIT_LOG}"
      sleep 2
    done
    ok "arriba"
  fi
fi

# ----------------------------------------------------------------- cobertura
step "Plan de cobertura ${FIELD_LENGTH_M} x ${FIELD_WIDTH_M} m"
PLAN_JSON="$("${SCRIPT_DIR}/run_coverage_waypoints.sh" \
  --field-length-m "${FIELD_LENGTH_M}" \
  --field-width-m "${FIELD_WIDTH_M}" \
  --cutter-width-m "${CUTTER_WIDTH_M}" \
  --overlap-ratio "${OVERLAP_RATIO}" \
  --min-turning-radius-m "${MIN_TURNING_RADIUS_M}")" || die "fallo la generacion del plan"

python3 -c '
import json, sys
plan = json.load(sys.stdin)
turns = plan["headland_turns"]
print("    pasadas            : %d  (separacion %.3f m)" % (plan["row_count"], plan["lane_spacing_m"]))
print("    giros              : %d U limpias, %d omega" % (turns["clean_uturns"], turns["omega_turns"]))
print("    separacion minima  : %.2f m  (hace falta %.1f m para una U limpia)"
      % (turns["min_separation_m"], turns["separation_needed_for_uturn_m"]))
print("    recorrido estimado : %.1f m" % plan["estimated_path_length_m"])
print("    cabecera libre     : %.2f m adelante, %.2f m atras, %.2f m al costado"
      % (plan["required_headland_m"]["before"], plan["required_headland_m"]["after"],
         plan["required_headland_m"]["lateral_centerline_overflow"]))
print("    waypoints a la ruta: %d" % plan["route_waypoint_count"])
if turns["omega_turns"]:
    print()
    print("    AVISO: hay %d giros omega. El lote es mas angosto que el diametro" % turns["omega_turns"])
    print("           de giro, asi que el vehiculo va a hacer rulos en las cabeceras.")
    print("           Para U limpias el ancho tiene que superar %.1f m."
          % (turns["separation_needed_for_uturn_m"] + plan["cutter_width_m"]))
' <<<"${PLAN_JSON}"

if [[ "${DRY_RUN}" == "true" ]]; then
  step "--dry-run: no se envia la ruta"
  exit 0
fi

if [[ "${ASSUME_YES}" != "true" ]]; then
  echo
  read -r -p "    Lanzar la ruta y mover el vehiculo? [s/N] " answer
  case "${answer}" in
    s|S|y|Y) ;;
    *) echo "    Cancelado. El plan de arriba no se envio."; exit 0 ;;
  esac
fi

if [[ "${START_COCKPIT}" == "true" && "${OPEN_BROWSER}" == "true" ]] && command -v xdg-open >/dev/null 2>&1; then
  xdg-open "http://localhost:${COCKPIT_PORT}/" >/dev/null 2>&1 || true
fi

step "Enviando la ruta"
"${SCRIPT_DIR}/run_coverage_waypoints.sh" \
  --field-length-m "${FIELD_LENGTH_M}" \
  --field-width-m "${FIELD_WIDTH_M}" \
  --cutter-width-m "${CUTTER_WIDTH_M}" \
  --overlap-ratio "${OVERLAP_RATIO}" \
  --min-turning-radius-m "${MIN_TURNING_RADIUS_M}" \
  --send-route >/dev/null || die "el route_executor rechazo la ruta"
ok "ruta activa"

cat <<EOF

  Cockpit  : http://localhost:${COCKPIT_PORT}/   -> modulo nav2 / NavLive
  Gazebo   : ventana 'ign gazebo gui'
  La traza roja del recorrido esta prendida por defecto en el mapa.

  Seguir el avance:
    ./tools/demo_coverage_watch.sh

  Cortar la mision:
    docker exec -it ${CONTAINER} bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 service call /route_executor/cancel_route interfaces/srv/CancelRouteMission "{}"'

EOF

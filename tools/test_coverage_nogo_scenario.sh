#!/usr/bin/env bash
# Runner comun para los escenarios no-go de cobertura completa.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCENARIO="${1:-}"
[[ "${SCENARIO}" == "inside" || "${SCENARIO}" == "boundary" ]] || {
  echo "Uso interno: $0 inside|boundary [opciones]" >&2
  exit 2
}
shift

source "${SCRIPT_DIR}/docker_ros_env.sh"

CONTAINER="${ROS2_CONTAINER_NAME:-ros2_salus}"
RMW_VALUE="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
BOOT_TIMEOUT_S="${BOOT_TIMEOUT_S:-180}"
MISSION_TIMEOUT_S="${MISSION_TIMEOUT_S:-900}"
FIELD_LENGTH_M="${FIELD_LENGTH_M:-36.0}"
FIELD_WIDTH_M="${FIELD_WIDTH_M:-23.0}"
ZONE_FORWARD_M="${ZONE_FORWARD_M:-}"
ZONE_LEFT_M="${ZONE_LEFT_M:-}"
ZONE_RADIUS_M="${ZONE_RADIUS_M:-1.5}"
if [[ -z "${ZONE_FORWARD_M}" && "${SCENARIO}" == "inside" ]]; then
  ZONE_FORWARD_M="14.0"
fi
RESTART_SIM="true"
OPEN_RVIZ="true"
PREVIEW_ONLY="false"
KEEP_TEST_ZONE="false"
STAMP="$(date +%Y%m%d_%H%M%S)"
REPORT_PATH="/ros2_ws/logs/coverage_nogo_${SCENARIO}_${STAMP}.json"

usage() {
  cat <<EOF
Uso: $0 ${SCENARIO} [opciones]

  --no-restart      Usa la simulacion que ya esta corriendo.
  --no-rviz         No abre RViz.
  --preview-only    Valida todo el trazado, pero no mueve el vehiculo.
  --keep-test-zone  No restaura las zonas originales al terminar.
  --timeout-s N     Plazo del recorrido completo (default ${MISSION_TIMEOUT_S}).
  --field-length-m N Largo del poligono de cobertura (default ${FIELD_LENGTH_M} m).
  --field-width-m N  Ancho del poligono de cobertura (default ${FIELD_WIDTH_M} m).
  --zone-forward-m N Posicion longitudinal de la zona (inside default 14.0 m).
  --zone-left-m N    Posicion lateral de la zona (inside default centro; boundary
                     default radio desde el borde).
  --zone-radius-m N  Radio de la zona no-go (default ${ZONE_RADIUS_M} m).
  --report PATH     Reporte JSON dentro del contenedor.
  -h, --help        Muestra esta ayuda.

La ejecucion normal reinicia sim_global_v2, carga una zona circular, genera el
poligono de cobertura, inicia la mision y espera hasta "route completed". Si se
interrumpe o falla, cancela la ruta y restaura el GeoJSON original.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-restart) RESTART_SIM="false"; shift ;;
    --no-rviz) OPEN_RVIZ="false"; shift ;;
    --preview-only) PREVIEW_ONLY="true"; shift ;;
    --keep-test-zone) KEEP_TEST_ZONE="true"; shift ;;
    --timeout-s) MISSION_TIMEOUT_S="$2"; shift 2 ;;
    --field-length-m) FIELD_LENGTH_M="$2"; shift 2 ;;
    --field-width-m) FIELD_WIDTH_M="$2"; shift 2 ;;
    --zone-forward-m) ZONE_FORWARD_M="$2"; shift 2 ;;
    --zone-left-m) ZONE_LEFT_M="$2"; shift 2 ;;
    --zone-radius-m) ZONE_RADIUS_M="$2"; shift 2 ;;
    --report) REPORT_PATH="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Opcion desconocida: $1" >&2; usage >&2; exit 2 ;;
  esac
done

step() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
ok() { printf '    \033[32mOK\033[0m  %s\n' "$*"; }
warn() { printf '    \033[33m!\033[0m   %s\n' "$*"; }
die() { printf '\n\033[31mError:\033[0m %s\n' "$*" >&2; exit 1; }

in_container() {
  docker exec -i "${CONTAINER}" bash -c "
    source /opt/ros/humble/setup.bash >/dev/null 2>&1
    source /ros2_ws/install/setup.bash >/dev/null 2>&1
    export RMW_IMPLEMENTATION='${RMW_VALUE}'
    $1"
}

wait_for_match() {
  local description="$1"
  local command="$2"
  local pattern="$3"
  local deadline=$((SECONDS + BOOT_TIMEOUT_S))
  until in_container "${command}" 2>/dev/null | grep -q "${pattern}"; do
    (( SECONDS < deadline )) || die "timeout esperando ${description}"
    sleep 2
  done
  ok "${description}"
}

docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}" \
  || die "el contenedor ${CONTAINER} no esta corriendo; usar ./tools/up-salus.sh"

if [[ "${RESTART_SIM}" == "true" ]]; then
  step "Reiniciando sim_global_v2"
  "${SCRIPT_DIR}/stop_sim_global_v2.sh" >/dev/null 2>&1 || true
  LAUNCH_RVIZ=false "${SCRIPT_DIR}/launch_sim_global_v2.sh" >/dev/null 2>&1 \
    || die "fallo sim_global_v2; revisar /ros2_ws/logs/sim_global_v2.log"
else
  step "Usando sim_global_v2 actual"
fi

wait_for_match "route_executor disponible" \
  "ros2 node list" '^/route_executor$'
wait_for_match "zones_manager disponible" \
  "ros2 node list" '^/zones_manager$'
wait_for_match "web_zone_server disponible" \
  "ros2 node list" '^/web_zone_server$'
wait_for_match "Nav2 aceptando rutas" \
  "ros2 action list" 'navigate_through_poses'
wait_for_match "servicio de recarga keepout disponible" \
  "ros2 service list" '^/keepout_filter_mask_server/load_map$'
wait_for_match "servidor de mascara keepout activo" \
  "ros2 lifecycle get /keepout_filter_mask_server" '^active'
wait_for_match "metadata keepout publicando" \
  "ros2 topic list" '^/costmap_filter_info$'
wait_for_match "/gps/fix publicando" \
  "ros2 topic list" '^/gps/fix$'

step "Parametros efectivos"
NOGO_PARAM="$(in_container 'ros2 param get /route_executor coverage_nogo_enabled' | tail -1)"
SKIP_PARAM="$(in_container 'ros2 param get /route_executor coverage_allow_row_skipping' | tail -1)"
ROUTE_PARAM="$(in_container 'ros2 param get /route_executor coverage_f2c_route_type' | tail -1)"
REVERSE_PARAM="$(in_container 'ros2 param get /route_executor coverage_f2c_allow_reverse' | tail -1)"
printf '    coverage_nogo_enabled       -> %s\n' "${NOGO_PARAM}"
printf '    coverage_allow_row_skipping -> %s\n' "${SKIP_PARAM}"
printf '    coverage_f2c_route_type     -> %s\n' "${ROUTE_PARAM}"
printf '    coverage_f2c_allow_reverse  -> %s\n' "${REVERSE_PARAM}"
[[ "${NOGO_PARAM}" == *"True"* ]] || die "coverage_nogo_enabled no esta activo"
[[ "${SKIP_PARAM}" == *"False"* ]] || die "todavia se permite saltar filas"
[[ "${ROUTE_PARAM}" == *"BOUSTROPHEDON"* ]] || die "route_type no es BOUSTROPHEDON"
# La politica forward-only es lo que garantiza que no aparezca coverage_backup:
# si estuviera en True, la prueba mediria otra cosa.
[[ "${REVERSE_PARAM}" == *"False"* ]] \
  || die "coverage_f2c_allow_reverse no esta en False: la simulacion no es forward-only"

# El planner y el controlador tienen que acompanar: Dubins no tiene primitivas
# hacia atras y el RPP no puede seguir un path con reversa.
MOTION_MODEL="$(in_container 'ros2 param get /planner_server GridBased.motion_model_for_search' | tail -1)"
ALLOW_REVERSING="$(in_container 'ros2 param get /controller_server FollowPath.allow_reversing' | tail -1)"
printf '    planner motion_model        -> %s\n' "${MOTION_MODEL}"
printf '    controller allow_reversing  -> %s\n' "${ALLOW_REVERSING}"
[[ "${MOTION_MODEL}" == *"DUBIN"* ]] || die "el planner de Nav2 no esta en DUBIN"
[[ "${ALLOW_REVERSING}" == *"False"* ]] || die "el controlador permite reversa"

if [[ "${OPEN_RVIZ}" == "true" ]]; then
  step "RViz"
  if docker exec "${CONTAINER}" pgrep -f 'rviz_sim_global_v2.launch.py' >/dev/null 2>&1; then
    ok "ya estaba abierto"
  elif ros2_prepare_gui; then
    ros2_build_docker_exec_env
    docker exec -d "${ros2_docker_exec_env[@]}" "${CONTAINER}" bash -lc "
      export RMW_IMPLEMENTATION='${RMW_VALUE}'
      source /opt/ros/humble/setup.bash
      source /ros2_ws/install/setup.bash
      ros2 launch navegacion_gps rviz_sim_global_v2.launch.py \
        >/ros2_ws/logs/coverage_nogo_rviz.log 2>&1"
    ok "abierto"
  else
    warn "no se pudo preparar DISPLAY; la prueba sigue sin RViz"
  fi
fi

step "Prueba de recorrido ${SCENARIO}"
DRIVER_ARGS=(
  --scenario "${SCENARIO}"
  --field-length-m "${FIELD_LENGTH_M}"
  --field-width-m "${FIELD_WIDTH_M}"
  --mission-timeout-s "${MISSION_TIMEOUT_S}"
  --report "${REPORT_PATH}"
)
[[ "${PREVIEW_ONLY}" == "true" ]] && DRIVER_ARGS+=(--preview-only)
[[ "${KEEP_TEST_ZONE}" == "true" ]] && DRIVER_ARGS+=(--keep-test-zone)
[[ -n "${ZONE_FORWARD_M}" ]] && DRIVER_ARGS+=(--zone-forward-m "${ZONE_FORWARD_M}")
[[ -n "${ZONE_LEFT_M}" ]] && DRIVER_ARGS+=(--zone-left-m "${ZONE_LEFT_M}")
DRIVER_ARGS+=(--zone-radius-m "${ZONE_RADIUS_M}")

docker exec -i "${CONTAINER}" bash -c '
  source /opt/ros/humble/setup.bash >/dev/null 2>&1
  source /ros2_ws/install/setup.bash >/dev/null 2>&1
  export RMW_IMPLEMENTATION="$1"
  shift
  exec python3 -u "$@"
' bash "${RMW_VALUE}" \
  /ros2_ws/src/navegacion_gps/test/manual_coverage_nogo_route.py \
  "${DRIVER_ARGS[@]}"

step "Reporte"
HOST_REPORT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)/artifacts"
mkdir -p "${HOST_REPORT_DIR}"
HOST_REPORT="${HOST_REPORT_DIR}/$(basename "${REPORT_PATH}")"
docker exec -i "${CONTAINER}" cat "${REPORT_PATH}" > "${HOST_REPORT}" \
  || die "no se pudo leer el reporte ${REPORT_PATH}"

# Ultima barrera, sobre el JSON ya escrito. El driver ya falla ante la primera
# reversa; esto ademas deja el veredicto legible sin abrir el archivo, y atrapa
# el caso de un driver que devolviera 0 con metricas malas.
python3 - "${HOST_REPORT}" <<'PYEOF'
import json
import sys

reporte = json.load(open(sys.argv[1], encoding="utf-8"))
velocidades = reporte.get("speeds") or {}
minima = velocidades.get("min_linear_velocity_mps")
backups = int(reporte.get("coverage_backup_action_count") or 0)
solo_preview = reporte.get("result") == "preview validated"
print("    estado final                -> %s" % reporte.get("final_status"))
print("    filas                       -> %s" % reporte.get("row_count"))
print("    salto maximo entre filas    -> %s" % reporte.get("max_row_jump"))
print("    distancia minima a la zona  -> %.3f m"
      % float(reporte.get("min_distance_to_nogo_m") or 0.0))
print("    velocidad lineal minima     -> %s m/s" % minima)
print("      /cmd_vel(_final)          -> %s m/s" % velocidades.get("min_cmd_vel_mps"))
print("      /controller/drive_telem.  -> %s m/s"
      % velocidades.get("min_drive_telemetry_mps"))
print("      /odometry/global          -> %s m/s" % velocidades.get("min_odometry_mps"))
print("    reversa pedida por el drive -> %s"
      % velocidades.get("telemetry_reverse_requested_count"))
print("    retroceso integrado peor    -> %s m (tolerancia %s m)"
      % (velocidades.get("odometry_backward_worst_distance_m"),
         velocidades.get("odometry_backward_distance_tolerance_m")))
print("    acciones coverage_backup    -> %d" % backups)
print("    waypoints fuera del lote    -> %s"
      % reporte.get("outside_field_waypoint_count"))
print("    muestras fuera del lote     -> %s" % reporte.get("outside_field_track_count"))

fallos = []
if not reporte.get("ok"):
    fallos.append("la prueba no termino OK: %s" % reporte.get("error"))
if backups:
    fallos.append("aparecieron %d acciones coverage_backup" % backups)
# Comando negativo o reversa pedida por el drive: inequivoco, falla al primero.
if int(velocidades.get("reverse_sample_count") or 0):
    fallos.append("se observaron %d comandos o mediciones de marcha atras"
                  % velocidades["reverse_sample_count"])
if int(velocidades.get("telemetry_reverse_requested_count") or 0):
    fallos.append("el drive reporto reversa %d veces"
                  % velocidades["telemetry_reverse_requested_count"])
umbral = float(velocidades.get("reverse_threshold_mps") or -0.02)
for nombre, clave in (
    ("comando", "min_cmd_vel_mps"),
    ("drive", "min_drive_telemetry_mps"),
):
    valor = velocidades.get(clave)
    if valor is not None and float(valor) < umbral:
        fallos.append("velocidad minima de %s %.3f m/s, por debajo de %.3f m/s"
                      % (nombre, float(valor), umbral))
# La estimacion global se juzga por distancia: su ruido baja del umbral
# instantaneo con el vehiculo quieto.
peor = velocidades.get("odometry_backward_worst_distance_m")
tolerancia = float(velocidades.get("odometry_backward_distance_tolerance_m") or 0.25)
if peor is not None and float(peor) > tolerancia:
    fallos.append("retroceso integrado de %.3f m, sobre la tolerancia de %.3f m"
                  % (float(peor), tolerancia))
if minima is None and not solo_preview:
    fallos.append("no se midio ninguna velocidad lineal")
if int(reporte.get("max_row_jump") or 0) > 1:
    fallos.append("el plan salto filas (salto maximo %s)" % reporte.get("max_row_jump"))
if fallos:
    for fallo in fallos:
        print("    FALLO: %s" % fallo, file=sys.stderr)
    raise SystemExit(1)
PYEOF

ok "escenario ${SCENARIO} finalizado"
echo "    reporte (contenedor): ${REPORT_PATH}"
echo "    reporte (host):       ${HOST_REPORT}"

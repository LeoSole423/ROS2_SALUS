#!/usr/bin/env bash
# Lanza el recorrido CAMPO (cobertura tipo cortadora) por la operacion
# start_coverage del web_zone_server.
#
# No usa run_coverage_waypoints.sh: ese CLI manda la curva muestreada como metas
# y conserva el comportamiento viejo. start_coverage manda la ruta ejecutable:
# las 2*N metas key, dos por pasada, sin puntos intermedios en la cabecera.
#
# Uso tipico:
#   ./tools/run_campo.sh --restart-sim --watch   # de cero, siguiendo la corrida
#   ./tools/run_campo.sh --preview               # solo calcula, no mueve nada
#   ./tools/run_campo.sh                         # lanza sobre la sim que ya corre
#   ./tools/run_campo.sh --cancel                # corta la mision activa

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER="${ROS2_CONTAINER_NAME:-ros2_salus}"
RMW="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
BOOT_TIMEOUT_S="${BOOT_TIMEOUT_S:-180}"

FIELD_LENGTH_M="40"
FIELD_WIDTH_M="20"
CUTTER_WIDTH_M="5"
OVERLAP_RATIO="0"
# El Smac de sim_global_v2 traza a 2.9 m: pedir mas grande achica el
# recorrido nominal a pura cabecera sin que el vehiculo lo necesite.
MIN_TURNING_RADIUS_M="2.9"
SIDE="left"
WAYPOINT_SPACING_M="2.0"

RESTART_SIM="false"
LAUNCH_RVIZ_OPT="false"
PREVIEW_ONLY="false"
WATCH="false"
CANCEL_ONLY="false"
SIM_ONLY="false"

# Nodos que stop_sim_global_v2.sh no mata. Si quedan vivos de un lanzamiento
# anterior, conviven varias copias publicando en los mismos topics y la
# simulacion parece correr codigo viejo.
LEAKED_NODES='scan_noise_filter|nav_observability|nav_trace_recorder|path_clearance_validator|polygon_stamped_republisher|ros_gz_bridge'

usage() {
  cat <<'EOF'
Uso: ./tools/run_campo.sh [opciones]

Geometria del lote
  --field-length-m N        Largo del lote            (default 40)
  --field-width-m N         Ancho del lote            (default 20)
  --cutter-width-m N        Ancho util de corte       (default 5)
  --overlap-ratio N         Solape entre pasadas      (default 0)
  --min-turning-radius-m N  Radio minimo Ackermann    (default 2.9)
  --side left|right         Hacia donde crecen las filas (default left)

Que hacer
  --solo-sim      Deja la simulacion lista y sale; no genera campo ni mueve nada
  --restart-sim   Baja la simulacion, limpia los nodos colgados y la levanta de cero
  --rviz          Con --restart-sim o --solo-sim, ademas abre RViz
  --preview       Solo calcula y muestra el plan; no mueve el vehiculo
  --watch         Sigue /odometry/global hasta que termina la mision
  --cancel        Cancela la mision activa y sale
  -h, --help      Esta ayuda

El lote se genera a partir de la pose actual del vehiculo: la esquina es donde
esta parado y las filas crecen hacia --side respecto de su rumbo. Con 5 m entre
pasadas y radio 4.0 m los giros son omega y salen del cuadrado; eso es correcto.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --field-length-m) FIELD_LENGTH_M="$2"; shift 2 ;;
    --field-width-m) FIELD_WIDTH_M="$2"; shift 2 ;;
    --cutter-width-m) CUTTER_WIDTH_M="$2"; shift 2 ;;
    --overlap-ratio) OVERLAP_RATIO="$2"; shift 2 ;;
    --min-turning-radius-m) MIN_TURNING_RADIUS_M="$2"; shift 2 ;;
    --side) SIDE="$2"; shift 2 ;;
    --waypoint-spacing-m) WAYPOINT_SPACING_M="$2"; shift 2 ;;
    --restart-sim) RESTART_SIM="true"; shift ;;
    --solo-sim) SIM_ONLY="true"; RESTART_SIM="true"; shift ;;
    --rviz) LAUNCH_RVIZ_OPT="true"; shift ;;
    --preview) PREVIEW_ONLY="true"; shift ;;
    --watch) WATCH="true"; shift ;;
    --cancel) CANCEL_ONLY="true"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Opcion desconocida: $1" >&2; usage >&2; exit 2 ;;
  esac
done

step() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
ok()   { printf '    \033[32mOK\033[0m  %s\n' "$*"; }
warn() { printf '    \033[33m!\033[0m   %s\n' "$*"; }
die()  { printf '\n\033[31mError:\033[0m %s\n' "$*" >&2; exit 1; }

# bash -c y no -lc: el profile de login del contenedor imprime diagnosticos que
# se mezclarian con la salida de este script. El entorno de ROS se toma de los
# setup.bash, que es lo unico que hace falta.
in_container() {
  docker exec -i "${CONTAINER}" bash -c "
    source /opt/ros/humble/setup.bash >/dev/null 2>&1
    source /ros2_ws/install/setup.bash >/dev/null 2>&1
    export RMW_IMPLEMENTATION='${RMW}'
    $1"
}

docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}" \
  || die "el contenedor ${CONTAINER} no esta corriendo (./tools/up-salus.sh)"

# ------------------------------------------------------------------- cancelar
if [[ "${CANCEL_ONLY}" == "true" ]]; then
  step "Cancelando la mision activa"
  in_container 'ros2 service call /route_executor/cancel_route interfaces/srv/CancelRouteMission "{}"' \
    | tail -3
  exit 0
fi

# ------------------------------------------------------------ reinicio limpio
if [[ "${RESTART_SIM}" == "true" ]]; then
  step "Bajando la simulacion"
  "${SCRIPT_DIR}/stop_sim_global_v2.sh" >/dev/null 2>&1 || true

  step "Limpiando nodos colgados de lanzamientos anteriores"
  leaked="$(docker exec "${CONTAINER}" bash -c \
    "ps -eo pid=,args= | grep -E '${LEAKED_NODES}' | grep -v grep | awk '{print \$1}'" || true)"
  if [[ -n "${leaked}" ]]; then
    count="$(printf '%s\n' "${leaked}" | wc -l | tr -d ' ')"
    docker exec "${CONTAINER}" bash -c "kill -9 ${leaked//$'\n'/ } 2>/dev/null; true"
    sleep 2
    warn "eliminados ${count} procesos que stop_sim_global_v2.sh no cubre"
  else
    ok "no quedaba ninguno"
  fi

  step "Levantando sim_global_v2 (Gazebo tarda ~30 s)"
  LAUNCH_RVIZ="${LAUNCH_RVIZ_OPT}" "${SCRIPT_DIR}/launch_sim_global_v2.sh" >/dev/null 2>&1 \
    || die "fallo el launch; ver 'docker exec ${CONTAINER} cat /ros2_ws/logs/sim_global_v2.log'"

  deadline=$(( SECONDS + BOOT_TIMEOUT_S ))
  until in_container 'ros2 node list 2>/dev/null' | grep -qx '/route_executor'; do
    (( SECONDS < deadline )) || die "la simulacion no levanto en ${BOOT_TIMEOUT_S}s"
    sleep 3
  done
  in_container 'ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1' || true
  sleep 3
  ok "route_executor arriba"

  deadline=$(( SECONDS + BOOT_TIMEOUT_S ))
  until in_container 'ros2 topic list 2>/dev/null' | grep -qx '/odometry/global'; do
    (( SECONDS < deadline )) || die "no aparecio /odometry/global"
    sleep 2
  done
  ok "/odometry/global publicando"

  # Nav2 tarda mas que el route_executor en activar el arbol de comportamiento.
  # Sin esta espera start_coverage se rechaza con "NavigateThroughPoses action
  # server not available" y hay que reintentar a mano.
  deadline=$(( SECONDS + BOOT_TIMEOUT_S ))
  until in_container 'ros2 action list 2>/dev/null' | grep -q 'navigate_through_poses'; do
    (( SECONDS < deadline )) || die "Nav2 no publico la accion navigate_through_poses"
    sleep 3
  done
  ok "Nav2 aceptando NavigateThroughPoses"
fi

# --------------------------------------------------------------- diagnostico
step "Estado de la simulacion"
dupes="$(in_container 'ros2 node list 2>/dev/null' \
  | sort | uniq -c | awk '$1 > 1 && $2 != "/ros_gz_bridge" {print $1" "$2}' || true)"
if [[ -n "${dupes}" ]]; then
  warn "hay nodos duplicados; la sim va a comportarse de forma erratica:"
  printf '        %s\n' ${dupes}
  warn "correr de nuevo con --restart-sim"
else
  ok "un solo nodo de cada uno"
fi

# La guia de cabecera tiene que estar apagada: partir el giro en dos metas deja
# un tramo de radio minimo exacto que Smac cierra con una vuelta completa de 25 m.
param_out="$(in_container 'ros2 param get /web_zone_server coverage_use_headland_guides 2>&1' | tail -1)"
echo "    coverage_use_headland_guides -> ${param_out}"
[[ "${param_out}" == *"False"* ]] \
  || die "las guias de cabecera estan prendidas; agregan una vuelta completa por giro"

if [[ "${SIM_ONLY}" == "true" ]]; then
  cat <<EOF

  Simulacion lista. No se genero campo ni se movio el vehiculo.

  Cockpit          : http://localhost:5173/   (si no corre: cd cockpit && npm run dev)
  Lanzar cobertura : ./tools/run_campo.sh
  Bajar todo       : ./tools/stop_sim_global_v2.sh

EOF
  # Con --watch se sigue de largo al seguidor de odometria; sin el, aca termina.
  [[ "${WATCH}" == "true" ]] || exit 0
fi

# ------------------------------------------------------- plan / arranque real
if [[ "${SIM_ONLY}" != "true" ]]; then
if [[ "${PREVIEW_ONLY}" == "true" ]]; then
  OP="preview_coverage"
  step "Calculando el plan (no se mueve el vehiculo)"
else
  OP="start_coverage"
  step "Lanzando la cobertura"
fi

# Directorio propio y no /tmp a secas: hay un /tmp/inspect.py viejo en el
# contenedor que tapa el modulo estandar 'inspect' para cualquier script que se
# ejecute desde ahi, porque Python antepone la carpeta del script a sys.path.
docker exec -i "${CONTAINER}" bash -c "mkdir -p /tmp/run_campo"

docker exec -i "${CONTAINER}" bash -c \
  "cat > /tmp/run_campo/ws.py" <<'PYEOF'
import asyncio
import json
import sys

import websockets

op = sys.argv[1]
msg = {
    "op": op,
    "client_req_id": "run_campo",
    "field_length_m": float(sys.argv[2]),
    "field_width_m": float(sys.argv[3]),
    "cutter_width_m": float(sys.argv[4]),
    "overlap_ratio": float(sys.argv[5]),
    "min_turning_radius_m": float(sys.argv[6]),
    "waypoint_spacing_m": float(sys.argv[7]),
    "side": sys.argv[8],
}


async def main() -> int:
    async with websockets.connect("ws://127.0.0.1:8766", max_size=64 * 1024 * 1024) as ws:
        await ws.send(json.dumps(msg))
        while True:
            data = json.loads(await asyncio.wait_for(ws.recv(), timeout=120))
            if data.get("op") != "ack" or data.get("client_req_id") != "run_campo":
                continue

            if not data.get("ok"):
                print("    RECHAZADO: %s" % (data.get("error") or "sin detalle"))
                return 1

            plan = data.get("coverage_plan") or {}
            metrics = plan.get("metrics") or {}
            route = (plan.get("route_request") or {}).get("waypoints") or []
            pattern = "".join("K" if wp["key"] else "G" for wp in route)

            print("    filas              : %s   orden %s"
                  % (metrics.get("row_count"), metrics.get("row_visit_order")))
            print("    separacion pasadas : %.2f m" % float(metrics.get("lane_spacing_m", 0.0)))
            print("    giros              : %s omega, %s U limpias"
                  % (metrics.get("omega_turn_count"), metrics.get("clean_uturn_count")))
            print("    sin cruces         : %s (%s)"
                  % (plan.get("topology_safe"), metrics.get("topology_scope")))
            print("    ruta               : %d waypoints   %s" % (len(route), pattern))
            print("    recorrido estimado : %.1f m"
                  % float(metrics.get("estimated_path_length_m", 0.0)))

            if op == "start_coverage":
                print("    metas key          : %s" % data.get("input_key_waypoint_count"))
                print("    guias de cabecera  : %s" % data.get("guide_waypoint_count"))
                print("    mision arrancada   : %s" % data.get("route_started"))
            return 0


sys.exit(asyncio.run(main()))
PYEOF

set +e
in_container "python3 /tmp/run_campo/ws.py '${OP}' '${FIELD_LENGTH_M}' '${FIELD_WIDTH_M}' \
  '${CUTTER_WIDTH_M}' '${OVERLAP_RATIO}' '${MIN_TURNING_RADIUS_M}' \
  '${WAYPOINT_SPACING_M}' '${SIDE}'"
rc=$?
set -e
(( rc == 0 )) || die "la operacion ${OP} fallo"

if [[ "${PREVIEW_ONLY}" == "true" ]]; then
  step "--preview: no se envio nada al vehiculo"
  exit 0
fi

ok "ruta activa"

cat <<EOF

  Seguir la corrida : ./tools/run_campo.sh --watch
  Cortar la mision  : ./tools/run_campo.sh --cancel
  Cockpit           : http://localhost:5173/  -> modulo nav2

EOF

fi   # fin del bloque de cobertura (se saltea con --solo-sim)

# ------------------------------------------------------------------- seguimiento
if [[ "${WATCH}" != "true" ]]; then
  exit 0
fi

step "Siguiendo /odometry/global (Ctrl-C para salir)"

docker exec -i "${CONTAINER}" bash -c "cat > /tmp/run_campo/watch.py" <<'PYEOF'
import math

import rclpy
from interfaces.msg import NavEvent
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class Watch(Node):
    """Imprime la pose una vez por segundo y cada evento del route_executor."""

    def __init__(self) -> None:
        super().__init__("run_campo_watch")
        self._last = 0.0
        self._min_vx = 0.0
        self.create_subscription(
            Odometry,
            "/odometry/global",
            self._on_odom,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.create_subscription(NavEvent, "/nav_command_server/events", self._on_event, 50)

    def _on_odom(self, msg: Odometry) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last < 1.0:
            return
        self._last = now
        pose = msg.pose.pose
        q = pose.orientation
        yaw = math.degrees(math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * q.z * q.z))
        vx = msg.twist.twist.linear.x
        self._min_vx = min(self._min_vx, vx)
        print(
            "    x=%8.2f  y=%8.2f  rumbo=%7.1f  v=%5.2f m/s" % (pose.position.x, pose.position.y, yaw, vx),
            flush=True,
        )

    def _on_event(self, msg: NavEvent) -> None:
        if msg.component != "route_executor":
            return
        details = {item.key: item.value for item in msg.details}
        if msg.code == "ROUTE_CHUNK_REQUESTED":
            print(
                "  >> chunk %s: waypoints %s..%s (%s poses)"
                % (
                    details.get("chunk_id"),
                    details.get("start_index"),
                    details.get("target_index"),
                    details.get("waypoint_count"),
                ),
                flush=True,
            )
        elif msg.code.startswith("ROUTE_MISSION_"):
            print("  >> %s" % msg.code, flush=True)
            if msg.code != "ROUTE_MISSION_STARTED":
                print("  >> velocidad minima observada: %.3f m/s" % self._min_vx, flush=True)
                raise SystemExit(0)


rclpy.init()
try:
    rclpy.spin(Watch())
except (KeyboardInterrupt, SystemExit):
    pass
PYEOF

in_container 'python3 /tmp/run_campo/watch.py' || true

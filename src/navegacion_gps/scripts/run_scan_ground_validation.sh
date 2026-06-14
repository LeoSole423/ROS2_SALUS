#!/usr/bin/env bash
# Validación A/B del scan_ground_filter en la rampa (slope_lidar.world).
#
# Corre el escenario dos veces — baseline (sin filtro) y con filtro — y compara
# los KPIs (FP en costmap local, eventos de freno falsos). Cada corrida levanta
# el stack de sim, mide `DURATION` segundos y escribe un JSON.
#
# Uso:
#   ./run_scan_ground_validation.sh [DURATION_S] [OUTDIR]
#
# Pre-requisitos: workspace compilado y sourceado (install/setup.bash), Gazebo
# disponible. Pensado para correr dentro del contenedor de sim.
#
# NOTA: los eventos de freno solo se cuentan si el robot avanza; para medirlos,
# mandá una meta con nav_command_server durante la ventana (la corrida sola mide
# FP estáticos, que es donde el filtro muestra el efecto dominante).
set -euo pipefail

DURATION="${1:-60}"
OUTDIR="${2:-/tmp/scan_ground_validation}"
mkdir -p "$OUTDIR"

run_case() {
  local label="$1" enable="$2" out="$3"
  echo "=== Corrida '${label}' (enable_scan_ground_filter:=${enable}) ==="
  # margen extra sobre DURATION para arranque de Gazebo/Nav2
  local timeout_s=$((DURATION + 60))
  timeout "${timeout_s}" ros2 launch navegacion_gps validate_scan_ground.launch.py \
    enable_scan_ground_filter:="${enable}" \
    label:="${label}" \
    duration_s:="${DURATION}" \
    output_path:="${out}" \
    use_rviz:=False || true
  if [[ -f "$out" ]]; then
    echo "--- ${label}:"; cat "$out"; echo
  else
    echo "ADVERTENCIA: no se generó ${out}"
  fi
}

run_case baseline False "${OUTDIR}/baseline.json"
sleep 3
run_case filtered True "${OUTDIR}/filtered.json"

echo "=== Comparación ==="
python3 - "$OUTDIR/baseline.json" "$OUTDIR/filtered.json" <<'PY'
import json, sys
def load(p):
    try:
        with open(p) as f: return json.load(f)
    except OSError:
        return None
b, f = load(sys.argv[1]), load(sys.argv[2])
if not b or not f:
    print("Falta algún reporte; no se puede comparar."); sys.exit(0)
def row(k, fmt="{}"):
    bv, fv = b.get(k, 0), f.get(k, 0)
    delta = ""
    if isinstance(bv, (int, float)) and bv:
        delta = f"  ({(fv-bv)/bv*100:+.1f}%)"
    print(f"{k:22} baseline={fmt.format(bv):>12}  filtered={fmt.format(fv):>12}{delta}")
print()
for k in ("costmap_frames", "fp_accumulated", "fp_mean_per_frame", "fp_max_frame",
          "slowdown_events", "stop_events"):
    row(k)
PY

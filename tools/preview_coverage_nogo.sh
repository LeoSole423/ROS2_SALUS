#!/usr/bin/env bash
# Levanta una simulacion limpia, carga un lote y una zona no-go, valida el
# preview y deja todos los nodos corriendo para inspeccion manual en RViz.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCENARIO="inside"
FIELD_LENGTH_M="36.0"
FIELD_WIDTH_M="23.0"
ZONE_FORWARD_M="14.0"
ZONE_LEFT_M=""
ZONE_RADIUS_M="1.5"
OPEN_RVIZ="true"
RESTORE_ZONES="false"

usage() {
  cat <<EOF
Uso: $0 [opciones]

Levanta sim_global_v2 desde cero, carga el lote rectangular como poligono y
una zona no-go circular, valida solamente el preview y deja la simulacion viva.

  --scenario inside|boundary  Zona interna o pegada al borde (default inside).
  --field-length-m N           Largo del lote (default ${FIELD_LENGTH_M} m).
  --field-width-m N            Ancho del lote (default ${FIELD_WIDTH_M} m).
  --zone-forward-m N           Posicion longitudinal de la zona (default ${ZONE_FORWARD_M} m).
  --zone-left-m N              Posicion lateral; por defecto centro interno o borde boundary.
  --zone-radius-m N            Radio de la zona (default ${ZONE_RADIUS_M} m).
  --no-rviz                    No abrir RViz.
  --restore-zones              Restaurar el GeoJSON anterior al terminar el preview.
  -h, --help                   Mostrar esta ayuda.

La simulacion no se detiene al terminar. Por defecto tampoco se restauran las
zonas, para que queden visibles en RViz y el cockpit.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --scenario)
      SCENARIO="$2"; shift 2 ;;
    --field-length-m)
      FIELD_LENGTH_M="$2"; shift 2 ;;
    --field-width-m)
      FIELD_WIDTH_M="$2"; shift 2 ;;
    --zone-forward-m)
      ZONE_FORWARD_M="$2"; shift 2 ;;
    --zone-left-m)
      ZONE_LEFT_M="$2"; shift 2 ;;
    --zone-radius-m)
      ZONE_RADIUS_M="$2"; shift 2 ;;
    --no-rviz)
      OPEN_RVIZ="false"; shift ;;
    --restore-zones)
      RESTORE_ZONES="true"; shift ;;
    -h|--help)
      usage; exit 0 ;;
    *)
      echo "Opcion desconocida: $1" >&2
      usage >&2
      exit 2 ;;
  esac
done

[[ "${SCENARIO}" == "inside" || "${SCENARIO}" == "boundary" ]] || {
  echo "--scenario debe ser inside o boundary" >&2
  exit 2
}

ARGS=(
  --no-rviz
  --preview-only
  --field-length-m "${FIELD_LENGTH_M}"
  --field-width-m "${FIELD_WIDTH_M}"
  --zone-forward-m "${ZONE_FORWARD_M}"
  --zone-radius-m "${ZONE_RADIUS_M}"
)
[[ "${OPEN_RVIZ}" == "true" ]] && ARGS=(
  --preview-only
  --field-length-m "${FIELD_LENGTH_M}"
  --field-width-m "${FIELD_WIDTH_M}"
  --zone-forward-m "${ZONE_FORWARD_M}"
  --zone-radius-m "${ZONE_RADIUS_M}"
)
[[ -n "${ZONE_LEFT_M}" ]] && ARGS+=(--zone-left-m "${ZONE_LEFT_M}")
[[ "${RESTORE_ZONES}" == "false" ]] && ARGS+=(--keep-test-zone)

exec "${SCRIPT_DIR}/test_coverage_nogo_${SCENARIO}.sh" "${ARGS[@]}"

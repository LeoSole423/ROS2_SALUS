#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TRACE_DIR="${1:-}"
if [[ -z "${TRACE_DIR}" ]]; then
  TRACE_DIR="$(find "${REPO_ROOT}/artifacts/nav_traces" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -n 1 | cut -d' ' -f2-)"
fi
if [[ -z "${TRACE_DIR}" || ! -d "${TRACE_DIR}" ]]; then
  echo "Traza inexistente: ${TRACE_DIR:-<vacia>}" >&2
  exit 1
fi

TRACE_DIR="$(realpath "${TRACE_DIR}")"
case "${TRACE_DIR}" in
  "${REPO_ROOT}"/*) CONTAINER_TRACE="/ros2_ws/${TRACE_DIR#"${REPO_ROOT}/"}" ;;
  *) echo "La traza debe estar dentro de ${REPO_ROOT}" >&2; exit 1 ;;
esac

"${REPO_ROOT}/tools/exec.sh" "ros2 run navegacion_gps nav_trace_report '${CONTAINER_TRACE}'"

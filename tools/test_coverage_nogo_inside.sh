#!/usr/bin/env bash
# Zona circular completamente dentro del poligono: anticipa y sigue fila N+1.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/test_coverage_nogo_scenario.sh" inside "$@"


#!/usr/bin/env bash
# Zona circular apoyada en el borde: sale del lote, rodea y vuelve a entrar.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/test_coverage_nogo_scenario.sh" boundary "$@"


#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOG_DIR="${REPO_DIR}/log/jetson_power_monitor"

mkdir -p "${LOG_DIR}"
cd "${REPO_DIR}"

exec /usr/bin/python3 "${REPO_DIR}/tools/jetson_power_monitor.py" "$@"

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_SRC="${SCRIPT_DIR}/systemd/jetson-power-monitor.service"
SERVICE_DST="/etc/systemd/system/jetson-power-monitor.service"

if [[ ! -f "${SERVICE_SRC}" ]]; then
  echo "Service file not found: ${SERVICE_SRC}" >&2
  exit 1
fi

CURRENT_CRONTAB="$(mktemp)"
trap 'rm -f "${CURRENT_CRONTAB}"' EXIT

crontab -l 2>/dev/null | grep -Ev 'jetson_power_monitor\.py|run_jetson_power_monitor\.sh' > "${CURRENT_CRONTAB}" || true
crontab "${CURRENT_CRONTAB}" || true

sudo install -m 0644 "${SERVICE_SRC}" "${SERVICE_DST}"
sudo systemctl daemon-reload
sudo systemctl enable --now jetson-power-monitor.service
sudo systemctl status --no-pager jetson-power-monitor.service

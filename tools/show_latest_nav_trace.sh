#!/usr/bin/env bash
set -euo pipefail

ROOT="${1:-$(pwd)/artifacts/nav_traces}"
LATEST="$(find "${ROOT}" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -n 1 | cut -d' ' -f2-)"

if [[ -z "${LATEST}" ]]; then
  echo "No hay trazas en ${ROOT}" >&2
  exit 1
fi

echo "${LATEST}"
echo
cat "${LATEST}/summary.md"

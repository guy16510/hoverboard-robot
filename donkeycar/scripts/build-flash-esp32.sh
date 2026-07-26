#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PORT="${1:-${ESP32_PORT:-/dev/ttyUSB0}}"
cd "$ROOT"

[[ -x .venv/bin/pio ]] || {
  python3 -m venv .venv
  .venv/bin/pip install --disable-pip-version-check -r requirements-dev.txt
}

./tools/bootstrap-tools.sh
export PLATFORMIO_CORE_DIR="$ROOT/.platformio"
export PLATFORMIO_SETTING_ENABLE_TELEMETRY=no

.venv/bin/pio run -c platformio-drive.ini -e esp32_drive_coordinator
.venv/bin/pio run -c platformio-drive.ini -e esp32_drive_coordinator \
  -t upload --upload-port "$PORT"

printf 'Flashed esp32_drive_coordinator to %s\n' "$PORT"
printf 'Do not lower the drive wheels. Run donkeycar/scripts/preflight.py first.\n'

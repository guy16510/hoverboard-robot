#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "$0")/.." && pwd)"
cd "$repo_dir"

if [[ ! -x .venv/bin/pio ]]; then
  python3 -m venv .venv
  .venv/bin/pip install --disable-pip-version-check -r requirements-dev.txt
fi

export PLATFORMIO_CORE_DIR="$repo_dir/.platformio"
export PLATFORMIO_SETTING_ENABLE_TELEMETRY=no
.venv/bin/pio run -e esp32_winxu_drive

echo "ESP32 firmware: $repo_dir/.pio/build/esp32_winxu_drive/firmware.bin"

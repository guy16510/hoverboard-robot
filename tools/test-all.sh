#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-only
set -euo pipefail

repo_dir="$(cd "$(dirname "$0")/.." && pwd)"
cd "$repo_dir"
./tools/test-native.sh
./tools/test-sanitized.sh
./tools/test-esp32-balance.sh
node --test tools/pi-client/test/*.test.mjs
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m unittest tests/test_drive_esp32.py
PLATFORMIO_CORE_DIR="$repo_dir/.platformio" PLATFORMIO_SETTING_ENABLE_TELEMETRY=no \
  .venv/bin/pio run -e native_tests
.pio/build/native_tests/program

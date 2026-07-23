#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-only
set -euo pipefail

repo_dir="$(cd "$(dirname "$0")/.." && pwd)"
cd "$repo_dir"
./tools/bootstrap-tools.sh
export PLATFORMIO_CORE_DIR="$repo_dir/.platformio"
export PLATFORMIO_SETTING_ENABLE_TELEMETRY=no
export SOURCE_DATE_EPOCH="$(git show -s --format=%ct HEAD)"
targets=(gausstop_master gausstop_master_swd bench_master_pa4_bypass \
         gausstop_slave gausstop_safe_recovery \
         gausstop_communication_diagnostic esp32_coordinator \
         esp32_swd_coordinator esp32_passive_probe)
.venv/bin/pio run $(printf -- ' -e %s' "${targets[@]}")
for target in "${targets[@]}"; do
  ./tools/verify-image.sh "$target"
  ./tools/firmware-manifest.sh "$target"
done

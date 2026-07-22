#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-only
set -euo pipefail

repo_dir="$(cd "$(dirname "$0")/.." && pwd)"
cd "$repo_dir"
automatic_files=()
while IFS= read -r source; do
  automatic_files+=("$source")
done < <(rg --files .github tools -g '*.sh' -g '*.yml' -g '*.yaml' | \
  rg -v '^tools/hardware/|^tools/check-hardware-isolation\.sh$')
automatic_files+=(platformio.ini)
if rg -n '(pio[[:space:]]+device|pio[[:space:]]+run[^\n]*upload|lsusb|system_profiler[[:space:]]+USB|/dev/(cu|tty)|st-flash|OpenOCD|openocd|STM32_Programmer_CLI|chip_id|flash_id|write_flash|erase_flash)' "${automatic_files[@]}"; then
  echo "automatic build/test path contains a hardware-access command" >&2
  exit 1
fi

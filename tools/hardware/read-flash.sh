#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-only
set -euo pipefail

interface_cfg="${1:-}"
target_cfg="${2:-}"
output="${3:-}"
[[ -n "$interface_cfg" && -n "$target_cfg" && -n "$output" ]] || {
  echo "usage: $0 INTERFACE_CFG TARGET_CFG OUTPUT.bin" >&2
  exit 2
}
[[ ! -e "$output" ]] || { echo "refusing to overwrite $output" >&2; exit 2; }
read -r -p "Type READ 64K GAUSSTOP to continue: " confirmation
[[ "$confirmation" == "READ 64K GAUSSTOP" ]] || { echo "cancelled"; exit 1; }
openocd -f "$interface_cfg" -f "$target_cfg" \
  -c "adapter speed 100; reset_config srst_only srst_nogate connect_assert_srst; init; reset halt; dump_image $output 0x08000000 0x10000; shutdown"
[[ "$(wc -c < "$output" | tr -d ' ')" == "65536" ]] || { echo "read length mismatch" >&2; exit 1; }
shasum -a 256 "$output"


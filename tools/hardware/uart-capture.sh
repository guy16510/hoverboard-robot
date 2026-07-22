#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-only
set -euo pipefail

port="${1:-}"
baud="${2:-}"
output="${3:-}"
[[ -n "$port" && -n "$baud" && -n "$output" ]] || {
  echo "usage: $0 EXACT_PORT BAUD OUTPUT.log" >&2
  exit 2
}
[[ "$baud" =~ ^[0-9]+$ ]] || { echo "invalid baud" >&2; exit 2; }
[[ ! -e "$output" ]] || { echo "refusing to overwrite $output" >&2; exit 2; }
read -r -p "Type CAPTURE EXACT UART to open $port: " confirmation
[[ "$confirmation" == "CAPTURE EXACT UART" ]] || { echo "cancelled"; exit 1; }
python3 -m serial.tools.miniterm "$port" "$baud" --raw | tee "$output"


#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-only
set -euo pipefail

interface_cfg="${1:-}"
target_cfg="${2:-}"
image="${3:-}"
[[ -n "$interface_cfg" && -n "$target_cfg" && -f "$image" ]] || {
  echo "usage: $0 INTERFACE_CFG TARGET_CFG IMAGE.bin" >&2
  exit 2
}
image_size="$(wc -c < "$image" | tr -d ' ')"
[[ "$image_size" -le 65536 ]] || { echo "image exceeds 64 KiB" >&2; exit 1; }
read -r -p "Type FLASH AND VERIFY GAUSSTOP to continue: " confirmation
[[ "$confirmation" == "FLASH AND VERIFY GAUSSTOP" ]] || { echo "cancelled"; exit 1; }
readback="${image}.readback.bin"
[[ ! -e "$readback" ]] || { echo "refusing to overwrite $readback" >&2; exit 2; }
reset_mode="${GAUSSTOP_RESET_MODE:-srst}"
case "$reset_mode" in
  srst)
    openocd_command="adapter speed 100; reset_config srst_only srst_nogate connect_assert_srst; init; reset halt; program $image 0x08000000 verify; dump_image $readback 0x08000000 $image_size; shutdown"
    ;;
  none)
    openocd_command="adapter speed 100; reset_config none; init; halt; program $image 0x08000000 verify; dump_image $readback 0x08000000 $image_size; shutdown"
    ;;
  *)
    echo "invalid GAUSSTOP_RESET_MODE: $reset_mode (expected srst or none)" >&2
    exit 2
    ;;
esac
openocd -f "$interface_cfg" -f "$target_cfg" -c "$openocd_command"
cmp "$image" "$readback"
shasum -a 256 "$image" "$readback"

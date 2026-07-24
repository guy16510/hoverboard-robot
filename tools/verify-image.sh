#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-only
set -euo pipefail

target="${1:-}"
[[ -n "$target" ]] || { echo "usage: $0 TARGET" >&2; exit 2; }
repo_dir="$(cd "$(dirname "$0")/.." && pwd)"
build_dir="$repo_dir/.pio/build/$target"
[[ -f "$build_dir/firmware.elf" ]] || { echo "missing ELF for $target" >&2; exit 1; }
[[ -f "$build_dir/firmware.bin" ]] || { echo "missing BIN for $target" >&2; exit 1; }

case "$target" in
  gausstop_* | bench_master_pa4_bypass)
    bin_size="$(wc -c < "$build_dir/firmware.bin" | tr -d ' ')"
    [[ "$bin_size" -le 65536 ]] || { echo "$target exceeds 64 KiB flash" >&2; exit 1; }
    size_tool="$repo_dir/.toolchains/arm-none-eabi/bin/arm-none-eabi-size"
    ram_size="$($size_tool -A "$build_dir/firmware.elf" | awk '
      $1 == ".data" || $1 == ".bss" || $1 == ".noinit" ||
      $1 == ".heartbeat" || $1 == ".ramfunc" { ram += $2 }
      END { print ram }
    ')"
    [[ "$ram_size" -le 7104 ]] || { echo "$target exceeds practical static RAM limit" >&2; exit 1; }
    ;;
  esp32_*)
    [[ -f "$build_dir/bootloader.bin" ]]
    [[ -f "$build_dir/partitions.bin" ]]
    [[ -f "$build_dir/boot_app0.bin" ]]
    [[ "$(wc -c < "$build_dir/boot_app0.bin" | tr -d ' ')" == "8192" ]]
    ;;
  *)
    echo "unknown target: $target" >&2
    exit 2
    ;;
esac
echo "$target image verified"

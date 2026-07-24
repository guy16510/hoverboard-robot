#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-only
set -euo pipefail

target="${1:-}"
[[ -n "$target" ]] || { echo "usage: $0 TARGET" >&2; exit 2; }
repo_dir="$(cd "$(dirname "$0")/.." && pwd)"
build_dir="$repo_dir/.pio/build/$target"
dist_dir="$repo_dir/dist/$target"
mkdir -p "$dist_dir"

cp "$build_dir/firmware.bin" "$dist_dir/firmware.bin"
cp "$build_dir/firmware.elf" "$dist_dir/firmware.elf"
[[ -f "$build_dir/firmware.map" ]] && cp "$build_dir/firmware.map" "$dist_dir/firmware.map"
[[ -f "$build_dir/firmware.lst" ]] && cp "$build_dir/firmware.lst" "$dist_dir/firmware.lst"
for extra in bootloader.bin partitions.bin boot_app0.bin; do
  [[ -f "$build_dir/$extra" ]] && cp "$build_dir/$extra" "$dist_dir/$extra"
done

git_commit="$(git -C "$repo_dir" rev-parse HEAD)"
dirty="false"
[[ -n "$(git -C "$repo_dir" status --porcelain --untracked-files=no)" ]] && dirty="true"
pio_version="$($repo_dir/.venv/bin/pio --version | awk '{print $NF}')"
if [[ "$target" == gausstop_* || "$target" == bench_master_pa4_bypass ]]; then
  size_tool="$repo_dir/.toolchains/arm-none-eabi/bin/arm-none-eabi-size"
  toolchain_version="$($repo_dir/.toolchains/arm-none-eabi/bin/arm-none-eabi-gcc -dumpfullversion)"
  board="GAUSSTOP_DPHC-V3.3_GD32F130C8T6"
  # Count loadable flash and allocated SRAM sections explicitly. GNU size's
  # Berkeley summary classifies init/fini arrays as "data" even though this
  # linker layout keeps them in flash.
  memory_line="$($size_tool -A "$build_dir/firmware.elf" | awk '
    $1 == ".isr_vector" || $1 == ".text" || $1 == ".rodata" ||
    $1 == ".init_array" || $1 == ".fini_array" || $1 == ".ARM.extab" ||
    $1 == ".ARM.exidx" || $1 == ".data" || $1 == ".ramfunc" { flash += $2 }
    $1 == ".data" || $1 == ".bss" || $1 == ".noinit" ||
    $1 == ".heartbeat" || $1 == ".ramfunc" { ram += $2 }
    END { printf "flash_bytes=%d\nram_bytes=%d", flash, ram }
  ')"
else
  size_tool="$repo_dir/.toolchains/xtensa-esp32/bin/xtensa-esp32-elf-size"
  toolchain_version="$($repo_dir/.toolchains/xtensa-esp32/bin/xtensa-esp32-elf-gcc -dumpfullversion)"
  board="ESP32_DEV_GPIO17_TX_GPIO35_RX"
  # Match the Espressif PlatformIO size accounting. Only DRAM-backed data and
  # BSS consume runtime RAM; flash-mapped rodata must not be counted as RAM.
  memory_line="$($size_tool -A "$build_dir/firmware.elf" | awk '
    $1 == ".iram0.vectors" || $1 == ".iram0.text" ||
    $1 == ".dram0.data" || $1 == ".flash.rodata" ||
    $1 == ".flash.text" { flash += $2 }
    $1 == ".dram0.data" || $1 == ".dram0.bss" { ram += $2 }
    END { printf "flash_bytes=%d\nram_bytes=%d", flash, ram }
  ')"
fi

manifest="$dist_dir/manifest.txt"
{
  echo "target=$target"
  echo "git_commit=$git_commit"
  echo "git_dirty=$dirty"
  echo "primary_upstream_commit=fa089568a523d8a43887bf8925b0ddb077ebb6ae"
  echo "layout_upstream_commit=eefe281b25086c55ff35b0322dd1de2c31965652"
  echo "build_timestamp_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "platformio_version=$pio_version"
  echo "toolchain_version=$toolchain_version"
  echo "firmware_bin_bytes=$(wc -c < "$build_dir/firmware.bin" | tr -d ' ')"
  echo "elf_bytes=$(wc -c < "$build_dir/firmware.elf" | tr -d ' ')"
  printf '%s\n' "$memory_line"
  echo "board_definition=$board"
  echo "build_flags=release,warnings-as-errors,function-sections,gc-sections"
  for artifact in "$dist_dir"/*.bin "$dist_dir"/*.elf; do
    [[ -f "$artifact" ]] || continue
    echo "sha256_$(basename "$artifact")=$(shasum -a 256 "$artifact" | awk '{print $1}')"
  done
} > "$manifest"
echo "$manifest"

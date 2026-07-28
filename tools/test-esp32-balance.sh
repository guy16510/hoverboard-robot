#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-only
set -euo pipefail

repo_dir="$(cd "$(dirname "$0")/.." && pwd)"
build_dir="${TMPDIR:-/tmp}/gausstop-esp32-balance-tests"
compiler="${CXX:-clang++}"
c_compiler="${CC:-clang}"
mkdir -p "$build_dir"
cd "$repo_dir"

sources=(
  tests/esp32/test_balance.cpp
  firmware/esp32/control/*.cpp
)
common_flags=(
  -std=c++17
  -Wall
  -Wextra
  -Wpedantic
  -Werror
  -Ifirmware/esp32/control
  -Ifirmware/gd32/common/protocol
  -Itests/esp32
)

"$compiler" "${common_flags[@]}" \
  "${sources[@]}" \
  -o "$build_dir/esp32_balance_tests"
"$build_dir/esp32_balance_tests"

"$compiler" "${common_flags[@]}" \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  -fno-sanitize-recover=all \
  "${sources[@]}" \
  -o "$build_dir/esp32_balance_tests_sanitized"
"$build_dir/esp32_balance_tests_sanitized"

safety_sources=(
  tests/esp32/test_drive_safety.cpp
  firmware/esp32/control/drive_safety.cpp
)

"$compiler" "${common_flags[@]}" \
  "${safety_sources[@]}" \
  -o "$build_dir/esp32_drive_safety_tests"
"$build_dir/esp32_drive_safety_tests"

"$compiler" "${common_flags[@]}" \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  -fno-sanitize-recover=all \
  "${safety_sources[@]}" \
  -o "$build_dir/esp32_drive_safety_tests_sanitized"
"$build_dir/esp32_drive_safety_tests_sanitized"

integration_includes=(
  -Ifirmware/esp32/control
  -Ifirmware/gd32/common/protocol
  -Ifirmware/gd32/common/control
  -Ifirmware/gd32/common/safety
  -Ifirmware/gd32/common/coordination
)
integration_c_sources=(
  firmware/gd32/common/protocol/gs_protocol.c
  firmware/gd32/common/control/gs_wheel_mix.c
  firmware/gd32/common/safety/gs_safety.c
  firmware/gd32/common/coordination/gs_master.c
  firmware/gd32/common/coordination/gs_slave.c
)

build_startup_recovery_test() {
  local variant="$1"
  shift
  local extra_flags=("$@")
  local objects=()

  for source in "${integration_c_sources[@]}"; do
    local source_name
    source_name="$(basename "${source%.c}")"
    local object="$build_dir/${source_name}-${variant}.o"
    "$c_compiler" \
      -std=c11 -Wall -Wextra -Wpedantic -Werror \
      "${integration_includes[@]}" "${extra_flags[@]}" \
      -c "$source" -o "$object"
    objects+=("$object")
  done

  local executable="$build_dir/esp32_drive_startup_recovery_${variant}"
  "$compiler" \
    -std=c++17 -Wall -Wextra -Wpedantic -Werror \
    "${integration_includes[@]}" "${extra_flags[@]}" \
    tests/esp32/test_drive_startup_recovery.cpp \
    firmware/esp32/control/drive_safety.cpp \
    "${objects[@]}" \
    -o "$executable"
  "$executable"
}

build_startup_recovery_test normal
build_startup_recovery_test sanitized \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  -fno-sanitize-recover=all

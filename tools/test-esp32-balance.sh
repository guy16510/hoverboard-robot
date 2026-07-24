#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-only
set -euo pipefail

repo_dir="$(cd "$(dirname "$0")/.." && pwd)"
build_dir="${TMPDIR:-/tmp}/gausstop-esp32-balance-tests"
compiler="${CXX:-clang++}"
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

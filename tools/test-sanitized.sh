#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-only
set -euo pipefail

repo_dir="$(cd "$(dirname "$0")/.." && pwd)"
build_dir="${TMPDIR:-/tmp}/gausstop-sanitized-tests"
compiler="${CC:-clang}"
mkdir -p "$build_dir"
cd "$repo_dir"

"$compiler" -std=c11 -Wall -Wextra -Wpedantic -Werror \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  -Ifirmware/gd32/common/protocol -Ifirmware/gd32/common/control \
  -Ifirmware/gd32/common/motor -Ifirmware/gd32/common/safety \
  -Ifirmware/gd32/common/transport -Ifirmware/gd32/common/coordination \
  -Itests/native \
  tests/native/test_main.c tests/native/test_protocol.c tests/native/test_control.c \
  tests/native/test_architecture.c \
  firmware/gd32/common/protocol/gs_protocol.c \
  firmware/gd32/common/protocol/gs_frame_parser.c \
  firmware/gd32/common/control/gs_wheel_mix.c \
  firmware/gd32/common/control/gs_console.c \
  firmware/gd32/common/motor/gs_commutation.c \
  firmware/gd32/common/motor/gs_motor_control.c \
  firmware/gd32/common/safety/gs_safety.c \
  firmware/gd32/common/coordination/gs_master.c \
  firmware/gd32/common/coordination/gs_slave.c \
  -o "$build_dir/native_tests_sanitized"
detect_leaks=1
[[ "$(uname -s)" == "Darwin" ]] && detect_leaks=0
ASAN_OPTIONS="detect_leaks=$detect_leaks" UBSAN_OPTIONS=halt_on_error=1 \
  "$build_dir/native_tests_sanitized"

#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-only
set -euo pipefail

repo_dir="$(cd "$(dirname "$0")/.." && pwd)"
cd "$repo_dir"

rg -q 'GS_PB12_BREAK_ENABLED == 0' firmware/gd32/boards/gausstop_dphc_v33/gausstop_board.h
rg -q 'breakstate = TIMER_BREAK_DISABLE' firmware/gd32/boards/gausstop_dphc_v33/gausstop_board.c
rg -q 'kControllerTx = 17' firmware/esp32/coordinator/main.cpp
rg -q 'kControllerRx = 35' firmware/esp32/coordinator/main.cpp
rg -q 'controller_rx.begin\(19200, SERIAL_8N1, kControllerRx, -1\)' firmware/esp32/passive_probe/main.cpp
if rg -q 'controller_rx\.(write|print|printf)' firmware/esp32/passive_probe/main.cpp; then
  echo "passive probe contains controller transmit behavior" >&2
  exit 1
fi
if rg -q 'gs_master_accept_esp_frame' firmware/gd32/slave; then
  echo "slave contains ESP32 command API" >&2
  exit 1
fi
if rg -q 'gs_slave_accept_master_frame' firmware/esp32; then
  echo "ESP32 directly addresses slave API" >&2
  exit 1
fi

nm_tool="$repo_dir/.toolchains/arm-none-eabi/bin/arm-none-eabi-nm"
if [[ -x "$nm_tool" ]]; then
  recovery_elf="$repo_dir/.pio/build/gausstop_safe_recovery/firmware.elf"
  diagnostic_elf="$repo_dir/.pio/build/gausstop_communication_diagnostic/firmware.elf"
  slave_elf="$repo_dir/.pio/build/gausstop_slave/firmware.elf"
  if [[ -f "$recovery_elf" ]] && "$nm_tool" "$recovery_elf" | rg -q 'gs_board_bridge_apply|timer_primary_output_config|adc_enable|usart_enable'; then
    echo "safe recovery links operational output or communications" >&2
    exit 1
  fi
  if [[ -f "$diagnostic_elf" ]] && "$nm_tool" "$diagnostic_elf" | rg -q 'gs_board_bridge_apply|gs_motor_step|timer_primary_output_config'; then
    echo "communication diagnostic links bridge activation" >&2
    exit 1
  fi
  if [[ -f "$slave_elf" ]] && "$nm_tool" "$slave_elf" | rg -q 'gs_master_accept_esp_frame'; then
    echo "slave ELF links ESP32 command acceptance" >&2
    exit 1
  fi
fi


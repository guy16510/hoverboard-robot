#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-only
set -euo pipefail

repo_dir="$(cd "$(dirname "$0")/.." && pwd)"
cd "$repo_dir"

require_match() {
  local pattern="$1"
  local path="$2"
  local description="$3"
  if ! grep -Eq "$pattern" "$path"; then
    echo "missing architecture invariant: $description" >&2
    exit 1
  fi
}

require_match 'GS_PB12_BREAK_ENABLED == 0' \
  firmware/gd32/boards/gausstop_dphc_v33/gausstop_board.h \
  'PB12 break remains disabled until validated'
require_match 'breakstate = TIMER_BREAK_DISABLE' \
  firmware/gd32/boards/gausstop_dphc_v33/gausstop_board.c \
  'timer hardware break remains disabled'
require_match 'kControllerTx = 17' firmware/esp32/coordinator/main.cpp \
  'ESP32 controller TX remains GPIO17'
require_match 'kControllerRx = 35' firmware/esp32/coordinator/main.cpp \
  'ESP32 controller RX remains GPIO35'
require_match 'controller_rx\.begin\(19200, SERIAL_8N1, kControllerRx, -1\)' \
  firmware/esp32/passive_probe/main.cpp \
  'passive probe remains RX-only at 19200 baud'
require_match 'master\.slave_feedback\.accepted_sequence == feedback_sequence' \
  firmware/gd32/master/main.c \
  'MASTER feedback is accelerated by an exact SLAVE acknowledgement'
require_match 'GS_REMOTE_FEEDBACK_FALLBACK_MS' firmware/gd32/master/main.c \
  'MASTER feedback retains a bounded fallback response'
require_match 'master\.state == GS_CONTROLLER_READY' firmware/gd32/master/main.c \
  'transport overflow faults before READY can arm motion'
require_match 'master\.transport_status_flags = 0u' firmware/gd32/master/main.c \
  'verified clearfault resets the transport latch'
require_match 'nvic_irq_enable\(TIMER13_IRQn, 1u, 0u\)' \
  firmware/gd32/boards/gausstop_dphc_v33/gausstop_swd_transport.c \
  'software feedback TX preserves exact bit timing above the link IRQ priority'

if grep -Eq 'GS_REMOTE_FEEDBACK_DELAY_MS' firmware/gd32/master/main.c; then
  echo "MASTER feedback reverted to a fixed-delay-only response" >&2
  exit 1
fi
if grep -Eq 'controller_rx\.(write|print|printf)' \
  firmware/esp32/passive_probe/main.cpp; then
  echo "passive probe contains controller transmit behavior" >&2
  exit 1
fi
if grep -ERq 'gs_master_accept_esp_frame' firmware/gd32/slave; then
  echo "slave contains ESP32 command API" >&2
  exit 1
fi
if grep -ERq 'gs_slave_accept_master_frame' firmware/esp32; then
  echo "ESP32 directly addresses slave API" >&2
  exit 1
fi

require_calibration_before_uart() {
  local source="$1"
  local calibration_line
  local uart_line
  calibration_line="$(grep -En '^[[:space:]]*calibrate_protection\(\);' "$source" |
    head -n 1 |
    cut -d: -f1 || true)"
  uart_line="$(grep -En '^[[:space:]]*gs_board_uart_init' "$source" |
    head -n 1 |
    cut -d: -f1 || true)"
  if [[ -z "$calibration_line" || -z "$uart_line" ||
        "$calibration_line" -ge "$uart_line" ]]; then
    echo "$source enables UART before blocking calibration completes" >&2
    exit 1
  fi
}

require_calibration_before_uart firmware/gd32/master/main.c
require_calibration_before_uart firmware/gd32/slave/main.c

nm_tool="$repo_dir/.toolchains/arm-none-eabi/bin/arm-none-eabi-nm"
if [[ -x "$nm_tool" ]]; then
  recovery_elf="$repo_dir/.pio/build/gausstop_safe_recovery/firmware.elf"
  diagnostic_elf="$repo_dir/.pio/build/gausstop_communication_diagnostic/firmware.elf"
  slave_elf="$repo_dir/.pio/build/gausstop_slave/firmware.elf"
  if [[ -f "$recovery_elf" ]] &&
     "$nm_tool" "$recovery_elf" | grep -Eq 'gs_board_bridge_apply|timer_primary_output_config|adc_enable|usart_enable'; then
    echo "safe recovery links operational output or communications" >&2
    exit 1
  fi
  if [[ -f "$diagnostic_elf" ]] &&
     "$nm_tool" "$diagnostic_elf" | grep -Eq 'gs_board_bridge_apply|gs_motor_step|timer_primary_output_config'; then
    echo "communication diagnostic links bridge activation" >&2
    exit 1
  fi
  if [[ -f "$slave_elf" ]] &&
     "$nm_tool" "$slave_elf" | grep -Eq 'gs_master_accept_esp_frame'; then
    echo "slave ELF links ESP32 command acceptance" >&2
    exit 1
  fi
fi

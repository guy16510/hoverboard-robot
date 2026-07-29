/* SPDX-License-Identifier: GPL-3.0-only */
#ifndef GAUSSTOP_SWD_TIMING_H
#define GAUSSTOP_SWD_TIMING_H

#include <stdint.h>

enum {
  GS_SWD_UART_BAUD = 19200u,
  GS_SWD_UART_TIMER_DIVIDER = 8u,
};

#define GS_SWD_UART_TIMER_HZ_FOR(system_clock_hz)                              \
  ((uint32_t)(system_clock_hz) / GS_SWD_UART_TIMER_DIVIDER)
#define GS_SWD_UART_BIT_TICKS_FOR(system_clock_hz)                             \
  ((GS_SWD_UART_TIMER_HZ_FOR(system_clock_hz) + (GS_SWD_UART_BAUD / 2u)) /     \
   GS_SWD_UART_BAUD)
#define GS_SWD_UART_FIRST_SAMPLE_TICKS_FOR(system_clock_hz)                    \
  ((GS_SWD_UART_BIT_TICKS_FOR(system_clock_hz) * 3u) / 2u)
#define GS_SWD_UART_SAMPLE_TICK_FOR(system_clock_hz, data_bit_index)           \
  (GS_SWD_UART_FIRST_SAMPLE_TICKS_FOR(system_clock_hz) +                       \
   ((uint32_t)(data_bit_index) * GS_SWD_UART_BIT_TICKS_FOR(system_clock_hz)))
#define GS_SWD_UART_STOP_SAMPLE_TICK_FOR(system_clock_hz)                      \
  GS_SWD_UART_SAMPLE_TICK_FOR(system_clock_hz, 8u)

#endif

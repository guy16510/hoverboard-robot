/* SPDX-License-Identifier: GPL-3.0-only */
#include <stdint.h>

#include "gausstop_swd_timing.h"
#include "test_harness.h"

void gs_test_swd_timing(void) {
  const uint32_t system_clock_hz = 8000000u;
  const uint32_t timer_hz = GS_SWD_UART_TIMER_HZ_FOR(system_clock_hz);
  const uint32_t bit_ticks = GS_SWD_UART_BIT_TICKS_FOR(system_clock_hz);
  const uint32_t first_sample =
      GS_SWD_UART_FIRST_SAMPLE_TICKS_FOR(system_clock_hz);

  GS_EXPECT_EQ(1000000u, timer_hz);
  GS_EXPECT_EQ(52u, bit_ticks);
  GS_EXPECT_EQ(78u, first_sample);
  GS_EXPECT_EQ(494u, GS_SWD_UART_STOP_SAMPLE_TICK_FOR(system_clock_hz));

  for (uint32_t bit = 0u; bit < 8u; ++bit) {
    const uint32_t sample =
        GS_SWD_UART_SAMPLE_TICK_FOR(system_clock_hz, bit);
    const uint32_t window_start = (bit + 1u) * bit_ticks;
    const uint32_t window_end = (bit + 2u) * bit_ticks;
    GS_EXPECT_TRUE(sample > window_start);
    GS_EXPECT_TRUE(sample < window_end);
    GS_EXPECT_EQ(window_start + (bit_ticks / 2u), sample);
  }

  const uint32_t actual_baud = timer_hz / bit_ticks;
  const uint32_t baud_error = actual_baud > GS_SWD_UART_BAUD
                                  ? actual_baud - GS_SWD_UART_BAUD
                                  : GS_SWD_UART_BAUD - actual_baud;
  GS_EXPECT_TRUE(baud_error * 100u < GS_SWD_UART_BAUD);
}

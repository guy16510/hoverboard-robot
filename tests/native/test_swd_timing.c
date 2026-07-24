/* SPDX-License-Identifier: GPL-3.0-only */
#include <stdint.h>
#include <string.h>

#include "gausstop_swd_pulse.h"
#include "gausstop_swd_timing.h"
#include "test_harness.h"

static uint16_t symbol_width_for_unit(uint8_t symbol, uint8_t unit_us) {
  return (uint16_t)((symbol + 1u) * unit_us);
}

static void feed_pulse_frame(const uint8_t expected[GS_SWD_PULSE_FRAME_BYTES],
                             uint8_t unit_us, int32_t jitter_us,
                             uint8_t actual[GS_SWD_PULSE_FRAME_BYTES]) {
  gs_swd_pulse_decoder decoder;
  gs_swd_pulse_decoder_init(&decoder);
  const uint16_t sync_width =
      (uint16_t)(unit_us * GS_SWD_PULSE_SYNC_UNITS);
  GS_EXPECT_EQ(GS_SWD_PULSE_SYNC,
               gs_swd_pulse_decoder_feed(&decoder, sync_width, actual));
  GS_EXPECT_EQ(unit_us, decoder.unit_us);
  for (uint8_t byte = 0u; byte < GS_SWD_PULSE_FRAME_BYTES; ++byte) {
    for (uint8_t symbol_index = 0u;
         symbol_index < GS_SWD_PULSE_SYMBOLS_PER_BYTE; ++symbol_index) {
      const uint8_t symbol =
          (uint8_t)((expected[byte] >>
                     (symbol_index * GS_SWD_PULSE_SYMBOL_BITS)) &
                    0x03u);
      const uint16_t nominal = symbol_width_for_unit(symbol, unit_us);
      const uint16_t width = (uint16_t)((int32_t)nominal + jitter_us);
      const bool last = byte == GS_SWD_PULSE_FRAME_BYTES - 1u &&
                        symbol_index == GS_SWD_PULSE_SYMBOLS_PER_BYTE - 1u;
      GS_EXPECT_EQ(last ? GS_SWD_PULSE_FRAME : GS_SWD_PULSE_NONE,
                   gs_swd_pulse_decoder_feed(&decoder, width, actual));
    }
  }
}

static void test_all_bytes_at_unit(uint8_t unit_us, bool exhaustive_jitter) {
  uint8_t expected[GS_SWD_PULSE_FRAME_BYTES];
  uint8_t actual[GS_SWD_PULSE_FRAME_BYTES];
  const int32_t tolerance =
      unit_us / GS_SWD_PULSE_SYMBOL_TOLERANCE_DIVISOR;
  for (uint16_t value = 0u; value <= UINT8_MAX; ++value) {
    memset(expected, (uint8_t)value, sizeof(expected));
    if (exhaustive_jitter) {
      for (int32_t jitter = -tolerance; jitter <= tolerance; ++jitter) {
        memset(actual, 0, sizeof(actual));
        feed_pulse_frame(expected, unit_us, jitter, actual);
        GS_EXPECT_BYTES(expected, actual, sizeof(expected));
      }
    } else {
      const int32_t jitters[] = {-tolerance, 0, tolerance};
      for (uint8_t index = 0u; index < 3u; ++index) {
        memset(actual, 0, sizeof(actual));
        feed_pulse_frame(expected, unit_us, jitters[index], actual);
        GS_EXPECT_BYTES(expected, actual, sizeof(expected));
      }
    }
  }
}

static void test_pulse_decoder(void) {
  test_all_bytes_at_unit(GS_SWD_PULSE_NOMINAL_UNIT_US, true);
  test_all_bytes_at_unit(GS_SWD_PULSE_MIN_UNIT_US, false);
  test_all_bytes_at_unit(GS_SWD_PULSE_MAX_UNIT_US, false);

  uint8_t actual[GS_SWD_PULSE_FRAME_BYTES] = {0};
  gs_swd_pulse_decoder decoder;
  gs_swd_pulse_decoder_init(&decoder);
  GS_EXPECT_EQ(
      GS_SWD_PULSE_SYNC,
      gs_swd_pulse_decoder_feed(
          &decoder,
          GS_SWD_PULSE_MIN_UNIT_US * GS_SWD_PULSE_SYNC_UNITS, actual));
  GS_EXPECT_EQ(GS_SWD_PULSE_MIN_UNIT_US, decoder.unit_us);
  GS_EXPECT_EQ(
      GS_SWD_PULSE_SYNC,
      gs_swd_pulse_decoder_feed(
          &decoder,
          GS_SWD_PULSE_MAX_UNIT_US * GS_SWD_PULSE_SYNC_UNITS, actual));
  GS_EXPECT_EQ(GS_SWD_PULSE_MAX_UNIT_US, decoder.unit_us);

  GS_EXPECT_EQ(GS_SWD_PULSE_ERROR,
               gs_swd_pulse_decoder_feed(&decoder, 440u, actual));
  GS_EXPECT_FALSE(decoder.active);
  GS_EXPECT_EQ(0u, decoder.unit_us);
  GS_EXPECT_EQ(0u, gs_swd_pulse_symbol_width_us(4u));

  gs_swd_pulse_decoder_init(&decoder);
  GS_EXPECT_EQ(GS_SWD_PULSE_SYNC,
               gs_swd_pulse_decoder_feed(&decoder, GS_SWD_PULSE_SYNC_US,
                                         actual));
  GS_EXPECT_EQ(GS_SWD_PULSE_NONE,
               gs_swd_pulse_decoder_feed(
                   &decoder, gs_swd_pulse_symbol_width_us(3u), actual));
  GS_EXPECT_EQ(GS_SWD_PULSE_SYNC,
               gs_swd_pulse_decoder_feed(&decoder, GS_SWD_PULSE_SYNC_US,
                                         actual));
  GS_EXPECT_EQ(0u, decoder.symbol_index);
  GS_EXPECT_TRUE(decoder.active);
}

void gs_test_swd_timing(void) {
  const uint32_t system_clock_hz = 8000000u;
  const uint32_t timer_hz = GS_SWD_UART_TIMER_HZ_FOR(system_clock_hz);
  const uint32_t bit_ticks = GS_SWD_UART_BIT_TICKS_FOR(system_clock_hz);

  GS_EXPECT_EQ(1000000u, timer_hz);
  GS_EXPECT_EQ(52u, bit_ticks);
  const uint32_t actual_baud = timer_hz / bit_ticks;
  const uint32_t baud_error = actual_baud > GS_SWD_UART_BAUD
                                  ? actual_baud - GS_SWD_UART_BAUD
                                  : GS_SWD_UART_BAUD - actual_baud;
  GS_EXPECT_TRUE(baud_error * 100u < GS_SWD_UART_BAUD);

  GS_EXPECT_EQ(64u, gs_swd_pulse_symbol_width_us(0u));
  GS_EXPECT_EQ(128u, gs_swd_pulse_symbol_width_us(1u));
  GS_EXPECT_EQ(192u, gs_swd_pulse_symbol_width_us(2u));
  GS_EXPECT_EQ(256u, gs_swd_pulse_symbol_width_us(3u));
  GS_EXPECT_TRUE(GS_SWD_PULSE_MAX_UNIT_US * 4u <
                 GS_SWD_PULSE_MIN_UNIT_US * GS_SWD_PULSE_SYNC_UNITS);
  test_pulse_decoder();
}

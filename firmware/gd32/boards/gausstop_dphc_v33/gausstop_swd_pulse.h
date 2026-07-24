/* SPDX-License-Identifier: GPL-3.0-only */
#ifndef GAUSSTOP_SWD_PULSE_H
#define GAUSSTOP_SWD_PULSE_H

#include <stdbool.h>
#include <stdint.h>

#define GS_SWD_PULSE_FRAME_BYTES 11u

enum {
  GS_SWD_PULSE_SYMBOL_BITS = 2u,
  GS_SWD_PULSE_SYMBOLS_PER_BYTE = 4u,
  GS_SWD_PULSE_FRAME_SYMBOLS =
      GS_SWD_PULSE_FRAME_BYTES * GS_SWD_PULSE_SYMBOLS_PER_BYTE,
  GS_SWD_PULSE_SYMBOL_BASE_US = 32u,
  GS_SWD_PULSE_SYMBOL_TOLERANCE_US = 12u,
  GS_SWD_PULSE_SYNC_US = 224u,
  GS_SWD_PULSE_SYNC_TOLERANCE_US = 32u,
  GS_SWD_PULSE_SEPARATOR_US = 32u,
  GS_SWD_PULSE_SYNC_SEPARATOR_US = 64u,
};

typedef enum {
  GS_SWD_PULSE_NONE = 0,
  GS_SWD_PULSE_SYNC,
  GS_SWD_PULSE_FRAME,
  GS_SWD_PULSE_ERROR,
} gs_swd_pulse_result;

typedef struct {
  uint8_t frame[GS_SWD_PULSE_FRAME_BYTES];
  uint8_t symbol_index;
  bool active;
} gs_swd_pulse_decoder;

void gs_swd_pulse_decoder_init(gs_swd_pulse_decoder *decoder);
uint16_t gs_swd_pulse_symbol_width_us(uint8_t symbol);
gs_swd_pulse_result
gs_swd_pulse_decoder_feed(gs_swd_pulse_decoder *decoder, uint16_t low_width_us,
                          uint8_t out_frame[GS_SWD_PULSE_FRAME_BYTES]);

#endif

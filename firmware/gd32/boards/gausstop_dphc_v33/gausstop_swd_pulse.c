/* SPDX-License-Identifier: GPL-3.0-only */
#include "gausstop_swd_pulse.h"

#include <stddef.h>
#include <string.h>

static bool within(uint16_t value, uint16_t target, uint16_t tolerance) {
  const uint16_t minimum =
      target > tolerance ? (uint16_t)(target - tolerance) : (uint16_t)0u;
  const uint16_t maximum = (uint16_t)(target + tolerance);
  return value >= minimum && value <= maximum;
}

void gs_swd_pulse_decoder_init(gs_swd_pulse_decoder *decoder) {
  if (decoder == NULL) {
    return;
  }
  memset(decoder, 0, sizeof(*decoder));
}

uint16_t gs_swd_pulse_symbol_width_us(uint8_t symbol) {
  if (symbol > 3u) {
    return 0u;
  }
  return (uint16_t)((symbol + 1u) * GS_SWD_PULSE_SYMBOL_BASE_US);
}

static bool decode_symbol(uint16_t low_width_us, uint8_t *symbol) {
  if (symbol == NULL) {
    return false;
  }
  for (uint8_t candidate = 0u; candidate < 4u; ++candidate) {
    if (within(low_width_us, gs_swd_pulse_symbol_width_us(candidate),
               GS_SWD_PULSE_SYMBOL_TOLERANCE_US)) {
      *symbol = candidate;
      return true;
    }
  }
  return false;
}

gs_swd_pulse_result
gs_swd_pulse_decoder_feed(gs_swd_pulse_decoder *decoder, uint16_t low_width_us,
                          uint8_t out_frame[GS_SWD_PULSE_FRAME_BYTES]) {
  if (decoder == NULL) {
    return GS_SWD_PULSE_ERROR;
  }
  if (within(low_width_us, GS_SWD_PULSE_SYNC_US,
             GS_SWD_PULSE_SYNC_TOLERANCE_US)) {
    memset(decoder->frame, 0, sizeof(decoder->frame));
    decoder->symbol_index = 0u;
    decoder->active = true;
    return GS_SWD_PULSE_SYNC;
  }
  if (!decoder->active) {
    return GS_SWD_PULSE_NONE;
  }
  uint8_t symbol = 0u;
  if (!decode_symbol(low_width_us, &symbol)) {
    decoder->active = false;
    decoder->symbol_index = 0u;
    return GS_SWD_PULSE_ERROR;
  }
  const uint8_t byte_index =
      (uint8_t)(decoder->symbol_index / GS_SWD_PULSE_SYMBOLS_PER_BYTE);
  const uint8_t shift =
      (uint8_t)((decoder->symbol_index % GS_SWD_PULSE_SYMBOLS_PER_BYTE) *
                GS_SWD_PULSE_SYMBOL_BITS);
  decoder->frame[byte_index] |= (uint8_t)(symbol << shift);
  ++decoder->symbol_index;
  if (decoder->symbol_index < GS_SWD_PULSE_FRAME_SYMBOLS) {
    return GS_SWD_PULSE_NONE;
  }
  decoder->active = false;
  decoder->symbol_index = 0u;
  if (out_frame != NULL) {
    memcpy(out_frame, decoder->frame, sizeof(decoder->frame));
  }
  return GS_SWD_PULSE_FRAME;
}

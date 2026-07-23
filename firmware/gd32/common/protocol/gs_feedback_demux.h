/* SPDX-License-Identifier: GPL-3.0-only */
#ifndef GS_FEEDBACK_DEMUX_H
#define GS_FEEDBACK_DEMUX_H

#include <stddef.h>
#include <stdint.h>

#include "gs_protocol.h"

typedef enum {
  GS_DEMUX_NONE = 0,
  GS_DEMUX_MASTER_FEEDBACK,
  GS_DEMUX_MASTER_DIAGNOSTIC,
  GS_DEMUX_BAD_CRC,
  GS_DEMUX_TIMEOUT,
} gs_demux_result;

typedef struct {
  uint8_t bytes[GS_MAX_FRAME_SIZE];
  size_t length;
  size_t target_size;
  uint32_t last_byte_ms;
  gs_demux_result target_result;
} gs_feedback_demux;

void gs_feedback_demux_init(gs_feedback_demux *demux);
gs_demux_result gs_feedback_demux_feed(
    gs_feedback_demux *demux, uint8_t byte, uint32_t now_ms,
    uint8_t out[GS_MAX_FRAME_SIZE]);
gs_demux_result gs_feedback_demux_poll(gs_feedback_demux *demux,
                                       uint32_t now_ms);

#endif

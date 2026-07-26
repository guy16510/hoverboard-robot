/* SPDX-License-Identifier: GPL-3.0-only */
#ifndef GS_FRAME_PARSER_H
#define GS_FRAME_PARSER_H

#include <stddef.h>
#include <stdint.h>

#include "gs_protocol.h"

typedef enum {
  GS_PARSE_NONE = 0,
  GS_PARSE_FRAME,
  GS_PARSE_BAD_CRC,
  GS_PARSE_TIMEOUT,
} gs_parse_result;

typedef struct {
  uint8_t bytes[GS_MAX_FRAME_SIZE];
  uint8_t marker[2];
  size_t length;
  size_t frame_size;
  size_t marker_size;
  uint32_t last_byte_ms;
} gs_frame_parser;

void gs_frame_parser_init_full(gs_frame_parser *parser, const uint8_t *marker,
                               size_t marker_size, size_t frame_size);
void gs_frame_parser_init_master_feedback(gs_frame_parser *parser,
                                          uint32_t legacy_timeout_ms);

#define GS_FRAME_PARSER_INIT_SELECT(_1, _2, _3, _4, NAME, ...) NAME
#define gs_frame_parser_init(...)                                             \
  GS_FRAME_PARSER_INIT_SELECT(__VA_ARGS__, gs_frame_parser_init_full,          \
                              gs_frame_parser_init_invalid_arity,              \
                              gs_frame_parser_init_master_feedback)            \
  (__VA_ARGS__)

gs_parse_result gs_frame_parser_feed(gs_frame_parser *parser, uint8_t byte,
                                      uint32_t now_ms,
                                      uint8_t out[GS_MAX_FRAME_SIZE]);
gs_parse_result gs_frame_parser_poll(gs_frame_parser *parser, uint32_t now_ms);

#endif

/* SPDX-License-Identifier: GPL-3.0-only */
#include "gs_frame_parser.h"

#include <stdbool.h>
#include <string.h>

static bool marker_at(const gs_frame_parser *parser, size_t offset) {
  return offset + parser->marker_size <= parser->length &&
         memcmp(&parser->bytes[offset], parser->marker, parser->marker_size) ==
             0;
}

static void preserve_nested_marker(gs_frame_parser *parser) {
  for (size_t offset = 1; offset < parser->length; ++offset) {
    if (marker_at(parser, offset)) {
      const size_t remaining = parser->length - offset;
      memmove(parser->bytes, &parser->bytes[offset], remaining);
      parser->length = remaining;
      return;
    }
  }
  parser->length = 0;
}

void gs_frame_parser_init(gs_frame_parser *parser, const uint8_t *marker,
                          size_t marker_size, size_t frame_size) {
  if (parser == NULL) {
    return;
  }
  memset(parser, 0, sizeof(*parser));
  if (marker == NULL || marker_size == 0u ||
      marker_size > sizeof(parser->marker) || frame_size < marker_size + 2u ||
      frame_size > GS_MAX_FRAME_SIZE) {
    return;
  }
  memcpy(parser->marker, marker, marker_size);
  parser->marker_size = marker_size;
  parser->frame_size = frame_size;
}

static void accept_marker_byte(gs_frame_parser *parser, uint8_t byte,
                               uint32_t now_ms) {
  if (parser->length == 0u) {
    if (byte == parser->marker[0]) {
      parser->bytes[0] = byte;
      parser->length = 1u;
      parser->last_byte_ms = now_ms;
    }
    return;
  }
  if (parser->length < parser->marker_size) {
    if (byte == parser->marker[parser->length]) {
      parser->bytes[parser->length++] = byte;
      parser->last_byte_ms = now_ms;
    } else {
      parser->length = 0u;
      accept_marker_byte(parser, byte, now_ms);
    }
  }
}

gs_parse_result gs_frame_parser_feed(gs_frame_parser *parser, uint8_t byte,
                                     uint32_t now_ms,
                                     uint8_t out[GS_MAX_FRAME_SIZE]) {
  if (parser == NULL || parser->frame_size == 0u) {
    return GS_PARSE_NONE;
  }
  if (parser->length != 0u &&
      (uint32_t)(now_ms - parser->last_byte_ms) > GS_PARTIAL_FRAME_TIMEOUT_MS) {
    parser->length = 0u;
  }
  if (parser->length < parser->marker_size) {
    accept_marker_byte(parser, byte, now_ms);
    return GS_PARSE_NONE;
  }
  if (parser->length < parser->frame_size) {
    parser->bytes[parser->length++] = byte;
    parser->last_byte_ms = now_ms;
  }
  if (parser->length != parser->frame_size) {
    return GS_PARSE_NONE;
  }

  const uint16_t expected =
      (uint16_t)parser->bytes[parser->frame_size - 2u] |
      ((uint16_t)parser->bytes[parser->frame_size - 1u] << 8);
  const uint16_t actual = gs_crc16(parser->bytes, parser->frame_size - 2u);
  if (actual == expected) {
    if (out != NULL) {
      memcpy(out, parser->bytes, parser->frame_size);
    }
    parser->length = 0u;
    return GS_PARSE_FRAME;
  }
  preserve_nested_marker(parser);
  return GS_PARSE_BAD_CRC;
}

gs_parse_result gs_frame_parser_poll(gs_frame_parser *parser, uint32_t now_ms) {
  if (parser == NULL || parser->length == 0u) {
    return GS_PARSE_NONE;
  }
  if ((uint32_t)(now_ms - parser->last_byte_ms) <=
      GS_PARTIAL_FRAME_TIMEOUT_MS) {
    return GS_PARSE_NONE;
  }
  parser->length = 0u;
  return GS_PARSE_TIMEOUT;
}

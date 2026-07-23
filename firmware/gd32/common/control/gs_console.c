/* SPDX-License-Identifier: GPL-3.0-only */
#include "gs_console.h"

#include <errno.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>

#include "gs_wheel_mix.h"

typedef struct {
  char storage[GS_CONSOLE_MAX_LINE + 1];
  char *tokens[4];
  size_t count;
} token_line;

static bool tokenize(token_line *parsed, const char *line, size_t length) {
  if (parsed == NULL || line == NULL || length == 0u ||
      length > GS_CONSOLE_MAX_LINE) {
    return false;
  }
  memcpy(parsed->storage, line, length);
  parsed->storage[length] = '\0';
  parsed->count = 0u;
  char *cursor = parsed->storage;
  while (*cursor != '\0') {
    while (*cursor == ' ' || *cursor == '\t') {
      ++cursor;
    }
    if (*cursor == '\0') {
      break;
    }
    if (parsed->count == 4u) {
      return false;
    }
    parsed->tokens[parsed->count++] = cursor;
    while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t') {
      if (*cursor == '\r' || *cursor == '\n') {
        return false;
      }
      ++cursor;
    }
    if (*cursor != '\0') {
      *cursor++ = '\0';
    }
  }
  return parsed->count != 0u;
}

static bool parse_integer(const char *token, int32_t minimum, int32_t maximum,
                          int16_t *out) {
  char *end = NULL;
  errno = 0;
  const long value = strtol(token, &end, 10);
  if (errno == ERANGE || end == token || *end != '\0' || value < minimum ||
      value > maximum || value < INT16_MIN || value > INT16_MAX) {
    return false;
  }
  *out = (int16_t)value;
  return true;
}

static void clear_motion(gs_console_state *state) {
  state->target.mode = GS_DRIVE_MIXED;
  state->target.first = 0;
  state->target.second = 0;
}

void gs_console_init(gs_console_state *state) {
  if (state == NULL) {
    return;
  }
  memset(state, 0, sizeof(*state));
  state->ramp_per_tick = GS_DEFAULT_RAMP_PER_TICK;
}

static bool motion_allowed(const gs_console_state *state) {
  return state->enabled && !state->shutdown;
}

gs_console_result gs_console_execute(gs_console_state *state, const char *line,
                                     size_t length) {
  token_line parsed;
  gs_console_state candidate;
  int16_t first = 0;
  int16_t second = 0;
  if (state == NULL || !tokenize(&parsed, line, length)) {
    return GS_CONSOLE_REJECTED;
  }
  candidate = *state;

  if (parsed.count == 1u && strcmp(parsed.tokens[0], "status") == 0) {
    return GS_CONSOLE_STATUS;
  }
  if (parsed.count == 1u && strcmp(parsed.tokens[0], "help") == 0) {
    return GS_CONSOLE_HELP;
  }
  if (parsed.count == 1u && strcmp(parsed.tokens[0], "enable") == 0) {
    if (candidate.shutdown) {
      return GS_CONSOLE_REJECTED;
    }
    candidate.enabled = true;
  } else if (parsed.count == 1u && strcmp(parsed.tokens[0], "disable") == 0) {
    candidate.enabled = false;
    clear_motion(&candidate);
    candidate.current = (gs_wheel_pair){0, 0};
  } else if (parsed.count == 1u && strcmp(parsed.tokens[0], "stop") == 0) {
    clear_motion(&candidate);
  } else if (parsed.count == 1u && strcmp(parsed.tokens[0], "shutdown") == 0) {
    candidate.enabled = false;
    candidate.shutdown = true;
    clear_motion(&candidate);
    candidate.current = (gs_wheel_pair){0, 0};
  } else if (parsed.count == 1u &&
             strcmp(parsed.tokens[0], "clearfault") == 0) {
    return !candidate.enabled && candidate.current.left == 0 &&
                   candidate.current.right == 0
               ? GS_CONSOLE_CLEAR_FAULT
               : GS_CONSOLE_REJECTED;
  } else if (parsed.count == 3u && strcmp(parsed.tokens[0], "drive") == 0 &&
             motion_allowed(&candidate) &&
             parse_integer(parsed.tokens[1], -1000, 1000, &first) &&
             parse_integer(parsed.tokens[2], -1000, 1000, &second)) {
    candidate.target = (gs_drive_request){GS_DRIVE_MIXED, first, second};
  } else if (parsed.count == 3u && strcmp(parsed.tokens[0], "lr") == 0 &&
             motion_allowed(&candidate) &&
             parse_integer(parsed.tokens[1], -1000, 1000, &first) &&
             parse_integer(parsed.tokens[2], -1000, 1000, &second)) {
    candidate.target = (gs_drive_request){GS_DRIVE_DIRECT_LR, first, second};
  } else if (parsed.count == 2u && strcmp(parsed.tokens[0], "forward") == 0 &&
             motion_allowed(&candidate) &&
             parse_integer(parsed.tokens[1], 0, 1000, &first)) {
    candidate.target = (gs_drive_request){GS_DRIVE_MIXED, first, 0};
  } else if (parsed.count == 2u && strcmp(parsed.tokens[0], "reverse") == 0 &&
             motion_allowed(&candidate) &&
             parse_integer(parsed.tokens[1], 0, 1000, &first)) {
    candidate.target = (gs_drive_request){GS_DRIVE_MIXED, (int16_t)-first, 0};
  } else if (parsed.count == 2u && strcmp(parsed.tokens[0], "ramp") == 0 &&
             parse_integer(parsed.tokens[1], 1, 1000, &first)) {
    candidate.ramp_per_tick = (uint16_t)first;
  } else {
    return GS_CONSOLE_REJECTED;
  }
  *state = candidate;
  return GS_CONSOLE_APPLIED;
}

static int16_t ramp_axis(int16_t current, int16_t target, uint16_t step) {
  const int32_t delta = (int32_t)target - current;
  if (delta > (int32_t)step) {
    return (int16_t)(current + step);
  }
  if (delta < -(int32_t)step) {
    return (int16_t)(current - step);
  }
  return target;
}

void gs_console_ramp_tick(gs_console_state *state) {
  if (state == NULL || !state->enabled || state->shutdown) {
    return;
  }
  const gs_wheel_pair target =
      state->target.mode == GS_DRIVE_DIRECT_LR
          ? gs_direct_wheels(state->target.first, state->target.second)
          : gs_mix_wheels(state->target.first, state->target.second);
  state->current.left =
      ramp_axis(state->current.left, target.left, state->ramp_per_tick);
  state->current.right =
      ramp_axis(state->current.right, target.right, state->ramp_per_tick);
}

void gs_console_ramp_tick_when_ready(gs_console_state *state,
                                     bool motion_ready) {
  if (state == NULL) {
    return;
  }
  if (!motion_ready) {
    state->current = (gs_wheel_pair){0, 0};
    return;
  }
  gs_console_ramp_tick(state);
}

/* SPDX-License-Identifier: GPL-3.0-only */
#ifndef GS_TYPES_H
#define GS_TYPES_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  int16_t left;
  int16_t right;
} gs_wheel_pair;

typedef enum {
  GS_DRIVE_MIXED = 0,
  GS_DRIVE_DIRECT_LR,
} gs_drive_mode;

typedef struct {
  gs_drive_mode mode;
  int16_t first;
  int16_t second;
} gs_drive_request;

typedef struct {
  int16_t logical_command;
  uint16_t compare_offset;
  bool bridge_enabled;
} gs_motor_demand;

#endif

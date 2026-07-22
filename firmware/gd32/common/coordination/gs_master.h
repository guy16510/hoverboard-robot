/* SPDX-License-Identifier: GPL-3.0-only */
#ifndef GS_MASTER_H
#define GS_MASTER_H

#include <stdbool.h>
#include <stdint.h>

#include "gs_protocol.h"
#include "gs_types.h"

typedef enum {
  GS_CONTROLLER_DISABLED = 0,
  GS_CONTROLLER_READY,
  GS_CONTROLLER_ACTIVE,
  GS_CONTROLLER_FAULTED,
  GS_CONTROLLER_SHUTDOWN,
} gs_controller_state;

typedef struct {
  gs_controller_state state;
  gs_wheel_pair demanded;
  gs_wheel_pair applied;
  int32_t local_odometer;
  gs_slave_feedback slave_feedback;
  uint32_t faults;
  uint32_t last_esp_command_ms;
  bool esp_seen;
  bool shutdown;
  uint8_t slave_flags;
} gs_master_coordinator;

void gs_master_init(gs_master_coordinator *master, uint32_t now_ms);
bool gs_master_accept_esp_frame(gs_master_coordinator *master,
                                const uint8_t frame[GS_ESP_COMMAND_SIZE],
                                uint32_t now_ms);
void gs_master_tick(gs_master_coordinator *master, uint32_t now_ms);
bool gs_master_make_slave_frame(const gs_master_coordinator *master,
                                uint8_t out[GS_SLAVE_COMMAND_SIZE]);
bool gs_master_accept_slave_feedback(
    gs_master_coordinator *master, const uint8_t frame[GS_SLAVE_FEEDBACK_SIZE]);
bool gs_master_make_feedback(const gs_master_coordinator *master,
                             uint8_t out[GS_MASTER_FEEDBACK_SIZE]);

#endif

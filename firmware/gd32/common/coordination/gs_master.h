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
  gs_wheel_pair requested;
  gs_wheel_pair demanded;
  gs_wheel_pair applied;
  int32_t local_odometer;
  gs_slave_feedback slave_feedback;
  gs_esp_command last_esp_command;
  uint32_t faults;
  uint32_t first_fault;
  uint32_t last_esp_command_ms;
  uint32_t last_new_esp_sequence_ms;
  uint32_t last_slave_feedback_ms;
  uint32_t valid_esp_frames;
  uint32_t invalid_esp_frames;
  uint32_t missing_esp_sequences;
  uint32_t valid_slave_feedback_frames;
  uint32_t invalid_slave_feedback_frames;
  uint16_t last_esp_sequence;
  uint16_t last_forwarded_sequence;
  uint16_t enable_epoch;
  uint16_t fault_epoch;
  bool esp_seen;
  bool slave_feedback_seen;
  bool shutdown;
  bool clear_fault_pending;
  bool enable_epoch_valid;
  bool recovery_required;
  uint8_t last_clear_result;
  uint8_t slave_flags;
  uint8_t runtime_status_flags;
} gs_master_coordinator;

void gs_master_init(gs_master_coordinator *master, uint32_t now_ms);
void gs_master_latch_fault(gs_master_coordinator *master, uint32_t fault);
bool gs_master_accept_esp_frame(gs_master_coordinator *master,
                                const uint8_t frame[GS_ESP_COMMAND_SIZE],
                                uint32_t now_ms);
void gs_master_tick(gs_master_coordinator *master, uint32_t now_ms);
bool gs_master_make_slave_frame(gs_master_coordinator *master,
                                uint8_t out[GS_SLAVE_COMMAND_SIZE]);
bool gs_master_accept_slave_feedback(
    gs_master_coordinator *master,
    const uint8_t frame[GS_SLAVE_FEEDBACK_SIZE], uint32_t now_ms);
bool gs_master_peer_healthy(const gs_master_coordinator *master,
                            uint32_t now_ms);
bool gs_master_fault_clear_requested(const gs_master_coordinator *master);
void gs_master_finish_fault_clear(gs_master_coordinator *master,
                                  gs_clear_result result);
void gs_master_set_runtime_status(gs_master_coordinator *master,
                                  bool pa4_raw_high, bool pa4_bypass);
bool gs_master_make_feedback(const gs_master_coordinator *master,
                             uint8_t out[GS_MASTER_FEEDBACK_SIZE],
                             uint32_t now_ms);

#endif

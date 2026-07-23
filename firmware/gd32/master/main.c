/* SPDX-License-Identifier: GPL-3.0-only */
#include <stdbool.h>
#include <stdint.h>

#include "gausstop_board.h"
#include "gd32f1x0.h"
#include "gs_frame_parser.h"
#include "gs_master.h"
#include "gs_motor_control.h"
#include "gs_safety.h"

static gs_master_coordinator master;
static gs_motor_controller motor;
static gs_safety_supervisor safety;
static gs_frame_parser esp_parser;
static gs_frame_parser slave_parser;

static void calibrate_protection(void) {
  for (uint8_t sample = 0; sample < GS_ADC_CALIBRATION_SAMPLES; ++sample) {
    const uint32_t wait_until = gs_board_millis() + GS_ADC_SAMPLE_PERIOD_MS;
    while ((int32_t)(gs_board_millis() - wait_until) < 0) {
    }
    uint16_t value = 0;
    if (gs_board_adc_read(&value)) {
      gs_safety_sample_adc_off(&safety, value);
    } else {
      gs_safety_latch(&safety, GS_FAULT_ADC_CALIBRATION);
    }
  }
}

static void stop_for_protocol_fault(void) {
  master.faults |= GS_FAULT_PROTOCOL;
  master.demanded = (gs_wheel_pair){0, 0};
  master.applied = (gs_wheel_pair){0, 0};
  master.state = GS_CONTROLLER_FAULTED;
  gs_safety_latch(&safety, GS_FAULT_PROTOCOL);
  gs_motor_force_off(&motor);
}

static void service_esp_rx(void) {
  uint8_t byte = 0;
  uint8_t frame[GS_MAX_FRAME_SIZE];
  while (gs_board_uart_read(GS_UART_REMOTE, &byte)) {
    const gs_parse_result result =
        gs_frame_parser_feed(&esp_parser, byte, gs_board_millis(), frame);
    if (result == GS_PARSE_FRAME) {
      gs_esp_command command;
      if (!gs_decode_esp_command(&command, frame) ||
          !gs_master_accept_esp_frame(&master, frame, gs_board_millis())) {
        if (master.state == GS_CONTROLLER_READY ||
            master.state == GS_CONTROLLER_ACTIVE) {
          stop_for_protocol_fault();
        }
      } else {
        gs_safety_note_command(&safety, gs_board_millis());
        if ((command.master_flags & GS_COMMAND_CLEAR_FAULT) != 0u &&
            (command.master_flags & GS_COMMAND_DISABLE) != 0u) {
          const gs_safety_sample sample = {gs_board_shutdown_clear(), true,
                                           safety.adc_baseline, true};
          (void)gs_safety_clear(&safety, &sample, gs_board_millis());
        }
      }
    } else if (result == GS_PARSE_BAD_CRC) {
      /*
       * Do not refresh the command watchdog for a corrupt frame. Isolated
       * line noise is discarded; sustained corruption reaches the 400 ms
       * command timeout and forces the bridge off.
       */
    }
  }
}

static void service_slave_rx(void) {
  uint8_t byte = 0;
  uint8_t frame[GS_MAX_FRAME_SIZE];
  while (gs_board_uart_read(GS_UART_LINK, &byte)) {
    const gs_parse_result result =
        gs_frame_parser_feed(&slave_parser, byte, gs_board_millis(), frame);
    if (result == GS_PARSE_FRAME) {
      if (!gs_master_accept_slave_feedback(&master, frame)) {
        if (master.state == GS_CONTROLLER_READY ||
            master.state == GS_CONTROLLER_ACTIVE) {
          stop_for_protocol_fault();
        }
      }
    } else if (result == GS_PARSE_BAD_CRC) {
      /*
       * The SLAVE continues to enforce its 100 ms command watchdog. Drop an
       * isolated corrupt feedback frame instead of permanently faulting both
       * otherwise healthy controllers.
       */
    }
  }
}

static void service_motor(void) {
  static uint32_t last_service_ms;
  static uint32_t last_hall_us;
  static uint8_t previous_hall;
  static uint32_t last_adc_ms;
  static uint16_t adc_value;
  static bool adc_valid;
  const uint32_t now_ms = gs_board_millis();
  if (now_ms == last_service_ms) {
    return;
  }
  last_service_ms = now_ms;
  if ((uint32_t)(now_ms - last_adc_ms) >= GS_ADC_SAMPLE_PERIOD_MS) {
    adc_valid = gs_board_adc_read(&adc_value);
    last_adc_ms = now_ms;
  }
  const uint8_t hall = gs_board_read_hall();
  const uint32_t hall_us = gs_board_micros();
  const bool hall_changed = previous_hall != 0u && hall != previous_hall;
  const uint32_t interval_us = hall_us - last_hall_us;
  if (hall_changed) {
    last_hall_us = hall_us;
  }
  previous_hall = hall;

  gs_safety_set_enabled(&safety, master.state == GS_CONTROLLER_READY ||
                                     master.state == GS_CONTROLLER_ACTIVE);
  gs_safety_note_demand(
      &safety, master.demanded.left != 0 || motor.applied_command != 0, now_ms);
  const gs_safety_sample sample = {gs_board_shutdown_clear(), adc_valid,
                                   adc_value, hall != 0u && hall != 7u};
  gs_safety_evaluate(&safety, &sample, now_ms);
  master.faults |= safety.faults.bits;
  if (safety.faults.bits != 0u) {
    master.demanded = (gs_wheel_pair){0, 0};
    master.state = GS_CONTROLLER_FAULTED;
  }
  const bool permitted =
      safety.enabled && safety.adc_ready && safety.faults.bits == 0u;
  const gs_motor_input input = {hall,      hall_changed,         interval_us,
                                permitted, master.demanded.left, now_ms};
  const gs_motor_output output = gs_motor_step(&motor, &input);
  if (output.faulted) {
    const gs_fault_flag fault =
        output.hall_result == GS_HALL_TOO_FAST
            ? GS_FAULT_HALL_TOO_FAST
            : (output.hall_result == GS_HALL_ILLEGAL ? GS_FAULT_HALL_SEQUENCE
                                                     : GS_FAULT_HALL_INVALID);
    gs_safety_latch(&safety, fault);
    master.faults |= (uint32_t)fault;
    master.state = GS_CONTROLLER_FAULTED;
  }
  if (output.hall_result == GS_HALL_LEGAL) {
    gs_safety_note_hall(&safety, now_ms);
  }
  master.applied.left = output.demand.logical_command;
  master.applied.right = master.demanded.right;
  master.local_odometer = motor.odometer;
}

int main(void) {
  const bool watchdog_reset = gs_board_watchdog_was_reset();
  rcu_all_reset_flag_clear();
  gs_board_operational_init();
  gs_board_uart_init(GS_UART_REMOTE, true);
  gs_board_uart_init(GS_UART_LINK, true);
  gs_master_init(&master, gs_board_millis());
  gs_motor_init(&motor, gs_board_bridge_port(), gs_board_millis());
  gs_safety_init(&safety, GS_SAFETY_MASTER, watchdog_reset, gs_board_millis());
  calibrate_protection();
  const uint8_t command_marker[] = {GS_COMMAND_MARKER};
  gs_frame_parser_init(&esp_parser, command_marker, 1, GS_ESP_COMMAND_SIZE);
  gs_frame_parser_init(&slave_parser, command_marker, 1,
                       GS_SLAVE_FEEDBACK_SIZE);
  gs_board_watchdog_start();

  uint32_t next_link_ms = 0;
  uint32_t next_feedback_ms = 0;
  for (;;) {
    service_esp_rx();
    service_slave_rx();
    gs_master_tick(&master, gs_board_millis());
    service_motor();
    const uint32_t now = gs_board_millis();
    if ((int32_t)(now - next_link_ms) >= 0) {
      uint8_t frame[GS_SLAVE_COMMAND_SIZE];
      next_link_ms = now + 20u;
      if (gs_master_make_slave_frame(&master, frame)) {
        (void)gs_board_uart_write(GS_UART_LINK, frame, sizeof(frame));
      }
    }
    if ((int32_t)(now - next_feedback_ms) >= 0) {
      uint8_t frame[GS_MASTER_FEEDBACK_SIZE];
      next_feedback_ms = now + 50u;
      if (gs_master_make_feedback(&master, frame)) {
        (void)gs_board_uart_write(GS_UART_REMOTE, frame, sizeof(frame));
      }
    }
    gs_board_watchdog_reload();
  }
}

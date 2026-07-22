/* SPDX-License-Identifier: GPL-3.0-only */
#include <stdint.h>

#include "gausstop_board.h"
#include "gd32f1x0.h"

typedef struct {
  uint32_t magic;
  uint32_t format;
  uint32_t target;
  uint32_t board;
  uint32_t clock_hz;
  volatile uint32_t sequence;
} gs_recovery_heartbeat;

__attribute__((section(".heartbeat"),
               used)) static volatile gs_recovery_heartbeat heartbeat;

int main(void) {
  rcu_periph_clock_enable(RCU_GPIOB);
  gpio_bit_reset(GPIOB, GPIO_PIN_2);
  gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_2);
  gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_2);
  gpio_bit_set(GPIOB, GPIO_PIN_2);
  heartbeat.magic = 0x47535243u; /* GSRC */
  heartbeat.format = 1u;
  heartbeat.target = 0x0130C8C6u;
  heartbeat.board = 0x44504843u; /* DPHC */
  heartbeat.clock_hz = GS_SYSTEM_CLOCK_HZ;
  heartbeat.sequence = 0u;
  for (;;) {
    for (volatile uint32_t delay = 0; delay < 250000u; ++delay) {
      __asm volatile("nop");
    }
    ++heartbeat.sequence;
  }
}

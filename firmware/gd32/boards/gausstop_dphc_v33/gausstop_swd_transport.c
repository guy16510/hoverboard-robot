/* SPDX-License-Identifier: GPL-3.0-only */
#include "gausstop_board.h"
#include "gausstop_swd_pulse.h"
#include "gausstop_swd_timing.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "gd32f1x0.h"

#if !defined(GS_REMOTE_TRANSPORT_SWD) || GS_REMOTE_TRANSPORT_SWD != 1
#error "GS_REMOTE_TRANSPORT_SWD must be enabled"
#endif

enum {
  GS_SWD_UART_TIMER_HZ = GS_SWD_UART_TIMER_HZ_FOR(GS_SYSTEM_CLOCK_HZ),
  GS_SWD_UART_BIT_TICKS = GS_SWD_UART_BIT_TICKS_FOR(GS_SYSTEM_CLOCK_HZ),
  GS_SWD_UART_RX_BUFFER_SIZE = 64,
  GS_SWD_UART_TX_BUFFER_SIZE = 128,
  GS_SWD_TAKEOVER_DELAY_MS = 2000,
};

_Static_assert(GS_SWD_UART_TIMER_HZ == 1000000u,
               "SWD feedback timer must run at 1 MHz");
_Static_assert(GS_SWD_UART_BIT_TICKS == 52u,
               "19,200-baud feedback must use 52 timer ticks");
_Static_assert((GS_SWD_UART_RX_BUFFER_SIZE &
                (GS_SWD_UART_RX_BUFFER_SIZE - 1u)) == 0u,
               "RX buffer size must be a power of two");
_Static_assert((GS_SWD_UART_TX_BUFFER_SIZE &
                (GS_SWD_UART_TX_BUFFER_SIZE - 1u)) == 0u,
               "TX buffer size must be a power of two");

static volatile uint8_t rx_buffer[GS_SWD_UART_RX_BUFFER_SIZE];
static volatile uint8_t rx_head;
static volatile uint8_t rx_tail;
static volatile uint32_t rx_byte_count;
static volatile uint32_t rx_overflow_count;
static volatile uint32_t rx_framing_error_count;
static volatile bool rx_low_active;
static volatile uint32_t rx_low_started_us;
static gs_swd_pulse_decoder rx_decoder;

static volatile uint8_t tx_buffer[GS_SWD_UART_TX_BUFFER_SIZE];
static volatile uint8_t tx_head;
static volatile uint8_t tx_tail;
static volatile uint32_t tx_byte_count;
static volatile uint32_t tx_overflow_count;
static volatile bool tx_active;
static volatile bool tx_timer_running;
static volatile uint8_t tx_byte;
static volatile uint8_t tx_bit;

void __real_gs_board_uart_init(gs_board_uart uart, bool transmit_enabled);
bool __real_gs_board_uart_read(gs_board_uart uart, uint8_t *byte);
bool __real_gs_board_uart_write(gs_board_uart uart, const uint8_t *bytes,
                                uint32_t length);
void __real_gs_board_uart_get_stats(gs_board_uart uart,
                                    gs_board_uart_stats *stats);

static void configure_tx_timer(void) {
  rcu_periph_clock_enable(RCU_TIMER13);
  timer_deinit(TIMER13);
  timer_parameter_struct timer = {0};
  timer.prescaler = GS_SWD_UART_TIMER_DIVIDER - 1u;
  timer.alignedmode = TIMER_COUNTER_EDGE;
  timer.counterdirection = TIMER_COUNTER_UP;
  timer.period = GS_SWD_UART_BIT_TICKS - 1u;
  timer.clockdivision = TIMER_CKDIV_DIV1;
  timer_init(TIMER13, &timer);
  timer_auto_reload_shadow_disable(TIMER13);
  timer_interrupt_flag_clear(TIMER13, TIMER_INT_FLAG_UP);
  timer_interrupt_enable(TIMER13, TIMER_INT_UP);
  timer_disable(TIMER13);
  nvic_irq_enable(TIMER13_IRQn, 2u, 0u);
}

static void push_rx_byte(uint8_t byte) {
  const uint8_t next =
      (uint8_t)((rx_head + 1u) & (GS_SWD_UART_RX_BUFFER_SIZE - 1u));
  if (next == rx_tail) {
    ++rx_overflow_count;
    return;
  }
  rx_buffer[rx_head] = byte;
  rx_head = next;
  ++rx_byte_count;
}

static void push_rx_frame(const uint8_t frame[GS_SWD_PULSE_FRAME_BYTES]) {
  for (uint8_t index = 0u; index < GS_SWD_PULSE_FRAME_BYTES; ++index) {
    push_rx_byte(frame[index]);
  }
}

static bool pop_tx_byte(uint8_t *byte) {
  if (byte == NULL || tx_tail == tx_head) {
    return false;
  }
  *byte = tx_buffer[tx_tail];
  tx_tail = (uint8_t)((tx_tail + 1u) & (GS_SWD_UART_TX_BUFFER_SIZE - 1u));
  return true;
}

static void service_rx_edge(void) {
  const uint32_t now_us = gs_board_micros();
  if (gpio_input_bit_get(GPIOA, GPIO_PIN_13) == RESET) {
    rx_low_started_us = now_us;
    rx_low_active = true;
    return;
  }
  if (!rx_low_active) {
    ++rx_framing_error_count;
    return;
  }
  rx_low_active = false;
  const uint32_t width_us = now_us - rx_low_started_us;
  const uint16_t bounded_width =
      width_us > UINT16_MAX ? UINT16_MAX : (uint16_t)width_us;
  uint8_t frame[GS_SWD_PULSE_FRAME_BYTES];
  const gs_swd_pulse_result result =
      gs_swd_pulse_decoder_feed(&rx_decoder, bounded_width, frame);
  if (result == GS_SWD_PULSE_FRAME) {
    push_rx_frame(frame);
  } else if (result == GS_SWD_PULSE_ERROR) {
    ++rx_framing_error_count;
  }
}

void EXTI4_15_IRQHandler(void) {
  gs_board_hall_exti_service();
  if (exti_interrupt_flag_get(EXTI_13) == RESET) {
    return;
  }
  exti_interrupt_flag_clear(EXTI_13);
  service_rx_edge();
}

static void drive_tx_bit(void) {
  if (!tx_active) {
    uint8_t next_byte = 0u;
    if (!pop_tx_byte(&next_byte)) {
      gpio_bit_set(GPIOA, GPIO_PIN_14);
      tx_timer_running = false;
      timer_disable(TIMER13);
      return;
    }
    tx_byte = next_byte;
    tx_active = true;
    tx_bit = 0u;
  }
  if (tx_bit == 0u) {
    gpio_bit_reset(GPIOA, GPIO_PIN_14);
  } else if (tx_bit <= 8u) {
    if ((tx_byte & (uint8_t)(1u << (tx_bit - 1u))) != 0u) {
      gpio_bit_set(GPIOA, GPIO_PIN_14);
    } else {
      gpio_bit_reset(GPIOA, GPIO_PIN_14);
    }
  } else {
    gpio_bit_set(GPIOA, GPIO_PIN_14);
  }
  ++tx_bit;
  if (tx_bit > 9u) {
    tx_active = false;
    ++tx_byte_count;
  }
}

void TIMER13_IRQHandler(void) {
  if (timer_interrupt_flag_get(TIMER13, TIMER_INT_FLAG_UP) == RESET) {
    return;
  }
  timer_interrupt_flag_clear(TIMER13, TIMER_INT_FLAG_UP);
  drive_tx_bit();
}

static void init_remote(bool transmit_enabled) {
  while (gs_board_millis() < GS_SWD_TAKEOVER_DELAY_MS) {
  }
  rx_head = 0u;
  rx_tail = 0u;
  rx_byte_count = 0u;
  rx_overflow_count = 0u;
  rx_framing_error_count = 0u;
  rx_low_active = false;
  rx_low_started_us = 0u;
  gs_swd_pulse_decoder_init(&rx_decoder);
  tx_head = 0u;
  tx_tail = 0u;
  tx_byte_count = 0u;
  tx_overflow_count = 0u;
  tx_active = false;
  tx_timer_running = false;
  rcu_periph_clock_enable(RCU_GPIOA);
  rcu_periph_clock_enable(RCU_CFGCMP);
  gpio_bit_set(GPIOA, GPIO_PIN_14);
  gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_14);
  gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_14);
  gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_13);
  syscfg_exti_line_config(EXTI_SOURCE_GPIOA, EXTI_SOURCE_PIN13);
  exti_init(EXTI_13, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
  exti_interrupt_flag_clear(EXTI_13);
  configure_tx_timer();
  NVIC_SetPriority(SysTick_IRQn, 3u);
  nvic_irq_enable(EXTI4_15_IRQn, 0u, 0u);
  (void)transmit_enabled;
}

void __wrap_gs_board_uart_init(gs_board_uart uart, bool transmit_enabled) {
  if (uart == GS_UART_REMOTE) {
    init_remote(transmit_enabled);
  } else {
    __real_gs_board_uart_init(uart, transmit_enabled);
  }
}

bool __wrap_gs_board_uart_read(gs_board_uart uart, uint8_t *byte) {
  if (uart != GS_UART_REMOTE) {
    return __real_gs_board_uart_read(uart, byte);
  }
  if (byte == NULL || rx_tail == rx_head) {
    return false;
  }
  *byte = rx_buffer[rx_tail];
  rx_tail = (uint8_t)((rx_tail + 1u) & (GS_SWD_UART_RX_BUFFER_SIZE - 1u));
  return true;
}

bool __wrap_gs_board_uart_write(gs_board_uart uart, const uint8_t *bytes,
                                uint32_t length) {
  if (uart != GS_UART_REMOTE) {
    return __real_gs_board_uart_write(uart, bytes, length);
  }
  if (bytes == NULL || length == 0u) {
    return false;
  }
  const uint8_t head = tx_head;
  const uint8_t tail = tx_tail;
  const uint8_t used =
      (uint8_t)((head - tail) & (GS_SWD_UART_TX_BUFFER_SIZE - 1u));
  const uint32_t available = GS_SWD_UART_TX_BUFFER_SIZE - 1u - used;
  if (length > available) {
    ++tx_overflow_count;
    return false;
  }
  uint8_t next = head;
  for (uint32_t index = 0u; index < length; ++index) {
    tx_buffer[next] = bytes[index];
    next = (uint8_t)((next + 1u) & (GS_SWD_UART_TX_BUFFER_SIZE - 1u));
  }
  tx_head = next;
  if (!tx_timer_running) {
    tx_timer_running = true;
    timer_counter_value_config(TIMER13, 0u);
    timer_interrupt_flag_clear(TIMER13, TIMER_INT_FLAG_UP);
    timer_enable(TIMER13);
  }
  return true;
}

void __wrap_gs_board_uart_get_stats(gs_board_uart uart,
                                    gs_board_uart_stats *stats) {
  if (uart != GS_UART_REMOTE) {
    __real_gs_board_uart_get_stats(uart, stats);
    return;
  }
  if (stats == NULL) {
    return;
  }
  stats->rx_bytes = rx_byte_count;
  stats->tx_bytes = tx_byte_count;
  stats->rx_overflows = rx_overflow_count;
  stats->tx_overflows = tx_overflow_count;
  stats->framing_errors = rx_framing_error_count;
}

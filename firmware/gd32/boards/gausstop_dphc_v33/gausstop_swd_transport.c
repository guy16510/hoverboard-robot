/* SPDX-License-Identifier: GPL-3.0-only */
#include "gausstop_board.h"

#include <stddef.h>
#include <stdint.h>

#include "gd32f1x0.h"

#if !defined(GS_REMOTE_TRANSPORT_SWD) || GS_REMOTE_TRANSPORT_SWD != 1
#error "GS_REMOTE_TRANSPORT_SWD must be enabled"
#endif

enum {
  GS_SWD_UART_BAUD = 38400,
  GS_SWD_UART_BIT_TICKS = 26,
  GS_SWD_UART_FIRST_SAMPLE_TICKS = 39,
  GS_SWD_UART_RX_BUFFER_SIZE = 64,
  GS_SWD_TAKEOVER_DELAY_MS = 2000,
};

static volatile uint8_t rx_buffer[GS_SWD_UART_RX_BUFFER_SIZE];
static volatile uint8_t rx_head;
static volatile uint8_t rx_tail;

void __real_gs_board_uart_init(gs_board_uart uart, bool transmit_enabled);
bool __real_gs_board_uart_read(gs_board_uart uart, uint8_t *byte);
bool __real_gs_board_uart_write(gs_board_uart uart, const uint8_t *bytes,
                                uint32_t length);

static void wait_timer1_offset(uint16_t start, uint16_t offset) {
  while ((uint16_t)((uint16_t)timer_counter_read(TIMER1) - start) < offset) {
  }
}

static void push_byte(uint8_t byte) {
  const uint8_t next =
      (uint8_t)((rx_head + 1u) & (GS_SWD_UART_RX_BUFFER_SIZE - 1u));
  if (next != rx_tail) {
    rx_buffer[rx_head] = byte;
    rx_head = next;
  }
}

void EXTI4_15_IRQHandler(void) {
  if (exti_interrupt_flag_get(EXTI_13) == RESET) {
    return;
  }
  exti_interrupt_flag_clear(EXTI_13);
  if (gpio_input_bit_get(GPIOA, GPIO_PIN_13) != RESET) {
    return;
  }

  const uint16_t start = (uint16_t)timer_counter_read(TIMER1);
  uint8_t value = 0u;
  for (uint8_t bit = 0u; bit < 8u; ++bit) {
    wait_timer1_offset(
        start, (uint16_t)(GS_SWD_UART_FIRST_SAMPLE_TICKS +
                          (uint16_t)bit * GS_SWD_UART_BIT_TICKS));
    if (gpio_input_bit_get(GPIOA, GPIO_PIN_13) != RESET) {
      value |= (uint8_t)(1u << bit);
    }
  }
  push_byte(value);
  exti_interrupt_flag_clear(EXTI_13);
}

static void init_remote(bool transmit_enabled) {
  while (gs_board_millis() < GS_SWD_TAKEOVER_DELAY_MS) {
  }

  rx_head = 0u;
  rx_tail = 0u;
  rcu_periph_clock_enable(RCU_GPIOA);
  rcu_periph_clock_enable(RCU_CFGCMP);
  rcu_periph_clock_enable(RCU_USART0);

  /* MASTER TX is PA14/SWCLK using USART0_TX alternate function 1. */
  gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_14);
  gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_14);
  gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
                          GPIO_PIN_14);

  /* MASTER RX is PA13/SWDIO using an interrupt-sampled software receiver. */
  gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_13);
  syscfg_exti_line_config(EXTI_SOURCE_GPIOA, EXTI_SOURCE_PIN13);
  exti_init(EXTI_13, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
  exti_interrupt_flag_clear(EXTI_13);
  NVIC_SetPriority(SysTick_IRQn, 3u);
  nvic_irq_enable(EXTI4_15_IRQn, 0u, 0u);

  usart_deinit(USART0);
  usart_baudrate_set(USART0, GS_SWD_UART_BAUD);
  usart_word_length_set(USART0, USART_WL_8BIT);
  usart_stop_bit_set(USART0, USART_STB_1BIT);
  usart_parity_config(USART0, USART_PM_NONE);
  usart_receive_config(USART0, USART_RECEIVE_DISABLE);
  usart_transmit_config(USART0, transmit_enabled ? USART_TRANSMIT_ENABLE
                                                 : USART_TRANSMIT_DISABLE);
  usart_enable(USART0);
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
  if (bytes == NULL) {
    return false;
  }
  for (uint32_t index = 0u; index < length; ++index) {
    uint32_t timeout = 100000u;
    while (usart_flag_get(USART0, USART_FLAG_TBE) == RESET && timeout != 0u) {
      --timeout;
    }
    if (timeout == 0u) {
      return false;
    }
    usart_data_transmit(USART0, bytes[index]);
  }
  return true;
}

/* SPDX-License-Identifier: GPL-3.0-only */
#ifndef GS_TRANSPORT_H
#define GS_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
  void *context;
  bool (*send)(void *context, const uint8_t *bytes, size_t length);
} gs_tx_port;

typedef struct {
  void *context;
  bool (*receive)(void *context, uint8_t *byte);
} gs_rx_port;

#endif

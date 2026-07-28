/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

extern "C" {
#include "gs_protocol.h"
}

namespace gs::balance {

class ControllerFaultClear {
public:
  void request();
  void apply(gs_esp_command &command) const;
  bool observe(const gs_master_feedback &feedback, uint16_t expected_sequence,
               bool command_sent);
  bool pending() const;

private:
  bool pending_ = false;
};

} // namespace gs::balance

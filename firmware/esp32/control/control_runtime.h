/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include "balance_controller.h"
#include "transport_test_limits.h"

namespace gs::balance {

class ControlRuntime {
public:
  ControlRuntime(IBalanceController &controller, IMotorCommandSink &motor_sink,
                 bool dry_run);

  BalanceOutput step(const BalanceInput &input, bool output_permitted);
  BalanceOutput stepDirect(float left, float right, bool output_permitted);
  bool dryRun() const;

private:
  IBalanceController &controller_;
  IMotorCommandSink &motor_sink_;
  bool dry_run_;
  bool output_was_permitted_ = false;
};

} // namespace gs::balance

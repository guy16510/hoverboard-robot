/* SPDX-License-Identifier: GPL-3.0-only */
#include "control_runtime.h"

namespace gs::balance {

ControlRuntime::ControlRuntime(IBalanceController &controller,
                               IMotorCommandSink &motor_sink, bool dry_run)
    : controller_(controller), motor_sink_(motor_sink), dry_run_(dry_run) {}

BalanceOutput ControlRuntime::step(const BalanceInput &input,
                                   bool output_permitted) {
  if (output_permitted && !output_was_permitted_) {
    controller_.reset(0.0f, 0.0f);
  }
  const BalanceOutput output = controller_.update(input);
  if (!output_permitted) {
    controller_.clear();
  }
  output_was_permitted_ = output_permitted;
  MotorCommand command;
  command.enabled = output_permitted && output.valid && !dry_run_;
  if (command.enabled) {
    command.left = output.left;
    command.right = output.right;
  }
  motor_sink_.write(command);
  return output;
}

bool ControlRuntime::dryRun() const { return dry_run_; }

} // namespace gs::balance

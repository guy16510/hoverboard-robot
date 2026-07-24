/* SPDX-License-Identifier: GPL-3.0-only */
#include "command_arbiter.h"

namespace gs::balance {

void CommandArbiter::submit(const ControlRequest &request) {
  const bool serial_preempts_web = request.source == CommandSource::kSerial &&
                                   active_source_ == CommandSource::kWeb;
  if (request.emergency_stop || serial_preempts_web) {
    clearOwnership();
  }
  if (request.source == CommandSource::kSerial) {
    if (serial_seen_ && request.sequence == serial_.sequence) {
      return;
    }
    serial_ = request;
    serial_seen_ = true;
    serial_pending_ = true;
  }
  if (request.source == CommandSource::kWeb) {
    if (web_seen_ && request.sequence == web_.sequence) {
      return;
    }
    web_ = request;
    web_seen_ = true;
    web_pending_ = active_source_ != CommandSource::kSerial;
  }
}

void CommandArbiter::disconnect(CommandSource source) {
  if (source == CommandSource::kSerial) {
    serial_ = {};
    serial_pending_ = false;
  }
  if (source == CommandSource::kWeb) {
    web_ = {};
    web_pending_ = false;
  }
  if (active_source_ == source) {
    clearOwnership();
  }
}

void CommandArbiter::setLocalDisarm(bool active) {
  if (active && !local_disarm_) {
    clearOwnership();
  }
  local_disarm_ = active;
}

void CommandArbiter::setEmergencyStop(bool active) {
  if (active && !emergency_stop_) {
    clearOwnership();
  }
  emergency_stop_ = active;
}

ControlRequest CommandArbiter::resolve(uint64_t now_us) {
  if (emergency_stop_) {
    ControlRequest result = stopped();
    result.emergency_stop = true;
    result.disarm = true;
    result.source = CommandSource::kLocal;
    return result;
  }
  if (local_disarm_) {
    ControlRequest result = stopped();
    result.disarm = true;
    result.source = CommandSource::kLocal;
    return result;
  }
  if (active_source_ == CommandSource::kSerial) {
    if (active(serial_, now_us)) {
      return serial_;
    }
    clearOwnership();
    return stopped();
  }
  if (active_source_ == CommandSource::kWeb) {
    if (active(web_, now_us)) {
      return web_;
    }
    clearOwnership();
    return stopped();
  }
  if (serial_pending_) {
    serial_pending_ = false;
    web_pending_ = false;
    if (active(serial_, now_us)) {
      active_source_ = CommandSource::kSerial;
      return serial_;
    }
  }
  if (web_pending_) {
    web_pending_ = false;
    if (active(web_, now_us)) {
      active_source_ = CommandSource::kWeb;
      return web_;
    }
  }
  return stopped();
}

CommandSource CommandArbiter::activeSource() const { return active_source_; }

bool CommandArbiter::active(const ControlRequest &request, uint64_t now_us) {
  return request.source != CommandSource::kNone &&
         request.lease_expires_us >= now_us;
}

ControlRequest CommandArbiter::stopped() { return {}; }

void CommandArbiter::clearOwnership() {
  active_source_ = CommandSource::kNone;
  serial_pending_ = false;
  web_pending_ = false;
}

} // namespace gs::balance

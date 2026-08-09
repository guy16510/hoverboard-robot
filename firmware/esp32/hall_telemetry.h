#pragma once

#include <cstdint>

#include "hall_sensor.h"
#include "protocol_contract.h"
#include "serial_protocol.h"

namespace trashbot::sensors {

inline void encodeHallSnapshot(const HallSnapshot &hall, uint8_t *payload) {
  payload[0] = hall.state;
  payload[1] = static_cast<uint8_t>((hall.valid ? 1u : 0u) |
                                    (hall.moving ? (1u << 1u) : 0u));
  protocol::writeU16(payload + 2, 0);
  protocol::writeU32(payload + 4, hall.transitions);
  protocol::writeU32(payload + 8, hall.invalidStates);
  protocol::writeU32(payload + 12, hall.skippedTransitions);
  protocol::writeU32(payload + 16, hall.rateMilliHz);
  protocol::writeU32(payload + 20, hall.lastTransitionAgeMs);
}

}  // namespace trashbot::sensors

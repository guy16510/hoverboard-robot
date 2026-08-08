#include <Arduino.h>
#include <algorithm>
#include <cmath>
#include <cstdint>

#include "board_config.h"
#include "motor_driver.h"
#include "protocol_contract.h"
#include "serial_protocol.h"
#include "ultrasonic_array.h"

namespace {

using trashbot::drive::DifferentialDrive;
using trashbot::protocol::FrameParser;
using trashbot::protocol::FrameView;
using trashbot::protocol::FrameWriter;
using trashbot::sensors::UltrasonicArray;
namespace board = trashbot::board;
namespace protocol = trashbot::protocol;

DifferentialDrive drivetrain;
UltrasonicArray ultrasonic;
FrameWriter writer(Serial);
FrameParser parser;

bool armed = false;
bool leaseActive = false;
uint8_t operatingMode = 0;
uint32_t leaseExpiresMs = 0;
uint32_t leaseId = 0;
int16_t requestedLinearMilli = 0;
int16_t requestedYawMilli = 0;
uint32_t rejectedCommands = 0;
uint32_t lastUltrasonicTelemetryMs = 0;
uint16_t telemetrySequence = 0;

void stopMotion(bool disarm) {
  requestedLinearMilli = 0;
  requestedYawMilli = 0;
  drivetrain.stop();
  if (disarm) {
    armed = false;
    leaseActive = false;
  }
}

bool leaseExpired(uint32_t nowMs) {
  return leaseActive &&
         static_cast<int32_t>(nowMs - leaseExpiresMs) >= 0;
}

void updateMotorTargets(uint32_t nowMs) {
  if (!armed || !leaseActive || leaseExpired(nowMs)) {
    if (leaseExpired(nowMs)) {
      leaseActive = false;
    }
    stopMotion(false);
    return;
  }

  const float linear = std::max(
      -1.0f,
      std::min(1.0f, static_cast<float>(requestedLinearMilli) /
                          board::kMaxLinearMilli));
  const float yaw = std::max(
      -1.0f,
      std::min(1.0f,
               static_cast<float>(requestedYawMilli) / board::kMaxYawMilli));
  drivetrain.setNormalized(linear, yaw);
}

void sendCapabilities(uint16_t sequence) {
  uint8_t payload[protocol::kCapabilitiesPayloadBytes] = {};
  payload[0] = protocol::kVersion;
  payload[1] = 0;  // dry-run false
  payload[2] = 0;  // web-control false
  payload[3] = static_cast<uint8_t>(1u << protocol::kDriveMode);
  protocol::writeU16(payload + 4, 50);  // control Hz
  protocol::writeU16(payload + 6, 50);  // motor output Hz target
  protocol::writeU16(payload + 8, protocol::kMaximumPayload);
  protocol::writeU16(payload + 10, 0);  // no runtime config keys
  writer.send(protocol::kCapabilities, sequence, payload, sizeof(payload));
}

void sendStatus(uint16_t sequence) {
  uint8_t payload[protocol::kStatusPayloadBytes] = {};
  const bool driving = armed && leaseActive &&
                       (std::abs(requestedLinearMilli) > 0 ||
                        std::abs(requestedYawMilli) > 0);
  payload[0] = armed ? (driving ? 4 : 3) : 2;
  payload[1] = operatingMode;
  payload[2] = leaseActive ? 2 : 0;
  payload[3] = static_cast<uint8_t>((armed ? (1u << 4) : 0u) | (1u << 6));
  protocol::writeU32(payload + 4, 0);  // faults
  protocol::writeU32(payload + 8, 0);  // loop overruns
  protocol::writeU32(payload + 12,
                     parser.rejectedFrames() + rejectedCommands);
  writer.send(protocol::kStatus, sequence, payload, sizeof(payload));
}

void sendUltrasonic(uint16_t sequence, uint32_t nowMs) {
  uint8_t payload[protocol::kUltrasonicPayloadBytes] = {};
  ultrasonic.encode(payload, nowMs);
  writer.send(protocol::kUltrasonic, sequence, payload, sizeof(payload));
}

bool decodeMotion(const uint8_t *payload, uint16_t length, uint32_t nowMs) {
  if (length != protocol::kMotionPayloadBytes) {
    return false;
  }
  const uint16_t lifetimeMs = protocol::readU16(payload + 8);
  if (lifetimeMs == 0 || lifetimeMs > 2000) {
    return false;
  }
  requestedLinearMilli = protocol::readI16(payload);
  requestedYawMilli = protocol::readI16(payload + 2);
  leaseId = protocol::readU32(payload + 4);
  leaseExpiresMs = nowMs + lifetimeMs;
  leaseActive = true;
  return true;
}

void reject(uint8_t type, uint16_t sequence, uint8_t code,
            uint16_t detail = 0) {
  ++rejectedCommands;
  writer.error(type, sequence, code, detail);
}

void handleFrame(const FrameView &frame, uint32_t nowMs) {
  switch (frame.type) {
    case protocol::kHello:
      stopMotion(true);
      operatingMode = 0;
      sendCapabilities(frame.sequence);
      return;

    case protocol::kCapabilities:
      sendCapabilities(frame.sequence);
      return;

    case protocol::kSetOperatingMode:
      if (frame.payloadLength != 1 ||
          frame.payload[0] != protocol::kDriveMode) {
        reject(frame.type, frame.sequence, 4,
               frame.payloadLength == 1 ? frame.payload[0] : 0xFFFFu);
        return;
      }
      stopMotion(true);
      operatingMode = protocol::kDriveMode;
      writer.acknowledge(frame.type, frame.sequence);
      return;

    case protocol::kArm:
      if (operatingMode != protocol::kDriveMode || !leaseActive ||
          leaseExpired(nowMs)) {
        reject(frame.type, frame.sequence, 6, operatingMode);
        return;
      }
      armed = true;
      writer.acknowledge(frame.type, frame.sequence);
      return;

    case protocol::kDisarm:
      stopMotion(true);
      writer.acknowledge(frame.type, frame.sequence);
      return;

    case protocol::kStop:
      stopMotion(false);
      writer.acknowledge(frame.type, frame.sequence);
      return;

    case protocol::kEmergencyStop:
      stopMotion(true);
      writer.acknowledge(frame.type, frame.sequence);
      return;

    case protocol::kClearFault:
      writer.acknowledge(frame.type, frame.sequence);
      return;

    case protocol::kSetVelocityYaw:
    case protocol::kHeartbeat:
      if (operatingMode != protocol::kDriveMode ||
          !decodeMotion(frame.payload, frame.payloadLength, nowMs)) {
        reject(frame.type, frame.sequence, 1, frame.payloadLength);
        return;
      }
      writer.acknowledge(frame.type, frame.sequence);
      return;

    case protocol::kStatus:
      sendStatus(frame.sequence);
      return;

    case protocol::kUltrasonic:
      sendUltrasonic(frame.sequence, nowMs);
      return;

    default:
      reject(frame.type, frame.sequence, 4, frame.type);
      return;
  }
}

}  // namespace

void setup() {
  drivetrain.begin();
  ultrasonic.begin();
  Serial.begin(board::kSerialBaud);
  delay(50);
  stopMotion(true);
}

void loop() {
  const uint32_t nowMs = millis();

  parser.service(Serial, nowMs,
                 [](const FrameView &frame, uint32_t timestampMs) {
                   handleFrame(frame, timestampMs);
                 });
  updateMotorTargets(nowMs);
  drivetrain.service(nowMs);
  ultrasonic.service(nowMs);

  if (nowMs - lastUltrasonicTelemetryMs >=
      board::kUltrasonicTelemetryMs) {
    lastUltrasonicTelemetryMs = nowMs;
    sendUltrasonic(telemetrySequence++, nowMs);
  }

  delay(1);
}

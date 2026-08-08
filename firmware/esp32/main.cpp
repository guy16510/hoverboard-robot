#include <Arduino.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace {

constexpr uint32_t kSerialBaud = 115200;

// ESP32 DevKit / ESP32-WROOM-32 motor-controller pins.
constexpr uint8_t kLeftThrottlePin = 25;   // DAC1
constexpr uint8_t kRightThrottlePin = 26;  // DAC2
constexpr uint8_t kLeftReversePin = 27;    // optocoupler / relay input
constexpr uint8_t kRightReversePin = 14;   // optocoupler / relay input
constexpr uint8_t kLeftBrakePin = 33;      // optocoupler / relay input
constexpr uint8_t kRightBrakePin = 32;     // optocoupler / relay input

// Three HC-SR04-style ultrasonic sensors. Echo is 5 V on common modules,
// so each echo MUST be level-shifted / divided to <= 3.3 V before the ESP32.
constexpr uint8_t kFrontTrigPin = 16;
constexpr uint8_t kFrontEchoPin = 34;
constexpr uint8_t kLeftTrigPin = 17;
constexpr uint8_t kLeftEchoPin = 35;
constexpr uint8_t kRightTrigPin = 18;
constexpr uint8_t kRightEchoPin = 39;

#ifndef WINXU_LEFT_MOTOR_SIGN
#define WINXU_LEFT_MOTOR_SIGN 1
#endif
#ifndef WINXU_RIGHT_MOTOR_SIGN
// Hoverboard wheels are normally mounted as mirror images. Override this to
// +1 at build time if your right wheel is already mechanically/electrically
// aligned with the left wheel.
#define WINXU_RIGHT_MOTOR_SIGN -1
#endif
#ifndef WINXU_SWITCH_ACTIVE_HIGH
// HIGH means the external optocoupler/relay closes the controller switch pair.
#define WINXU_SWITCH_ACTIVE_HIGH 1
#endif
#ifndef WINXU_RUNTIME_REVERSE
// Set to 0 if the specific controller harness does not have a dedicated
// run-time reverse connector. Negative motor commands will then be rejected.
#define WINXU_RUNTIME_REVERSE 1
#endif

static_assert(WINXU_LEFT_MOTOR_SIGN == 1 || WINXU_LEFT_MOTOR_SIGN == -1,
              "WINXU_LEFT_MOTOR_SIGN must be +1 or -1");
static_assert(WINXU_RIGHT_MOTOR_SIGN == 1 || WINXU_RIGHT_MOTOR_SIGN == -1,
              "WINXU_RIGHT_MOTOR_SIGN must be +1 or -1");

// Matches donkeycar/config/robot.yaml. The Pi sends physical milli-units.
constexpr float kMaxLinearMilli = 350.0f;  // 0.35 m/s
constexpr float kMaxYawMilli = 800.0f;     // 0.8 rad/s

// ESP32 DAC tops out around the 3.3 V rail, so this intentionally does not
// request the controller's full 4.2 V throttle range. Start conservatively.
constexpr float kDacReferenceVolts = 3.3f;
constexpr float kThrottleIdleVolts = 0.85f;
constexpr float kThrottleStartVolts = 1.15f;
constexpr float kThrottleMaxVolts = 3.15f;
constexpr float kCommandDeadband = 0.04f;
constexpr float kThrottleSlewPerSecond = 1.5f;
constexpr uint32_t kDirectionBrakeBeforeMs = 180;
constexpr uint32_t kDirectionBrakeAfterMs = 180;

constexpr uint32_t kUltrasonicPingSpacingMs = 40;
constexpr uint32_t kUltrasonicTimeoutUs = 15000;  // about 2.5 m max range
constexpr uint16_t kUltrasonicMinMm = 25;
constexpr uint16_t kUltrasonicMaxMm = 2500;
constexpr uint32_t kUltrasonicStaleMs = 500;
constexpr uint32_t kUltrasonicTelemetryMs = 100;

constexpr uint8_t kMarker0 = 0xA5;
constexpr uint8_t kMarker1 = 0x5A;
constexpr uint8_t kProtocolVersion = 1;
constexpr uint16_t kMaximumPayload = 48;
constexpr size_t kMaximumFrame = 11 + kMaximumPayload;

enum MessageType : uint8_t {
  kHello = 0x01,
  kCapabilities = 0x02,
  kArm = 0x10,
  kDisarm = 0x11,
  kStop = 0x12,
  kEmergencyStop = 0x13,
  kClearFault = 0x14,
  kSetOperatingMode = 0x15,
  kSetVelocityYaw = 0x22,
  kHeartbeat = 0x23,
  kStatus = 0x30,
  kUltrasonic = 0x35,
  kAcknowledgment = 0x7E,
  kError = 0x7F,
};

constexpr uint8_t kDriveMode = 2;

uint16_t crc16(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t index = 0; index < length; ++index) {
    crc ^= static_cast<uint16_t>(data[index]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000u) != 0u
                ? static_cast<uint16_t>((crc << 1) ^ 0x1021u)
                : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

uint16_t readU16(const uint8_t *data) {
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8);
}

int16_t readI16(const uint8_t *data) {
  return static_cast<int16_t>(readU16(data));
}

uint32_t readU32(const uint8_t *data) {
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

void writeU16(uint8_t *data, uint16_t value) {
  data[0] = static_cast<uint8_t>(value & 0xFFu);
  data[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
}

void writeU32(uint8_t *data, uint32_t value) {
  data[0] = static_cast<uint8_t>(value & 0xFFu);
  data[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
  data[2] = static_cast<uint8_t>((value >> 16) & 0xFFu);
  data[3] = static_cast<uint8_t>((value >> 24) & 0xFFu);
}

void sendFrame(uint8_t type, uint16_t sequence, const uint8_t *payload,
               uint16_t payloadLength) {
  if (payloadLength > kMaximumPayload) {
    return;
  }
  uint8_t frame[kMaximumFrame] = {};
  frame[0] = kMarker0;
  frame[1] = kMarker1;
  frame[2] = kProtocolVersion;
  frame[3] = type;
  frame[4] = 0;
  writeU16(frame + 5, sequence);
  writeU16(frame + 7, payloadLength);
  if (payloadLength != 0 && payload != nullptr) {
    memcpy(frame + 9, payload, payloadLength);
  }
  const uint16_t checksum = crc16(frame + 2, 7 + payloadLength);
  writeU16(frame + 9 + payloadLength, checksum);
  Serial.write(frame, 11 + payloadLength);
}

void acknowledge(uint8_t requestType, uint16_t sequence, uint8_t status = 0) {
  const uint8_t payload[2] = {requestType, status};
  sendFrame(kAcknowledgment, sequence, payload, sizeof(payload));
}

void sendError(uint8_t requestType, uint16_t sequence, uint8_t code,
               uint16_t detail = 0) {
  uint8_t payload[4] = {requestType, code, 0, 0};
  writeU16(payload + 2, detail);
  sendFrame(kError, sequence, payload, sizeof(payload));
}

uint8_t switchLevel(bool active) {
#if WINXU_SWITCH_ACTIVE_HIGH
  return active ? HIGH : LOW;
#else
  return active ? LOW : HIGH;
#endif
}

uint8_t voltsToDac(float volts) {
  const float bounded = std::max(0.0f, std::min(kDacReferenceVolts, volts));
  return static_cast<uint8_t>(lroundf((bounded / kDacReferenceVolts) * 255.0f));
}

struct MotorChannel {
  enum class Phase : uint8_t {
    kRun,
    kBrakeBeforeDirection,
    kBrakeAfterDirection,
  };

  uint8_t throttlePin;
  uint8_t reversePin;
  uint8_t brakePin;
  float target = 0.0f;
  float appliedMagnitude = 0.0f;
  bool reverse = false;
  bool pendingReverse = false;
  Phase phase = Phase::kRun;
  uint32_t phaseStartedMs = 0;
  uint32_t lastServiceMs = 0;
};

MotorChannel leftMotor{kLeftThrottlePin, kLeftReversePin, kLeftBrakePin};
MotorChannel rightMotor{kRightThrottlePin, kRightReversePin, kRightBrakePin};

void writeBrake(MotorChannel &motor, bool active) {
  digitalWrite(motor.brakePin, switchLevel(active));
}

void writeReverse(MotorChannel &motor, bool active) {
  digitalWrite(motor.reversePin, switchLevel(active));
}

void writeThrottle(MotorChannel &motor, float magnitude) {
  magnitude = std::max(0.0f, std::min(1.0f, magnitude));
  float volts = kThrottleIdleVolts;
  if (magnitude >= kCommandDeadband) {
    volts = kThrottleStartVolts +
            magnitude * (kThrottleMaxVolts - kThrottleStartVolts);
  }
  dacWrite(motor.throttlePin, voltsToDac(volts));
}

void hardStop(MotorChannel &motor) {
  motor.target = 0.0f;
  motor.appliedMagnitude = 0.0f;
  motor.phase = MotorChannel::Phase::kRun;
  writeThrottle(motor, 0.0f);
  writeBrake(motor, true);
}

float moveToward(float current, float target, float maximumDelta) {
  if (target > current) {
    return std::min(target, current + maximumDelta);
  }
  return std::max(target, current - maximumDelta);
}

void serviceMotor(MotorChannel &motor, uint32_t nowMs) {
  const uint32_t elapsedMs = motor.lastServiceMs == 0 ? 0 : nowMs - motor.lastServiceMs;
  motor.lastServiceMs = nowMs;
  const float maximumDelta =
      kThrottleSlewPerSecond * (static_cast<float>(elapsedMs) / 1000.0f);

  const bool wantsMotion = std::fabs(motor.target) >= kCommandDeadband;
  const bool desiredReverse = motor.target < 0.0f;
  const float desiredMagnitude = wantsMotion ? std::fabs(motor.target) : 0.0f;

#if !WINXU_RUNTIME_REVERSE
  if (desiredReverse) {
    hardStop(motor);
    return;
  }
#endif

  switch (motor.phase) {
    case MotorChannel::Phase::kRun:
      if (!wantsMotion) {
        motor.appliedMagnitude =
            moveToward(motor.appliedMagnitude, 0.0f, maximumDelta);
        writeThrottle(motor, motor.appliedMagnitude);
        if (motor.appliedMagnitude <= 0.001f) {
          motor.appliedMagnitude = 0.0f;
          writeThrottle(motor, 0.0f);
          writeBrake(motor, true);
        }
        return;
      }

      if (desiredReverse != motor.reverse) {
        motor.appliedMagnitude =
            moveToward(motor.appliedMagnitude, 0.0f, maximumDelta);
        writeThrottle(motor, motor.appliedMagnitude);
        if (motor.appliedMagnitude <= 0.001f) {
          motor.appliedMagnitude = 0.0f;
          motor.pendingReverse = desiredReverse;
          writeThrottle(motor, 0.0f);
          writeBrake(motor, true);
          motor.phase = MotorChannel::Phase::kBrakeBeforeDirection;
          motor.phaseStartedMs = nowMs;
        }
        return;
      }

      writeBrake(motor, false);
      motor.appliedMagnitude =
          moveToward(motor.appliedMagnitude, desiredMagnitude, maximumDelta);
      writeThrottle(motor, motor.appliedMagnitude);
      return;

    case MotorChannel::Phase::kBrakeBeforeDirection:
      writeThrottle(motor, 0.0f);
      writeBrake(motor, true);
      if (nowMs - motor.phaseStartedMs >= kDirectionBrakeBeforeMs) {
        motor.reverse = motor.pendingReverse;
        writeReverse(motor, motor.reverse);
        motor.phase = MotorChannel::Phase::kBrakeAfterDirection;
        motor.phaseStartedMs = nowMs;
      }
      return;

    case MotorChannel::Phase::kBrakeAfterDirection:
      writeThrottle(motor, 0.0f);
      writeBrake(motor, true);
      if (nowMs - motor.phaseStartedMs >= kDirectionBrakeAfterMs) {
        motor.phase = MotorChannel::Phase::kRun;
      }
      return;
  }
}

struct UltrasonicSensor {
  uint8_t triggerPin;
  uint8_t echoPin;
  uint16_t distanceMm = 0xFFFFu;
  uint32_t measuredAtMs = 0;
};

UltrasonicSensor ultrasonic[3] = {
    {kFrontTrigPin, kFrontEchoPin},
    {kLeftTrigPin, kLeftEchoPin},
    {kRightTrigPin, kRightEchoPin},
};
uint8_t nextUltrasonic = 0;
uint32_t lastUltrasonicPingMs = 0;
uint32_t lastUltrasonicTelemetryMs = 0;
uint16_t telemetrySequence = 0;

void serviceUltrasonic(uint32_t nowMs) {
  if (nowMs - lastUltrasonicPingMs < kUltrasonicPingSpacingMs) {
    return;
  }
  lastUltrasonicPingMs = nowMs;

  UltrasonicSensor &sensor = ultrasonic[nextUltrasonic];
  nextUltrasonic = static_cast<uint8_t>((nextUltrasonic + 1) % 3);

  digitalWrite(sensor.triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(sensor.triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensor.triggerPin, LOW);

  const uint32_t durationUs = pulseIn(sensor.echoPin, HIGH, kUltrasonicTimeoutUs);
  if (durationUs == 0) {
    sensor.distanceMm = 0xFFFFu;
  } else {
    const uint32_t distanceMm = (durationUs * 343u) / 2000u;
    sensor.distanceMm =
        distanceMm >= kUltrasonicMinMm && distanceMm <= kUltrasonicMaxMm
            ? static_cast<uint16_t>(distanceMm)
            : 0xFFFFu;
  }
  sensor.measuredAtMs = nowMs;
}

uint16_t freshDistance(const UltrasonicSensor &sensor, uint32_t nowMs) {
  if (sensor.measuredAtMs == 0 || nowMs - sensor.measuredAtMs > kUltrasonicStaleMs) {
    return 0xFFFFu;
  }
  return sensor.distanceMm;
}

void sendUltrasonic(uint16_t sequence, uint32_t nowMs) {
  uint8_t payload[8] = {};
  uint8_t validMask = 0;
  for (uint8_t index = 0; index < 3; ++index) {
    const uint16_t distance = freshDistance(ultrasonic[index], nowMs);
    writeU16(payload + (index * 2), distance);
    if (distance != 0xFFFFu) {
      validMask |= static_cast<uint8_t>(1u << index);
    }
  }
  payload[6] = validMask;  // bit 0 front, bit 1 left, bit 2 right
  payload[7] = 0;
  sendFrame(kUltrasonic, sequence, payload, sizeof(payload));
}

bool armed = false;
bool leaseActive = false;
uint8_t operatingMode = 0;
uint32_t leaseExpiresMs = 0;
uint32_t leaseId = 0;
int16_t requestedLinearMilli = 0;
int16_t requestedYawMilli = 0;
uint32_t rejectedFrames = 0;

void stopMotion(bool disarm) {
  requestedLinearMilli = 0;
  requestedYawMilli = 0;
  leftMotor.target = 0.0f;
  rightMotor.target = 0.0f;
  hardStop(leftMotor);
  hardStop(rightMotor);
  if (disarm) {
    armed = false;
    leaseActive = false;
  }
}

void updateMotorTargets(uint32_t nowMs) {
  if (!armed || !leaseActive || static_cast<int32_t>(nowMs - leaseExpiresMs) >= 0) {
    if (leaseActive && static_cast<int32_t>(nowMs - leaseExpiresMs) >= 0) {
      leaseActive = false;
    }
    stopMotion(false);
    return;
  }

  const float linear = std::max(-1.0f, std::min(1.0f,
      static_cast<float>(requestedLinearMilli) / kMaxLinearMilli));
  const float yaw = std::max(-1.0f, std::min(1.0f,
      static_cast<float>(requestedYawMilli) / kMaxYawMilli));

  float left = (linear + yaw) * static_cast<float>(WINXU_LEFT_MOTOR_SIGN);
  float right = (linear - yaw) * static_cast<float>(WINXU_RIGHT_MOTOR_SIGN);
  const float peak = std::max(std::fabs(left), std::fabs(right));
  if (peak > 1.0f) {
    left /= peak;
    right /= peak;
  }
  leftMotor.target = left;
  rightMotor.target = right;
}

void sendCapabilities(uint16_t sequence) {
  uint8_t payload[12] = {};
  payload[0] = kProtocolVersion;
  payload[1] = 0;  // dry run false
  payload[2] = 0;  // web control false
  payload[3] = static_cast<uint8_t>(1u << kDriveMode);
  writeU16(payload + 4, 50);  // control Hz
  writeU16(payload + 6, 50);  // motor output Hz target
  writeU16(payload + 8, kMaximumPayload);
  writeU16(payload + 10, 0);  // no runtime configuration keys
  sendFrame(kCapabilities, sequence, payload, sizeof(payload));
}

void sendStatus(uint16_t sequence, uint32_t nowMs) {
  uint8_t payload[16] = {};
  const bool driving = armed && leaseActive &&
                       (std::abs(requestedLinearMilli) > 0 || std::abs(requestedYawMilli) > 0);
  payload[0] = armed ? (driving ? 4 : 3) : 2;
  payload[1] = operatingMode;
  payload[2] = leaseActive ? 2 : 0;  // serial source
  payload[3] = static_cast<uint8_t>((armed ? (1u << 4) : 0u) | (1u << 6));
  writeU32(payload + 4, 0);  // faults
  writeU32(payload + 8, 0);  // loop overruns
  writeU32(payload + 12, rejectedFrames);
  sendFrame(kStatus, sequence, payload, sizeof(payload));
  (void)nowMs;
}

bool decodeMotion(const uint8_t *payload, uint16_t length, uint32_t nowMs) {
  if (length != 10) {
    return false;
  }
  const uint16_t lifetimeMs = readU16(payload + 8);
  if (lifetimeMs == 0 || lifetimeMs > 2000) {
    return false;
  }
  requestedLinearMilli = readI16(payload);
  requestedYawMilli = readI16(payload + 2);
  leaseId = readU32(payload + 4);
  leaseExpiresMs = nowMs + lifetimeMs;
  leaseActive = true;
  return true;
}

void handleFrame(uint8_t type, uint16_t sequence, const uint8_t *payload,
                 uint16_t length, uint32_t nowMs) {
  switch (type) {
    case kHello:
      stopMotion(true);
      operatingMode = 0;
      sendCapabilities(sequence);
      return;

    case kCapabilities:
      sendCapabilities(sequence);
      return;

    case kSetOperatingMode:
      if (length != 1 || payload[0] != kDriveMode) {
        ++rejectedFrames;
        sendError(type, sequence, 4, length == 1 ? payload[0] : 0xFFFFu);
        return;
      }
      stopMotion(true);
      operatingMode = kDriveMode;
      acknowledge(type, sequence);
      return;

    case kArm:
      if (operatingMode != kDriveMode || !leaseActive) {
        ++rejectedFrames;
        sendError(type, sequence, 6, operatingMode);
        return;
      }
      armed = true;
      acknowledge(type, sequence);
      return;

    case kDisarm:
      stopMotion(true);
      acknowledge(type, sequence);
      return;

    case kStop:
      stopMotion(false);
      acknowledge(type, sequence);
      return;

    case kEmergencyStop:
      stopMotion(true);
      acknowledge(type, sequence);
      return;

    case kClearFault:
      acknowledge(type, sequence);
      return;

    case kSetVelocityYaw:
    case kHeartbeat:
      if (operatingMode != kDriveMode || !decodeMotion(payload, length, nowMs)) {
        ++rejectedFrames;
        sendError(type, sequence, 1, length);
        return;
      }
      acknowledge(type, sequence);
      return;

    case kStatus:
      sendStatus(sequence, nowMs);
      return;

    case kUltrasonic:
      sendUltrasonic(sequence, nowMs);
      return;

    default:
      ++rejectedFrames;
      sendError(type, sequence, 4, type);
      return;
  }
}

uint8_t receiveBuffer[kMaximumFrame] = {};
size_t receiveLength = 0;

void resetParser(uint8_t possibleMarker = 0) {
  receiveLength = 0;
  if (possibleMarker == kMarker0) {
    receiveBuffer[0] = kMarker0;
    receiveLength = 1;
  }
}

void feedSerialByte(uint8_t byte, uint32_t nowMs) {
  if (receiveLength == 0) {
    if (byte == kMarker0) {
      receiveBuffer[receiveLength++] = byte;
    }
    return;
  }
  if (receiveLength == 1) {
    if (byte == kMarker1) {
      receiveBuffer[receiveLength++] = byte;
    } else {
      resetParser(byte);
    }
    return;
  }
  if (receiveLength >= sizeof(receiveBuffer)) {
    ++rejectedFrames;
    resetParser(byte);
    return;
  }
  receiveBuffer[receiveLength++] = byte;

  if (receiveLength < 9) {
    return;
  }
  const uint16_t payloadLength = readU16(receiveBuffer + 7);
  if (payloadLength > kMaximumPayload) {
    ++rejectedFrames;
    resetParser(byte);
    return;
  }
  const size_t expectedLength = 11 + payloadLength;
  if (receiveLength < expectedLength) {
    return;
  }
  if (receiveLength != expectedLength || receiveBuffer[2] != kProtocolVersion) {
    ++rejectedFrames;
    resetParser(byte);
    return;
  }
  const uint16_t expectedCrc = readU16(receiveBuffer + 9 + payloadLength);
  const uint16_t actualCrc = crc16(receiveBuffer + 2, 7 + payloadLength);
  if (expectedCrc != actualCrc) {
    ++rejectedFrames;
    resetParser(byte);
    return;
  }
  const uint8_t type = receiveBuffer[3];
  const uint16_t sequence = readU16(receiveBuffer + 5);
  handleFrame(type, sequence, receiveBuffer + 9, payloadLength, nowMs);
  resetParser();
}

void serviceSerial(uint32_t nowMs) {
  while (Serial.available() > 0) {
    feedSerialByte(static_cast<uint8_t>(Serial.read()), nowMs);
  }
}

void configureMotor(MotorChannel &motor) {
  pinMode(motor.reversePin, OUTPUT);
  pinMode(motor.brakePin, OUTPUT);
  writeReverse(motor, false);
  writeThrottle(motor, 0.0f);
  writeBrake(motor, true);
}

void configureUltrasonic(UltrasonicSensor &sensor) {
  pinMode(sensor.triggerPin, OUTPUT);
  pinMode(sensor.echoPin, INPUT);
  digitalWrite(sensor.triggerPin, LOW);
}

}  // namespace

void setup() {
  configureMotor(leftMotor);
  configureMotor(rightMotor);
  for (auto &sensor : ultrasonic) {
    configureUltrasonic(sensor);
  }

  Serial.begin(kSerialBaud);
  delay(50);
  stopMotion(true);
}

void loop() {
  const uint32_t nowMs = millis();
  serviceSerial(nowMs);
  serviceUltrasonic(nowMs);
  updateMotorTargets(nowMs);
  serviceMotor(leftMotor, nowMs);
  serviceMotor(rightMotor, nowMs);

  if (nowMs - lastUltrasonicTelemetryMs >= kUltrasonicTelemetryMs) {
    lastUltrasonicTelemetryMs = nowMs;
    sendUltrasonic(telemetrySequence++, nowMs);
  }

  delay(1);
}

// SPDX-License-Identifier: GPL-3.0-only

export const VERSION = 1;
export const MAX_PAYLOAD = 48;
export const MAX_FRAME = 11 + MAX_PAYLOAD;
export const MAX_TRANSPORT_TEST_COMMAND = 250;
export const UPRIGHT_OFFSET_CONFIGURATION_KEY = 14;

function requireUnsignedWireInteger(name, value, maximum) {
  if (!Number.isInteger(value) || value < 0 || value > maximum) {
    throw new RangeError(`${name} must be an integer from 0 to ${maximum}`);
  }
}

export const MessageType = Object.freeze({
  HELLO: 0x01,
  CAPABILITIES: 0x02,
  ARM: 0x10,
  DISARM: 0x11,
  STOP: 0x12,
  EMERGENCY_STOP: 0x13,
  CLEAR_FAULT: 0x14,
  SET_OPERATING_MODE: 0x15,
  SET_LINEAR_VELOCITY: 0x20,
  SET_YAW_RATE: 0x21,
  SET_VELOCITY_AND_YAW: 0x22,
  HEARTBEAT: 0x23,
  SET_DIRECT_MOTOR: 0x24,
  STATUS: 0x30,
  IMU_TELEMETRY: 0x31,
  MOTOR_TELEMETRY: 0x32,
  ODOMETRY: 0x33,
  ACTIVE_FAULTS: 0x34,
  RESILIENCE_TELEMETRY: 0x37,
  CONFIGURATION_READ: 0x40,
  CONFIGURATION_UPDATE: 0x41,
  ACKNOWLEDGMENT: 0x7e,
  ERROR_RESPONSE: 0x7f,
});

export function crc16(bytes) {
  let crc = 0xffff;
  for (const byte of bytes) {
    crc ^= byte << 8;
    for (let bit = 0; bit < 8; bit += 1) {
      crc = (crc & 0x8000) !== 0
        ? ((crc << 1) ^ 0x1021) & 0xffff
        : (crc << 1) & 0xffff;
    }
  }
  return crc;
}

export function encodeFrame({ type, flags = 0, sequence, payload = Buffer.alloc(0) }) {
  requireUnsignedWireInteger("type", type, 0xff);
  requireUnsignedWireInteger("flags", flags, 0xff);
  requireUnsignedWireInteger("sequence", sequence, 0xffff);
  if (payload.length > MAX_PAYLOAD) {
    throw new RangeError(`payload exceeds ${MAX_PAYLOAD} bytes`);
  }
  const frame = Buffer.alloc(11 + payload.length);
  frame[0] = 0xa5;
  frame[1] = 0x5a;
  frame[2] = VERSION;
  frame[3] = type;
  frame[4] = flags;
  frame.writeUInt16LE(sequence, 5);
  frame.writeUInt16LE(payload.length, 7);
  payload.copy(frame, 9);
  frame.writeUInt16LE(crc16(frame.subarray(2, 9 + payload.length)),
    9 + payload.length);
  return frame;
}

export function encodeMovement({
  linearVelocityMilli,
  yawRateMilli,
  leaseId,
  lifetimeMs,
}) {
  for (const value of [linearVelocityMilli, yawRateMilli]) {
    if (!Number.isInteger(value) || value < -32768 || value > 32767) {
      throw new RangeError("movement values must fit signed 16-bit fields");
    }
  }
  if (!Number.isInteger(lifetimeMs) || lifetimeMs <= 0 || lifetimeMs > 65535) {
    throw new RangeError("lifetimeMs must be between 1 and 65535");
  }
  requireUnsignedWireInteger("leaseId", leaseId, 0xffffffff);
  const payload = Buffer.alloc(10);
  payload.writeInt16LE(linearVelocityMilli, 0);
  payload.writeInt16LE(yawRateMilli, 2);
  payload.writeUInt32LE(leaseId, 4);
  payload.writeUInt16LE(lifetimeMs, 8);
  return payload;
}

export function encodeDirectMotor({
  left,
  right,
  leaseId,
  lifetimeMs,
}) {
  for (const [name, value] of [["left", left], ["right", right]]) {
    if (!Number.isInteger(value) ||
        Math.abs(value) > MAX_TRANSPORT_TEST_COMMAND) {
      throw new RangeError(
        `${name} must be an integer from ` +
        `${-MAX_TRANSPORT_TEST_COMMAND} to ${MAX_TRANSPORT_TEST_COMMAND}`,
      );
    }
  }
  if (!Number.isInteger(lifetimeMs) || lifetimeMs <= 0 || lifetimeMs > 65535) {
    throw new RangeError("lifetimeMs must be between 1 and 65535");
  }
  requireUnsignedWireInteger("leaseId", leaseId, 0xffffffff);
  const payload = Buffer.alloc(10);
  payload.writeInt16LE(left, 0);
  payload.writeInt16LE(right, 2);
  payload.writeUInt32LE(leaseId, 4);
  payload.writeUInt16LE(lifetimeMs, 8);
  return payload;
}

export function encodeUprightOffset(valueDegrees) {
  if (!Number.isFinite(valueDegrees) ||
      valueDegrees < -45 || valueDegrees > 45) {
    throw new RangeError("upright offset must be from -45 to 45 degrees");
  }
  const payload = Buffer.alloc(5);
  payload[0] = UPRIGHT_OFFSET_CONFIGURATION_KEY;
  payload.writeInt32LE(Math.round(valueDegrees * 1000), 1);
  return payload;
}

const stateNames = [
  "BOOT",
  "IMU_CALIBRATING",
  "DISARMED",
  "ARMED_BALANCE",
  "DRIVING",
  "FALLEN",
  "FAULT",
];

const sourceNames = ["none", "web", "serial", "local"];

function invalidPayload(type, payload, expected) {
  return {
    name: "invalid-payload",
    type,
    expected,
    actual: payload.length,
  };
}

function decodeCapabilities(payload) {
  if (payload.length !== 12) {
    return invalidPayload(MessageType.CAPABILITIES, payload, 12);
  }
  return {
    name: "capabilities",
    protocolVersion: payload[0],
    dryRun: payload[1] !== 0,
    webEnabled: payload[2] !== 0,
    supportedModes: payload[3],
    controlRateHz: payload.readUInt16LE(4),
    motorRateHz: payload.readUInt16LE(6),
    maximumPayload: payload.readUInt16LE(8),
    configurationKeys: payload.readUInt16LE(10),
  };
}

function decodeStatus(payload) {
  if (payload.length !== 16) {
    return invalidPayload(MessageType.STATUS, payload, 16);
  }
  return {
    name: "status",
    state: stateNames[payload[0]] ?? `unknown-${payload[0]}`,
    operatingMode: payload[1],
    activeSource: sourceNames[payload[2]] ?? `unknown-${payload[2]}`,
    healthFlags: payload[3],
    faults: payload.readUInt32LE(4),
    loopOverruns: payload.readUInt32LE(8),
    rejectedSerialFrames: payload.readUInt32LE(12),
  };
}

function decodeImu(payload) {
  if (payload.length !== 44) {
    return invalidPayload(MessageType.IMU_TELEMETRY, payload, 44);
  }
  const calibrated = payload[1] !== 0;
  const gyroscopeRawDps = [10, 12, 14].map(offset =>
    payload.readInt16LE(offset) / 100);
  const gyroBiasDps = [38, 40, 42].map(offset =>
    payload.readInt16LE(offset) / 100);
  return {
    name: "imu",
    address: payload[0],
    calibrated,
    valid: payload[2] !== 0,
    accelerationG: [4, 6, 8].map(offset => payload.readInt16LE(offset) / 1000),
    gyroscopeRawDps,
    rawPitchDeg: payload.readInt16LE(16) / 100,
    filteredPitchDeg: payload.readInt16LE(18) / 100,
    pitchRateDps: payload.readInt16LE(20) / 100,
    sampleRateHz: payload.readUInt16LE(22) / 100,
    i2cErrors: payload.readUInt32LE(24),
    missedSamples: payload.readUInt32LE(28),
    sampleAgeUs: payload.readUInt32LE(32),
    calibrationSamples: payload.readUInt16LE(36),
    gyroBiasDps,
    gyroscopeCorrectedDps: gyroscopeRawDps.map((value, index) =>
      calibrated ? value - gyroBiasDps[index] : value),
  };
}

function decodeMotor(payload) {
  if (payload.length !== 46) {
    return invalidPayload(MessageType.MOTOR_TELEMETRY, payload, 46);
  }
  return {
    name: "motor",
    calculated: [payload.readInt16LE(0), payload.readInt16LE(2)],
    applied: [payload.readInt16LE(4), payload.readInt16LE(6)],
    sequence: payload.readUInt16LE(8),
    flags: payload.readUInt16LE(10),
    transmittedFrames: payload.readUInt32LE(12),
    feedbackFrames: payload.readUInt32LE(16),
    crcErrors: payload.readUInt32LE(20),
    acknowledgmentTimeouts: payload.readUInt32LE(24),
    lastAckLatencyUs: payload.readUInt32LE(28),
    maximumAckLatencyUs: payload.readUInt32LE(32),
    lastApplyLatencyUs: payload.readUInt32LE(36),
    maximumApplyLatencyUs: payload.readUInt32LE(40),
    transmitRateHz: payload.readUInt16LE(44) / 100,
  };
}

function decodeOdometry(payload) {
  if (payload.length !== 20) {
    return invalidPayload(MessageType.ODOMETRY, payload, 20);
  }
  return {
    name: "odometry",
    left: payload.readInt32LE(0),
    right: payload.readInt32LE(4),
    velocity: payload.readInt32LE(8) / 1000,
    timestampUs: Number(payload.readBigUInt64LE(12)),
  };
}

function decodeFaults(payload) {
  if (payload.length !== 16) {
    return invalidPayload(MessageType.ACTIVE_FAULTS, payload, 16);
  }
  return {
    name: "faults",
    balance: payload.readUInt32LE(0),
    master: payload.readUInt32LE(4),
    slave: payload.readUInt32LE(8),
    feedbackHealth: payload.readUInt32LE(12),
  };
}

function decodeResilience(payload) {
  if (payload.length !== 48) {
    return invalidPayload(MessageType.RESILIENCE_TELEMETRY, payload, 48);
  }
  return {
    name: "resilience",
    warningFlags: payload.readUInt16LE(0),
    feedbackCrc: {
      streak: payload[2],
      threshold: payload[3],
      total: payload.readUInt32LE(4),
    },
    hallGlitches: [payload.readUInt16LE(8), payload.readUInt16LE(10)],
    interControllerLink: {
      slaveFeedbackInvalid: payload.readUInt16LE(12),
      slaveFeedbackFraming: payload.readUInt16LE(14),
      slaveCommandInvalid: payload.readUInt16LE(16),
      slaveCommandFraming: payload.readUInt16LE(18),
    },
    firstFault: {
      drive: payload.readUInt32LE(20),
      master: payload.readUInt32LE(24),
      slave: payload.readUInt32LE(28),
      states: [payload[32], payload[33]],
      halls: [payload[34], payload[35]],
      commanded: [payload.readInt16LE(36), payload.readInt16LE(38)],
      applied: [payload.readInt16LE(40), payload.readInt16LE(42)],
      esp32UptimeMs: payload.readUInt32LE(44),
    },
  };
}

function decodeConfiguration(payload) {
  if (payload.length !== 5) {
    return invalidPayload(MessageType.CONFIGURATION_READ, payload, 5);
  }
  return {
    name: "configuration",
    key: payload[0],
    value: payload.readInt32LE(1) / 1000,
  };
}

function decodeAcknowledgment(payload) {
  if (payload.length !== 2) {
    return invalidPayload(MessageType.ACKNOWLEDGMENT, payload, 2);
  }
  return {
    name: "acknowledgment",
    requestType: payload[0],
    status: payload[1],
  };
}

function decodeError(payload) {
  if (payload.length !== 4) {
    return invalidPayload(MessageType.ERROR_RESPONSE, payload, 4);
  }
  return {
    name: "error",
    requestType: payload[0],
    errorCode: payload[1],
    detail: payload.readUInt16LE(2),
  };
}

export function decodeMessage(frame) {
  const payload = frame.payload ?? Buffer.alloc(0);
  switch (frame.type) {
    case MessageType.CAPABILITIES:
      return decodeCapabilities(payload);
    case MessageType.STATUS:
      return decodeStatus(payload);
    case MessageType.IMU_TELEMETRY:
      return decodeImu(payload);
    case MessageType.MOTOR_TELEMETRY:
      return decodeMotor(payload);
    case MessageType.ODOMETRY:
      return decodeOdometry(payload);
    case MessageType.ACTIVE_FAULTS:
      return decodeFaults(payload);
    case MessageType.RESILIENCE_TELEMETRY:
      return decodeResilience(payload);
    case MessageType.CONFIGURATION_READ:
      return decodeConfiguration(payload);
    case MessageType.ACKNOWLEDGMENT:
      return decodeAcknowledgment(payload);
    case MessageType.ERROR_RESPONSE:
      return decodeError(payload);
    default:
      return {
        name: "unknown",
        type: frame.type,
        payloadHex: payload.toString("hex"),
      };
  }
}

export class FrameParser {
  #buffer = Buffer.alloc(0);

  get bufferedBytes() {
    return this.#buffer.length;
  }

  feed(chunk) {
    this.#buffer = Buffer.concat([this.#buffer, chunk]);
    const results = [];
    while (this.#buffer.length >= 2) {
      const marker = this.#buffer.indexOf(Buffer.from([0xa5, 0x5a]));
      if (marker < 0) {
        this.#buffer = this.#buffer.subarray(
          this.#buffer.at(-1) === 0xa5 ? -1 : this.#buffer.length,
        );
        break;
      }
      this.#buffer = this.#buffer.subarray(marker);
      if (this.#buffer.length < 9) break;
      const payloadLength = this.#buffer.readUInt16LE(7);
      if (payloadLength > MAX_PAYLOAD) {
        results.push({ error: "malformed-length" });
        this.#buffer = this.#buffer.subarray(2);
        continue;
      }
      const frameLength = 11 + payloadLength;
      if (this.#buffer.length < frameLength) break;
      const candidate = this.#buffer.subarray(0, frameLength);
      this.#buffer = this.#buffer.subarray(frameLength);
      const expected = crc16(candidate.subarray(2, 9 + payloadLength));
      const actual = candidate.readUInt16LE(9 + payloadLength);
      if (actual !== expected) {
        results.push({ error: "invalid-crc" });
        continue;
      }
      if (candidate[2] !== VERSION) {
        results.push({ error: "unsupported-version" });
        continue;
      }
      results.push({
        version: candidate[2],
        type: candidate[3],
        flags: candidate[4],
        sequence: candidate.readUInt16LE(5),
        payload: Buffer.from(candidate.subarray(9, 9 + payloadLength)),
      });
    }
    return results;
  }
}

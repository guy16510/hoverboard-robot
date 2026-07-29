// SPDX-License-Identifier: GPL-3.0-only

export const DRIVE_MODE = 2;
export const DRIVE_TELEMETRY = 0x35;
export const OUTPUT_LIMIT = 250;

export function decodeDriveTelemetry(payload) {
  if (!Buffer.isBuffer(payload) || payload.length !== 24) {
    return {
      name: "invalid-payload",
      type: DRIVE_TELEMETRY,
      expected: 24,
      actual: payload?.length ?? 0,
    };
  }
  return {
    name: "drive",
    requestedLinear: payload.readInt16LE(0) / 1000,
    requestedYaw: payload.readInt16LE(2) / 1000,
    mixed: [payload.readInt16LE(4), payload.readInt16LE(6)],
    commanded: [payload.readInt16LE(8), payload.readInt16LE(10)],
    applied: [payload.readInt16LE(12), payload.readInt16LE(14)],
    safetyFaults: payload.readUInt32LE(16),
    activeSource: payload[20],
    operatingMode: payload[21],
    armed: payload[22] !== 0,
    flags: payload[23],
  };
}

export function assertBoundedDriveTelemetry(value) {
  for (const field of ["mixed", "commanded", "applied"]) {
    for (const output of value[field]) {
      if (!Number.isInteger(output) || Math.abs(output) > OUTPUT_LIMIT) {
        throw new RangeError(`${field} output exceeds validated ${OUTPUT_LIMIT} ceiling`);
      }
    }
  }
  return value;
}

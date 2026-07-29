// SPDX-License-Identifier: GPL-3.0-only

import assert from "node:assert/strict";
import test from "node:test";

import { SerialClient } from "../client.mjs";
import {
  DRIVE_MODE,
  DRIVE_TELEMETRY,
  OUTPUT_LIMIT,
  assertBoundedDriveTelemetry,
  decodeDriveTelemetry,
} from "../manual_drive.mjs";
import { FrameParser, MessageType } from "../protocol.mjs";

function captureClient() {
  const writes = [];
  let closed = false;
  const client = new SerialClient("/dev/simulated", {
    configure: false,
    leaseId: 0x12345678,
    openPort: () => 42,
    write: (_descriptor, frame) => writes.push(Buffer.from(frame)),
    close: () => { closed = true; },
  });
  return { client, writes, isClosed: () => closed };
}

function decodeWrites(writes) {
  const parser = new FrameParser();
  return writes.flatMap(frame => parser.feed(frame));
}

test("Node client initializes manual drive with mode 2, zero, then ARM", () => {
  const { client, writes } = captureClient();
  const sequences = client.initializeDrive();
  const frames = decodeWrites(writes);

  assert.deepEqual(frames.map(frame => frame.type), [
    MessageType.HELLO,
    MessageType.SET_OPERATING_MODE,
    MessageType.SET_VELOCITY_AND_YAW,
    MessageType.ARM,
  ]);
  assert.equal(frames[1].payload[0], DRIVE_MODE);
  assert.equal(frames[2].payload.readInt16LE(0), 0);
  assert.equal(frames[2].payload.readInt16LE(2), 0);
  assert.deepEqual(sequences, { modeSequence: 1, zeroSequence: 2, armSequence: 3 });
});

test("Node safeClose independently attempts zero, STOP, and DISARM", () => {
  const { client, writes, isClosed } = captureClient();
  client.initializeDrive();
  client.safeClose();
  const frames = decodeWrites(writes);
  assert.deepEqual(frames.slice(-3).map(frame => frame.type), [
    MessageType.SET_VELOCITY_AND_YAW,
    MessageType.STOP,
    MessageType.DISARM,
  ]);
  assert.equal(frames.at(-3).payload.readInt16LE(0), 0);
  assert.equal(frames.at(-3).payload.readInt16LE(2), 0);
  assert.equal(isClosed(), true);
});

test("drive telemetry binary layout matches Python and ESP32 contract", () => {
  const payload = Buffer.alloc(24);
  payload.writeInt16LE(175, 0);
  payload.writeInt16LE(-200, 2);
  payload.writeInt16LE(250, 4);
  payload.writeInt16LE(-25, 6);
  payload.writeInt16LE(100, 8);
  payload.writeInt16LE(-25, 10);
  payload.writeInt16LE(98, 12);
  payload.writeInt16LE(-24, 14);
  payload.writeUInt32LE(0, 16);
  payload[20] = 2;
  payload[21] = DRIVE_MODE;
  payload[22] = 1;
  payload[23] = 0x0f;

  const decoded = decodeDriveTelemetry(payload);
  assert.deepEqual(decoded, {
    name: "drive",
    requestedLinear: 0.175,
    requestedYaw: -0.2,
    mixed: [250, -25],
    commanded: [100, -25],
    applied: [98, -24],
    safetyFaults: 0,
    activeSource: 2,
    operatingMode: 2,
    armed: true,
    flags: 0x0f,
  });
  assert.equal(DRIVE_TELEMETRY, 0x35);
  assert.equal(assertBoundedDriveTelemetry(decoded), decoded);
});

test("drive telemetry rejects outputs above validated ceiling", () => {
  assert.equal(OUTPUT_LIMIT, 250);
  assert.throws(() => assertBoundedDriveTelemetry({
    mixed: [251, 0],
    commanded: [0, 0],
    applied: [0, 0],
  }), RangeError);
});

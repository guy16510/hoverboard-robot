// SPDX-License-Identifier: GPL-3.0-only

import assert from "node:assert/strict";
import test from "node:test";

import {
  FrameParser,
  MessageType,
  crc16,
  decodeMessage,
  encodeDirectMotor,
  encodeFrame,
  encodeMovement,
} from "../protocol.mjs";
import {
  PollingSerialStream,
  Sequence,
  openConfiguredSerialPort,
} from "../client.mjs";
import { MixedTelemetryDecoder } from "../capture_decoder.mjs";
import { CaptureLifecycle } from "../capture_lifecycle.mjs";
import { normalizeCaptureEvent } from "../evidence.mjs";
import { normalizeStagePlan } from "../stage_plan.mjs";

test("CRC-16/CCITT-FALSE matches the standard check vector", () => {
  assert.equal(crc16(Buffer.from("123456789")), 0x29b1);
});

test("frame fields use deterministic little-endian encoding", () => {
  const frame = encodeFrame({
    type: MessageType.SET_VELOCITY_AND_YAW,
    flags: 3,
    sequence: 0x1234,
    payload: Buffer.from([0x78, 0x56, 0x34, 0x12]),
  });
  assert.deepEqual([...frame.subarray(0, 9)],
    [0xa5, 0x5a, 1, 0x22, 3, 0x34, 0x12, 4, 0]);
});

test("frame encoder rejects fields that do not fit their wire widths", () => {
  assert.throws(() => encodeFrame({ type: -1, sequence: 0 }), RangeError);
  assert.throws(() => encodeFrame({ type: 256, sequence: 0 }), RangeError);
  assert.throws(
    () => encodeFrame({ type: MessageType.STOP, flags: 256, sequence: 0 }),
    RangeError,
  );
  assert.throws(
    () => encodeFrame({ type: MessageType.STOP, sequence: 65536 }),
    RangeError,
  );
});

test("parser accepts fragmented frames and resynchronizes past noise", () => {
  const frame = encodeFrame({
    type: MessageType.ARM,
    sequence: 9,
  });
  const parser = new FrameParser();
  assert.deepEqual(parser.feed(Buffer.from([1, 2, 3, frame[0]])), []);
  const results = parser.feed(frame.subarray(1));
  assert.equal(results.length, 1);
  assert.equal(results[0].type, MessageType.ARM);
  assert.equal(results[0].sequence, 9);
});

test("parser rejects a corrupted CRC", () => {
  const frame = encodeFrame({
    type: MessageType.STOP,
    sequence: 1,
  });
  frame[frame.length - 1] ^= 1;
  assert.deepEqual(new FrameParser().feed(frame), [{ error: "invalid-crc" }]);
});

test("movement payload is signed, little-endian, and lease bounded", () => {
  const payload = encodeMovement({
    linearVelocityMilli: -250,
    yawRateMilli: 125,
    leaseId: 0x12345678,
    lifetimeMs: 500,
  });
  assert.equal(payload.readInt16LE(0), -250);
  assert.equal(payload.readInt16LE(2), 125);
  assert.equal(payload.readUInt32LE(4), 0x12345678);
  assert.equal(payload.readUInt16LE(8), 500);
});

test("movement encoder rejects invalid lease identifiers", () => {
  const movement = {
    linearVelocityMilli: 0,
    yawRateMilli: 0,
    lifetimeMs: 500,
  };
  assert.throws(() => encodeMovement({ ...movement, leaseId: -1 }), RangeError);
  assert.throws(
    () => encodeMovement({ ...movement, leaseId: 0x1_0000_0000 }),
    RangeError,
  );
  assert.throws(
    () => encodeMovement({ ...movement, leaseId: 1.5 }),
    RangeError,
  );
});

test("direct motor encoder enforces the transport-test command ceiling", () => {
  const payload = encodeDirectMotor({
    left: 20,
    right: -30,
    leaseId: 0x12345678,
    lifetimeMs: 400,
  });
  assert.equal(payload.readInt16LE(0), 20);
  assert.equal(payload.readInt16LE(2), -30);
  assert.equal(payload.readUInt32LE(4), 0x12345678);
  assert.equal(payload.readUInt16LE(8), 400);
  assert.throws(
    () => encodeDirectMotor({
      left: 51,
      right: 0,
      leaseId: 1,
      lifetimeMs: 500,
    }),
    RangeError,
  );
});

test("sequence generation rolls over from 65535 to zero", () => {
  const sequence = new Sequence(0xffff);
  assert.equal(sequence.next(), 0xffff);
  assert.equal(sequence.next(), 0);
});

test("serial descriptor opens before macOS baud configuration", () => {
  const calls = [];
  const descriptor = openConfiguredSerialPort("/dev/test", {
    baud: 115200,
    open: () => {
      calls.push("open");
      return 42;
    },
    configurePort: () => calls.push("configure"),
  });
  assert.equal(descriptor, 42);
  assert.deepEqual(calls, ["open", "configure"]);
});

test("nonblocking serial stream retries EAGAIN and closes cleanly", async () => {
  let calls = 0;
  const stream = new PollingSerialStream(42, {
    intervalMs: 1,
    read: (_descriptor, buffer) => {
      calls += 1;
      if (calls === 1) {
        const error = new Error("would block");
        error.code = "EAGAIN";
        throw error;
      }
      Buffer.from("ok").copy(buffer);
      return 2;
    },
  });
  const data = await new Promise((resolve, reject) => {
    stream.once("data", resolve);
    stream.once("error", reject);
  });
  assert.equal(data.toString(), "ok");
  stream.destroy();
  await new Promise(resolve => stream.once("close", resolve));
});

test("capabilities and status payloads decode into named fields", () => {
  const capabilities = Buffer.from([
    1, 1, 0, 7, 0xc8, 0, 10, 0, 48, 0, 17, 0,
  ]);
  assert.deepEqual(
    decodeMessage({ type: MessageType.CAPABILITIES, payload: capabilities }),
    {
      name: "capabilities",
      protocolVersion: 1,
      dryRun: true,
      webEnabled: false,
      supportedModes: 7,
      controlRateHz: 200,
      motorRateHz: 10,
      maximumPayload: 48,
      configurationKeys: 17,
    },
  );

  const status = Buffer.alloc(16);
  status[0] = 3;
  status[1] = 1;
  status[2] = 2;
  status[3] = 0x6f;
  status.writeUInt32LE(0x20, 4);
  status.writeUInt32LE(4, 8);
  status.writeUInt32LE(2, 12);
  assert.deepEqual(
    decodeMessage({ type: MessageType.STATUS, payload: status }),
    {
      name: "status",
      state: "ARMED_BALANCE",
      operatingMode: 1,
      activeSource: "serial",
      healthFlags: 0x6f,
      faults: 0x20,
      loopOverruns: 4,
      rejectedSerialFrames: 2,
    },
  );
});

test("parser bounds discarded noise while waiting for a marker", () => {
  const parser = new FrameParser();
  parser.feed(Buffer.alloc(4096, 0x55));
  assert.equal(parser.bufferedBytes, 0);
  parser.feed(Buffer.from([0xa5]));
  assert.equal(parser.bufferedBytes, 1);
});

test("IMU, motor, odometry, fault, and configuration payloads decode", () => {
  const imu = Buffer.alloc(44);
  imu[0] = 0x69;
  imu[1] = 1;
  imu[2] = 1;
  imu.writeInt16LE(1000, 4);
  imu.writeInt16LE(-250, 10);
  imu.writeInt16LE(1234, 18);
  imu.writeUInt16LE(20000, 22);
  imu.writeUInt32LE(3, 24);
  imu.writeUInt16LE(400, 36);
  imu.writeInt16LE(125, 38);
  imu.writeInt16LE(-250, 40);
  imu.writeInt16LE(50, 42);
  assert.deepEqual(
    decodeMessage({ type: MessageType.IMU_TELEMETRY, payload: imu }),
    {
      name: "imu",
      address: 0x69,
      calibrated: true,
      valid: true,
      accelerationG: [1, 0, 0],
      gyroscopeRawDps: [-2.5, 0, 0],
      rawPitchDeg: 0,
      filteredPitchDeg: 12.34,
      pitchRateDps: 0,
      sampleRateHz: 200,
      i2cErrors: 3,
      missedSamples: 0,
      sampleAgeUs: 0,
      calibrationSamples: 400,
      gyroBiasDps: [1.25, -2.5, 0.5],
      gyroscopeCorrectedDps: [-3.75, 2.5, -0.5],
    },
  );
  imu[1] = 0;
  assert.deepEqual(
    decodeMessage({ type: MessageType.IMU_TELEMETRY, payload: imu })
      .gyroscopeCorrectedDps,
    [-2.5, 0, 0],
  );

  const motor = Buffer.alloc(46);
  motor.writeInt16LE(20, 0);
  motor.writeInt16LE(-20, 6);
  motor.writeUInt32LE(3000, 40);
  motor.writeUInt16LE(1000, 44);
  const decodedMotor =
    decodeMessage({ type: MessageType.MOTOR_TELEMETRY, payload: motor });
  assert.deepEqual(decodedMotor.calculated, [20, 0]);
  assert.deepEqual(decodedMotor.applied, [0, -20]);
  assert.equal(decodedMotor.maximumApplyLatencyUs, 3000);
  assert.equal(decodedMotor.transmitRateHz, 10);

  const odometry = Buffer.alloc(20);
  odometry.writeInt32LE(-4, 0);
  odometry.writeInt32LE(7, 4);
  odometry.writeInt32LE(1250, 8);
  odometry.writeBigUInt64LE(123456n, 12);
  assert.deepEqual(
    decodeMessage({ type: MessageType.ODOMETRY, payload: odometry }),
    {
      name: "odometry",
      left: -4,
      right: 7,
      velocity: 1.25,
      timestampUs: 123456,
    },
  );

  const faults = Buffer.alloc(16);
  faults.writeUInt32LE(2, 0);
  faults.writeUInt32LE(3, 4);
  faults.writeUInt32LE(4, 8);
  faults.writeUInt32LE(5, 12);
  assert.deepEqual(
    decodeMessage({ type: MessageType.ACTIVE_FAULTS, payload: faults }),
    {
      name: "faults",
      balance: 2,
      master: 3,
      slave: 4,
      feedbackHealth: 5,
    },
  );

  const configuration = Buffer.alloc(5);
  configuration[0] = 14;
  configuration.writeInt32LE(-2500, 1);
  assert.deepEqual(
    decodeMessage({
      type: MessageType.CONFIGURATION_READ,
      payload: configuration,
    }),
    { name: "configuration", key: 14, value: -2.5 },
  );
});

test("mixed telemetry decoder preserves text and fragmented binary frames", () => {
  const frame = encodeFrame({
    type: MessageType.STATUS,
    sequence: 42,
    payload: Buffer.alloc(16),
  });
  const balance = 'BALANCE {"dry_run":1,"state":"DISARMED"}\r\n';
  const bytes = Buffer.concat([
    Buffer.from("boot banner\n"),
    frame,
    Buffer.from(balance),
  ]);
  const decoder = new MixedTelemetryDecoder();
  const events = [];
  for (let offset = 0; offset < bytes.length; offset += 3) {
    events.push(...decoder.feed(bytes.subarray(offset, offset + 3)));
  }

  assert.deepEqual(events[0], { kind: "text", line: "boot banner" });
  assert.equal(events[1].kind, "frame");
  assert.equal(events[1].frame.sequence, 42);
  assert.deepEqual(events[2], {
    kind: "text",
    line: 'BALANCE {"dry_run":1,"state":"DISARMED"}',
  });
  assert.equal(decoder.bufferedBytes, 0);
});

test("mixed telemetry decoder reports corruption and bounds unterminated text", () => {
  const frame = encodeFrame({
    type: MessageType.STOP,
    sequence: 4,
  });
  frame[frame.length - 1] ^= 1;
  const decoder = new MixedTelemetryDecoder({ maximumTextBytes: 64 });

  assert.deepEqual(decoder.feed(frame), [
    { kind: "protocol-error", error: "invalid-crc" },
  ]);
  assert.deepEqual(decoder.feed(Buffer.alloc(68, 0x41)), [
    { kind: "text-overflow", discardedBytes: 4 },
  ]);
  assert.equal(decoder.bufferedBytes, 64);
});

test("capture normalization preserves decoded binary and BALANCE telemetry", () => {
  const balance = normalizeCaptureEvent({
    kind: "text",
    line: 'BALANCE {"dry_run":1,"state":"DISARMED","pitch_filtered":2.5}',
  });
  assert.deepEqual(balance, {
    kind: "balance-telemetry",
    telemetry: {
      dry_run: 1,
      state: "DISARMED",
      pitch_filtered: 2.5,
    },
  });

  const frame = normalizeCaptureEvent({
    kind: "frame",
    frame: {
      version: 1,
      type: MessageType.STATUS,
      flags: 0,
      sequence: 8,
      payload: Buffer.alloc(16),
    },
  });
  assert.equal(frame.kind, "binary-frame");
  assert.equal(frame.sequence, 8);
  assert.equal(frame.payloadHex, "00000000000000000000000000000000");
  assert.equal(frame.decoded.name, "status");
});

test("capture normalization retains malformed device text for later review", () => {
  assert.deepEqual(
    normalizeCaptureEvent({ kind: "text", line: "BALANCE {not-json}" }),
    {
      kind: "device-text",
      line: "BALANCE {not-json}",
      parseError: "invalid BALANCE JSON",
    },
  );
});

test("motor transport plans are bounded, ordered, and fail-safe", () => {
  const plan = normalizeStagePlan({
    actions: [
      { atMs: 1000, command: "mode", value: 3 },
      { atMs: 1500, command: "direct", left: 0, right: 0 },
      { atMs: 2000, command: "arm" },
      { atMs: 2500, command: "direct", left: 10, right: 0 },
      { atMs: 3500, command: "direct", left: 0, right: 0 },
      { atMs: 4000, command: "stop" },
      { atMs: 4500, command: "disarm" },
    ],
  }, { stage: "motor-transport", durationSeconds: 10 });
  assert.equal(plan.sendsArm, true);
  assert.equal(plan.sendsMovement, true);
  assert.equal(plan.maximumAbsoluteCommand, 10);
  assert.throws(() => normalizeStagePlan({
    actions: [
      { atMs: 1000, command: "mode", value: 3 },
      { atMs: 1500, command: "arm" },
      { atMs: 2000, command: "direct", left: 51, right: 0 },
      { atMs: 3000, command: "direct", left: 0, right: 0 },
      { atMs: 3500, command: "stop" },
      { atMs: 4000, command: "disarm" },
    ],
  }, { stage: "motor-transport", durationSeconds: 10 }), /left/);
});

test("lifted-wheel plans reject direct motion and require fail-safe ending", () => {
  const actions = [
    { atMs: 1000, command: "mode", value: 1 },
    { atMs: 2000, command: "arm" },
    { atMs: 6000, command: "direct", left: 0, right: 0 },
    { atMs: 6500, command: "stop" },
    { atMs: 7000, command: "disarm" },
  ];
  assert.equal(normalizeStagePlan(
    { actions }, { stage: "lifted-wheel", durationSeconds: 10 },
  ).sendsMovement, false);
  assert.throws(() => normalizeStagePlan(
    { actions: actions.slice(0, -1) },
    { stage: "lifted-wheel", durationSeconds: 10 },
  ), /end with direct zero/);
});

test("capture lifecycle ignores late serial events after finalization", () => {
  const lifecycle = new CaptureLifecycle();
  assert.equal(lifecycle.canHandleSerialEvent, true);
  assert.equal(lifecycle.beginFinish(), true);
  assert.equal(lifecycle.beginFinish(), false);
  lifecycle.markFinalized();
  assert.equal(lifecycle.canRecord, false);
  assert.equal(lifecycle.canHandleSerialEvent, false);
});

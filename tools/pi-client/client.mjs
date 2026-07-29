// SPDX-License-Identifier: GPL-3.0-only

import fs from "node:fs";
import { spawnSync } from "node:child_process";
import { Readable } from "node:stream";

import {
  MessageType,
  encodeDirectMotor,
  encodeFrame,
  encodeMovement,
} from "./protocol.mjs";
import { DRIVE_MODE } from "./manual_drive.mjs";

export class Sequence {
  #value;

  constructor(initial = 0) {
    this.#value = initial & 0xffff;
  }

  next() {
    const current = this.#value;
    this.#value = (this.#value + 1) & 0xffff;
    return current;
  }
}

export function configureSerialPort(port, baud = 115200) {
  const selector = process.platform === "darwin" ? "-f" : "-F";
  const result = spawnSync(
    "stty",
    [selector, port, String(baud), "raw", "-echo", "-hupcl"],
    { encoding: "utf8" });
  if (result.status !== 0) {
    throw new Error(`stty failed: ${result.stderr.trim()}`);
  }
}

export function openConfiguredSerialPort(port, {
  baud = 115200,
  configure = true,
  open = value => fs.openSync(
    value, fs.constants.O_RDWR | fs.constants.O_NOCTTY |
      fs.constants.O_NONBLOCK),
  configurePort = configureSerialPort,
  close = fs.closeSync,
} = {}) {
  const descriptor = open(port);
  try {
    if (configure) configurePort(port, baud);
  } catch (error) {
    close(descriptor);
    throw error;
  }
  return descriptor;
}

export class PollingSerialStream extends Readable {
  #buffer = Buffer.alloc(4096);
  #descriptor;
  #read;
  #timer;

  constructor(descriptor, {
    read = fs.readSync,
    intervalMs = 5,
  } = {}) {
    super();
    this.#descriptor = descriptor;
    this.#read = read;
    this.#timer = setInterval(() => this.#poll(), intervalMs);
  }

  _read() {}

  _destroy(error, callback) {
    clearInterval(this.#timer);
    callback(error);
  }

  #poll() {
    try {
      const length = this.#read(
        this.#descriptor, this.#buffer, 0, this.#buffer.length, null);
      if (length > 0) {
        this.push(Buffer.from(this.#buffer.subarray(0, length)));
      }
    } catch (error) {
      if (error.code !== "EAGAIN" && error.code !== "EWOULDBLOCK") {
        this.destroy(error);
      }
    }
  }
}

export class SerialClient {
  #descriptor;
  #leaseId;
  #sequence;
  #write;
  #close;
  #closed = false;

  constructor(port, {
    baud = 115200,
    configure = true,
    hello = true,
    leaseId = Date.now(),
    openPort = openConfiguredSerialPort,
    write = fs.writeSync,
    close = fs.closeSync,
  } = {}) {
    this.#descriptor = openPort(port, { baud, configure });
    this.#leaseId = leaseId >>> 0;
    this.#sequence = new Sequence();
    this.#write = write;
    this.#close = close;
    if (hello) this.send(MessageType.HELLO);
  }

  send(type, payload = Buffer.alloc(0), flags = 0) {
    return this.sendDetailed(type, payload, flags).sequence;
  }

  sendDetailed(type, payload = Buffer.alloc(0), flags = 0) {
    if (this.#closed) throw new Error("serial client is closed");
    const frame = encodeFrame({
      type,
      flags,
      sequence: this.#sequence.next(),
      payload,
    });
    this.#write(this.#descriptor, frame);
    return {
      sequence: frame.readUInt16LE(5),
      frame,
    };
  }

  movement(linear, yaw, { lifetimeMs = 500, leaseId = this.#leaseId } = {}) {
    const payload = encodeMovement({
      linearVelocityMilli: Math.round(linear * 1000),
      yawRateMilli: Math.round(yaw * 1000),
      leaseId,
      lifetimeMs,
    });
    return this.send(MessageType.SET_VELOCITY_AND_YAW, payload);
  }

  directMotor(left, right,
              { lifetimeMs = 500, leaseId = this.#leaseId } = {}) {
    const payload = encodeDirectMotor({ left, right, leaseId, lifetimeMs });
    return this.send(MessageType.SET_DIRECT_MOTOR, payload);
  }

  mode(value) {
    return this.send(MessageType.SET_OPERATING_MODE, Buffer.from([value]));
  }

  initializeDrive() {
    const modeSequence = this.mode(DRIVE_MODE);
    const zeroSequence = this.movement(0, 0);
    const armSequence = this.send(MessageType.ARM);
    return { modeSequence, zeroSequence, armSequence };
  }

  requestTelemetry(type) {
    return this.send(type);
  }

  readStream() {
    if (this.#closed) throw new Error("serial client is closed");
    return new PollingSerialStream(this.#descriptor);
  }

  safeClose() {
    if (this.#closed) return;
    for (const operation of [
      () => this.movement(0, 0),
      () => this.send(MessageType.STOP),
      () => this.send(MessageType.DISARM),
    ]) {
      try {
        operation();
      } catch {
        // Shutdown is best effort, every operation is attempted independently.
      }
    }
    this.close();
  }

  close() {
    if (this.#closed) return;
    this.#closed = true;
    this.#close(this.#descriptor);
  }
}

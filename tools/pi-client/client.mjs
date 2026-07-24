// SPDX-License-Identifier: GPL-3.0-only

import fs from "node:fs";
import { spawnSync } from "node:child_process";

import {
  MessageType,
  encodeFrame,
  encodeMovement,
} from "./protocol.mjs";

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
  const result = spawnSync("stty", [selector, port, String(baud), "raw", "-echo"],
    { encoding: "utf8" });
  if (result.status !== 0) {
    throw new Error(`stty failed: ${result.stderr.trim()}`);
  }
}

export class SerialClient {
  #descriptor;
  #leaseId;
  #sequence;

  constructor(port, {
    baud = 115200,
    configure = true,
    hello = true,
    leaseId = Date.now(),
  } = {}) {
    if (configure) configureSerialPort(port, baud);
    this.#descriptor = fs.openSync(port, fs.constants.O_RDWR | fs.constants.O_NOCTTY);
    this.#leaseId = leaseId >>> 0;
    this.#sequence = new Sequence();
    if (hello) this.send(MessageType.HELLO);
  }

  send(type, payload = Buffer.alloc(0), flags = 0) {
    const frame = encodeFrame({
      type,
      flags,
      sequence: this.#sequence.next(),
      payload,
    });
    fs.writeSync(this.#descriptor, frame);
    return frame.readUInt16LE(5);
  }

  movement(linear, yaw, { lifetimeMs = 500, leaseId = this.#leaseId } = {}) {
    const payload = encodeMovement({
      linearVelocityMilli: Math.round(linear * 1000),
      yawRateMilli: Math.round(yaw * 1000),
      leaseId,
      lifetimeMs,
    });
    this.send(MessageType.SET_VELOCITY_AND_YAW, payload);
  }

  mode(value) {
    this.send(MessageType.SET_OPERATING_MODE, Buffer.from([value]));
  }

  readStream() {
    return fs.createReadStream(null, {
      fd: this.#descriptor,
      autoClose: false,
    });
  }

  close() {
    fs.closeSync(this.#descriptor);
  }
}

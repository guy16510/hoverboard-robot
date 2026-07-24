// SPDX-License-Identifier: GPL-3.0-only

import { FrameParser, MAX_FRAME, MAX_PAYLOAD } from "./protocol.mjs";

const marker = Buffer.from([0xa5, 0x5a]);
const headerBytes = 9;

function textLine(bytes) {
  const end = bytes.at(-1) === 0x0d ? bytes.length - 1 : bytes.length;
  return bytes.subarray(0, end).toString("utf8");
}

function decodeFrame(candidate) {
  const [result] = new FrameParser().feed(candidate);
  if (result?.error) {
    return { kind: "protocol-error", error: result.error };
  }
  return { kind: "frame", frame: result };
}

export class MixedTelemetryDecoder {
  #buffer = Buffer.alloc(0);
  #maximumTextBytes;

  constructor({ maximumTextBytes = 8192 } = {}) {
    if (!Number.isInteger(maximumTextBytes) || maximumTextBytes < MAX_FRAME) {
      throw new RangeError(`maximumTextBytes must be at least ${MAX_FRAME}`);
    }
    this.#maximumTextBytes = maximumTextBytes;
  }

  get bufferedBytes() {
    return this.#buffer.length;
  }

  feed(chunk) {
    if (!Buffer.isBuffer(chunk)) {
      throw new TypeError("serial chunk must be a Buffer");
    }
    this.#buffer = Buffer.concat([this.#buffer, chunk]);
    const events = [];
    while (this.#consumeOne(events)) {
      // Consume every complete line or frame currently available.
    }
    this.#boundText(events);
    return events;
  }

  #consumeOne(events) {
    if (this.#startsWithMarker()) {
      return this.#consumeFrame(events);
    }
    const markerIndex = this.#buffer.indexOf(marker);
    const newlineIndex = this.#buffer.indexOf(0x0a);
    if (newlineIndex >= 0 && (markerIndex < 0 || newlineIndex < markerIndex)) {
      this.#consumeLine(events, newlineIndex);
      return true;
    }
    if (markerIndex > 0) {
      const fragment = this.#buffer.subarray(0, markerIndex).toString("utf8");
      this.#buffer = this.#buffer.subarray(markerIndex);
      events.push({ kind: "text-fragment", text: fragment });
      return true;
    }
    return false;
  }

  #startsWithMarker() {
    return this.#buffer.length >= 2 &&
      this.#buffer[0] === marker[0] &&
      this.#buffer[1] === marker[1];
  }

  #consumeFrame(events) {
    if (this.#buffer.length < headerBytes) {
      return false;
    }
    const payloadLength = this.#buffer.readUInt16LE(7);
    if (payloadLength > MAX_PAYLOAD) {
      events.push({ kind: "protocol-error", error: "malformed-length" });
      this.#buffer = this.#buffer.subarray(2);
      return true;
    }
    const frameLength = 11 + payloadLength;
    if (this.#buffer.length < frameLength) {
      return false;
    }
    const candidate = this.#buffer.subarray(0, frameLength);
    this.#buffer = this.#buffer.subarray(frameLength);
    events.push(decodeFrame(candidate));
    return true;
  }

  #consumeLine(events, newlineIndex) {
    const line = textLine(this.#buffer.subarray(0, newlineIndex));
    this.#buffer = this.#buffer.subarray(newlineIndex + 1);
    if (line.length !== 0) {
      events.push({ kind: "text", line });
    }
  }

  #boundText(events) {
    if (this.#buffer.length <= this.#maximumTextBytes) {
      return;
    }
    const discardedBytes = this.#buffer.length - this.#maximumTextBytes;
    this.#buffer = this.#buffer.subarray(discardedBytes);
    events.push({ kind: "text-overflow", discardedBytes });
  }
}

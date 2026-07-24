// SPDX-License-Identifier: GPL-3.0-only

import { decodeMessage } from "./protocol.mjs";

function normalizeFrame(frame) {
  return {
    kind: "binary-frame",
    version: frame.version,
    type: frame.type,
    flags: frame.flags,
    sequence: frame.sequence,
    payloadHex: frame.payload.toString("hex"),
    decoded: decodeMessage(frame),
  };
}

function normalizeText(line) {
  if (!line.startsWith("BALANCE ")) {
    return { kind: "device-text", line };
  }
  try {
    const telemetry = JSON.parse(line.slice("BALANCE ".length));
    return { kind: "balance-telemetry", telemetry };
  } catch {
    return {
      kind: "device-text",
      line,
      parseError: "invalid BALANCE JSON",
    };
  }
}

export function normalizeCaptureEvent(event) {
  if (event.kind === "frame") {
    return normalizeFrame(event.frame);
  }
  if (event.kind === "text") {
    return normalizeText(event.line);
  }
  return { ...event };
}

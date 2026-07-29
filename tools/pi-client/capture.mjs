#!/usr/bin/env node
// SPDX-License-Identifier: GPL-3.0-only

import crypto from "node:crypto";
import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import process from "node:process";
import readline from "node:readline";
import { spawnSync } from "node:child_process";
import { fileURLToPath } from "node:url";

import { SerialClient } from "./client.mjs";
import { MixedTelemetryDecoder } from "./capture_decoder.mjs";
import { CaptureLifecycle } from "./capture_lifecycle.mjs";
import { normalizeCaptureEvent } from "./evidence.mjs";
import {
  MessageType,
  encodeDirectMotor,
  encodeUprightOffset,
} from "./protocol.mjs";
import { loadStagePlan } from "./stage_plan.mjs";

const repositoryDirectory = fileURLToPath(new URL("../../", import.meta.url));
const stages = new Set([
  "mpu-diagnostic",
  "controller-dry-run",
  "motor-transport",
  "lifted-wheel",
]);

function usage(output = process.stderr) {
  output.write(
    "usage: capture.mjs --port DEVICE --output DIRECTORY " +
    "[--duration SECONDS] [--poll-ms MILLISECONDS] [--baud RATE] " +
    "[--stage NAME] [--label TEXT] [--firmware FILE] " +
    "[--command-plan FILE] [--no-query]\n",
  );
}

function positiveInteger(value, label, maximum) {
  const parsed = Number(value);
  if (!Number.isInteger(parsed) || parsed <= 0 || parsed > maximum) {
    throw new Error(`${label} must be an integer from 1 to ${maximum}`);
  }
  return parsed;
}

function optionValue(arguments_, index, name) {
  const value = arguments_[index + 1];
  if (!value || value.startsWith("--")) {
    throw new Error(`${name} requires a value`);
  }
  return value;
}

function parseArguments(arguments_) {
  const options = {
    baud: 115200,
    durationSeconds: 120,
    pollMilliseconds: 500,
    query: true,
    stage: "mpu-diagnostic",
    label: "",
  };
  for (let index = 0; index < arguments_.length; index += 1) {
    const argument = arguments_[index];
    if (argument === "--help") return { help: true };
    if (argument === "--no-query") {
      options.query = false;
      continue;
    }
    const value = optionValue(arguments_, index, argument);
    index += 1;
    if (argument === "--port") options.port = value;
    else if (argument === "--output") options.output = path.resolve(value);
    else if (argument === "--firmware") options.firmware = path.resolve(value);
    else if (argument === "--command-plan") {
      options.commandPlanFile = path.resolve(value);
    }
    else if (argument === "--label") options.label = value;
    else if (argument === "--stage") options.stage = value;
    else if (argument === "--duration") {
      options.durationSeconds = positiveInteger(value, "duration", 86400);
    } else if (argument === "--poll-ms") {
      options.pollMilliseconds = positiveInteger(value, "poll-ms", 60000);
    } else if (argument === "--baud") {
      options.baud = positiveInteger(value, "baud", 4000000);
    } else {
      throw new Error(`unknown option: ${argument}`);
    }
  }
  if (!options.port) throw new Error("--port DEVICE is required");
  if (!options.output) throw new Error("--output DIRECTORY is required");
  if (!stages.has(options.stage)) {
    throw new Error(`unsupported stage: ${options.stage}`);
  }
  if (options.pollMilliseconds < 100) {
    throw new Error("poll-ms must be at least 100");
  }
  if (options.commandPlanFile) {
    options.commandPlan = loadStagePlan(options.commandPlanFile, {
      stage: options.stage,
      durationSeconds: options.durationSeconds,
    });
  }
  return options;
}

function git(arguments_, { trim = true } = {}) {
  const result = spawnSync("git", ["-C", repositoryDirectory, ...arguments_], {
    encoding: "utf8",
  });
  if (result.status !== 0) return null;
  if (trim) return result.stdout.trim();
  return result.stdout.replace(/\n$/, "");
}

function sha256File(file) {
  return crypto.createHash("sha256").update(fs.readFileSync(file)).digest("hex");
}

function firmwareMetadata(file) {
  if (!file) return null;
  const statistics = fs.statSync(file);
  if (!statistics.isFile()) throw new Error(`firmware is not a file: ${file}`);
  return {
    file: path.basename(file),
    bytes: statistics.size,
    sha256: sha256File(file),
  };
}

function sessionMetadata(options, startedUtc) {
  return {
    schemaVersion: 1,
    tool: "gausstop-balance-evidence-capture",
    startedUtc,
    stage: options.stage,
    label: options.label,
    serial: {
      port: options.port,
      baud: options.baud,
      queryIntervalMs: options.query ? options.pollMilliseconds : null,
    },
    requestedDurationSeconds: options.durationSeconds,
    host: {
      platform: process.platform,
      architecture: process.arch,
      osRelease: os.release(),
      nodeVersion: process.version,
    },
    repository: {
      branch: git(["branch", "--show-current"]),
      head: git(["rev-parse", "HEAD"]),
      statusPorcelain: git(["status", "--porcelain=v1"], { trim: false })
        ?.split("\n")
        .filter(Boolean) ?? null,
    },
    firmware: firmwareMetadata(options.firmware),
    commandPlan: options.commandPlan ?? null,
    safety: {
      flashesFirmware: false,
      sendsArm: options.commandPlan?.sendsArm ?? false,
      sendsMovement: options.commandPlan?.sendsMovement ?? false,
      maximumAbsoluteCommand:
        options.commandPlan?.maximumAbsoluteCommand ?? 0,
      startupCommands: ["hello", "disarm"],
      shutdownCommands: ["stop", "disarm"],
    },
  };
}

function prepareOutputDirectory(directory) {
  if (fs.existsSync(directory)) {
    throw new Error(`output directory already exists: ${directory}`);
  }
  fs.mkdirSync(directory, { recursive: true });
}

function writeJsonExclusive(file, value) {
  fs.writeFileSync(file, `${JSON.stringify(value, null, 2)}\n`, {
    encoding: "utf8",
    flag: "wx",
  });
}

function writeChecksums(directory, files) {
  const lines = files.map(file =>
    `${sha256File(path.join(directory, file))}  ${file}`);
  fs.writeFileSync(
    path.join(directory, "SHA256SUMS"),
    `${lines.join("\n")}\n`,
    { encoding: "utf8", flag: "wx" },
  );
}

function elapsedMilliseconds(started) {
  return Number(process.hrtime.bigint() - started) / 1e6;
}

async function capture(options) {
  prepareOutputDirectory(options.output);
  const startedUtc = new Date().toISOString();
  const startedMonotonic = process.hrtime.bigint();
  const startFile = path.join(options.output, "session-start.json");
  const endFile = path.join(options.output, "session-end.json");
  const rawFile = path.join(options.output, "raw-serial.bin");
  const eventsFile = path.join(options.output, "events.ndjson");
  writeJsonExclusive(startFile, sessionMetadata(options, startedUtc));
  const rawDescriptor = fs.openSync(rawFile, "wx");
  const eventsDescriptor = fs.openSync(eventsFile, "wx");
  const decoder = new MixedTelemetryDecoder();
  const counts = {};
  let rawBytes = 0;
  const lifecycle = new CaptureLifecycle();
  let client;
  let stream;
  let queryTimer;
  let durationTimer;
  const actionTimers = [];
  let notes;

  const record = body => {
    if (!lifecycle.canRecord) return;
    const event = {
      timeUtc: new Date().toISOString(),
      elapsedMs: Number(elapsedMilliseconds(startedMonotonic).toFixed(3)),
      ...body,
    };
    counts[event.kind] = (counts[event.kind] ?? 0) + 1;
    fs.writeSync(eventsDescriptor, `${JSON.stringify(event)}\n`);
  };

  const send = (type, command, payload = Buffer.alloc(0), fields = {}) => {
    const { sequence, frame } = client.sendDetailed(type, payload);
    record({
      kind: "host-command",
      direction: "host-to-device",
      command,
      type,
      sequence,
      payloadHex: payload.toString("hex"),
      rawFrameHex: frame.toString("hex"),
      ...fields,
    });
  };

  const executeAction = action => {
    try {
      switch (action.command) {
        case "mode":
          send(MessageType.SET_OPERATING_MODE, "set-operating-mode",
            Buffer.from([action.value]), { mode: action.value });
          break;
        case "arm":
          send(MessageType.ARM, "arm");
          break;
        case "direct": {
          const payload = encodeDirectMotor({
            left: action.left,
            right: action.right,
            leaseId: 0x47535450,
            lifetimeMs: action.lifetimeMs,
          });
          send(MessageType.SET_DIRECT_MOTOR, "direct-motor", payload, {
            left: action.left,
            right: action.right,
            lifetimeMs: action.lifetimeMs,
          });
          break;
        }
        case "stop":
          send(MessageType.STOP, "stop");
          break;
        case "disarm":
          send(MessageType.DISARM, "disarm");
          break;
        case "clear-fault":
          send(MessageType.CLEAR_FAULT, "clear-fault");
          break;
        case "upright-offset":
          send(MessageType.CONFIGURATION_UPDATE, "upright-offset",
            encodeUprightOffset(action.value), {
              valueDegrees: action.value,
            });
          break;
        case "emergency-stop":
          send(MessageType.EMERGENCY_STOP, "emergency-stop");
          break;
        default:
          throw new Error(`unsupported planned command: ${action.command}`);
      }
    } catch (error) {
      record({ kind: "capture-error", operation: "planned-command",
        command: action.command, message: error.message });
      finish("planned-command-error", 1);
    }
  };

  const poll = () => {
    try {
      send(MessageType.STATUS, "status");
      send(MessageType.IMU_TELEMETRY, "imu-telemetry");
      send(MessageType.MOTOR_TELEMETRY, "motor-telemetry");
      send(MessageType.ODOMETRY, "odometry");
      send(MessageType.ACTIVE_FAULTS, "active-faults");
    } catch (error) {
      record({ kind: "capture-error", operation: "telemetry-query",
        message: error.message });
      finish("serial-write-error", 1);
    }
  };

  const finalize = (reason, exitCode) => {
    if (stream) stream.destroy();
    if (notes) notes.close();
    try {
      client?.close();
    } catch (error) {
      record({ kind: "capture-error", operation: "serial-close",
        message: error.message });
    }
    record({ kind: "capture-end", reason });
    lifecycle.markFinalized();
    fs.closeSync(rawDescriptor);
    fs.closeSync(eventsDescriptor);
    writeJsonExclusive(endFile, {
      schemaVersion: 1,
      finishedUtc: new Date().toISOString(),
      elapsedMs: Number(elapsedMilliseconds(startedMonotonic).toFixed(3)),
      reason,
      rawBytes,
      eventCounts: counts,
    });
    writeChecksums(options.output, [
      "session-start.json",
      "session-end.json",
      "raw-serial.bin",
      "events.ndjson",
    ]);
    process.stdout.write(`evidence captured in ${options.output}\n`);
    process.exitCode = exitCode;
  };

  const finish = (reason, exitCode = 0) => {
    if (!lifecycle.beginFinish()) return;
    clearInterval(queryTimer);
    clearTimeout(durationTimer);
    for (const timer of actionTimers) clearTimeout(timer);
    try {
      send(MessageType.STOP, "stop");
      send(MessageType.DISARM, "disarm");
    } catch (error) {
      record({ kind: "capture-error", operation: "safe-shutdown",
        message: error.message });
    }
    setTimeout(() => finalize(reason, exitCode), 250);
  };

  try {
    client = new SerialClient(options.port, {
      baud: options.baud,
      hello: false,
    });
  } catch (error) {
    record({ kind: "capture-error", operation: "serial-open",
      message: error.message });
    finalize("serial-open-error", 1);
    return;
  }
  stream = client.readStream();
  stream.on("data", chunk => {
    if (!lifecycle.canHandleSerialEvent) return;
    try {
      fs.writeSync(rawDescriptor, chunk);
      rawBytes += chunk.length;
      for (const event of decoder.feed(chunk)) {
        record({
          direction: "device-to-host",
          ...normalizeCaptureEvent(event),
        });
      }
    } catch (error) {
      record({ kind: "capture-error", operation: "serial-decode",
        message: error.message });
      finish("capture-processing-error", 1);
    }
  });
  stream.on("error", error => {
    if (!lifecycle.canHandleSerialEvent) return;
    record({ kind: "capture-error", operation: "serial-read",
      message: error.message });
    finish("serial-error", 1);
  });
  stream.on("end", () => {
    if (lifecycle.canHandleSerialEvent) finish("serial-ended", 1);
  });

  actionTimers.push(setTimeout(() => {
    send(MessageType.HELLO, "hello");
    send(MessageType.DISARM, "disarm");
    send(MessageType.CAPABILITIES, "capabilities");
    if (options.query) {
      poll();
      queryTimer = setInterval(poll, options.pollMilliseconds);
    }
    for (const action of options.commandPlan?.actions ?? []) {
      actionTimers.push(setTimeout(() => executeAction(action), action.atMs));
    }
  }, 1000));

  if (process.stdin.isTTY) {
    process.stdout.write(
      "Type physical-action notes and press Enter; notes are logged only " +
      "and are never sent to the robot.\n",
    );
    notes = readline.createInterface({
      input: process.stdin,
      output: process.stdout,
    });
    notes.on("line", line => {
      if (line.trim()) {
        record({ kind: "operator-note", note: line.trim() });
      }
    });
    notes.on("SIGINT", () => finish("operator-interrupt"));
  }

  process.once("SIGINT", () => finish("operator-interrupt"));
  process.once("SIGTERM", () => finish("terminated", 1));
  durationTimer = setTimeout(
    () => finish("duration-complete"),
    options.durationSeconds * 1000 + 1000,
  );
}

try {
  const options = parseArguments(process.argv.slice(2));
  if (options.help) {
    usage(process.stdout);
  } else {
    await capture(options);
  }
} catch (error) {
  usage();
  process.stderr.write(`${error.message}\n`);
  process.exitCode = 2;
}

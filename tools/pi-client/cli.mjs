#!/usr/bin/env node
// SPDX-License-Identifier: GPL-3.0-only

import process from "node:process";

import { SerialClient } from "./client.mjs";
import { FrameParser, MessageType, decodeMessage } from "./protocol.mjs";

function usage() {
  console.error(
    "usage: cli.mjs --port DEVICE status|arm|disarm|stop|estop|clear-fault|" +
    "velocity VALUE [YAW]|yaw VALUE [VELOCITY]|telemetry",
  );
}

function parseArguments(arguments_) {
  const portIndex = arguments_.indexOf("--port");
  if (portIndex < 0 || !arguments_[portIndex + 1]) {
    throw new Error("--port DEVICE is required");
  }
  const port = arguments_[portIndex + 1];
  const commandArguments = arguments_.filter((_, index) =>
    index !== portIndex && index !== portIndex + 1);
  if (commandArguments.length === 0) throw new Error("command is required");
  return { port, command: commandArguments[0], values: commandArguments.slice(1) };
}

function finiteNumber(value, label) {
  const parsed = Number(value);
  if (!Number.isFinite(parsed)) throw new Error(`${label} must be a number`);
  return parsed;
}

function streamTelemetry(client) {
  const stream = client.readStream();
  const parser = new FrameParser();
  stream.on("data", chunk => {
    for (const frame of parser.feed(chunk)) {
      console.log(JSON.stringify(frame.error ? frame : decodeMessage(frame)));
    }
  });
  const requestTelemetry = () => {
    client.send(MessageType.STATUS);
    client.send(MessageType.IMU_TELEMETRY);
    client.send(MessageType.MOTOR_TELEMETRY);
    client.send(MessageType.ODOMETRY);
    client.send(MessageType.ACTIVE_FAULTS);
  };
  requestTelemetry();
  const interval = setInterval(requestTelemetry, 250);
  process.on("SIGINT", () => {
    clearInterval(interval);
    stream.destroy();
    client.close();
    process.exit(0);
  });
}

function readStatus(client) {
  const stream = client.readStream();
  const parser = new FrameParser();
  stream.on("data", chunk => {
    for (const frame of parser.feed(chunk)) {
      console.log(JSON.stringify(frame.error ? frame : decodeMessage(frame)));
    }
  });
  setTimeout(() => {
    stream.destroy();
    client.close();
  }, 1000);
}

function sendCommand(client, command, values) {
  const simple = {
    disarm: MessageType.DISARM,
    stop: MessageType.STOP,
    estop: MessageType.EMERGENCY_STOP,
    "clear-fault": MessageType.CLEAR_FAULT,
  };
  if (Object.hasOwn(simple, command)) {
    client.send(simple[command]);
    return false;
  }
  if (command === "status") {
    client.send(MessageType.STATUS);
    readStatus(client);
    return true;
  }
  if (command === "arm") {
    client.mode(1);
    client.send(MessageType.ARM);
    return false;
  }
  if (command === "velocity") {
    client.mode(2);
    client.movement(finiteNumber(values[0], "velocity"),
      finiteNumber(values[1] ?? "0", "yaw"));
    return false;
  }
  if (command === "yaw") {
    client.mode(2);
    client.movement(finiteNumber(values[1] ?? "0", "velocity"),
      finiteNumber(values[0], "yaw"));
    return false;
  }
  if (command === "telemetry") {
    streamTelemetry(client);
    return true;
  }
  throw new Error(`unknown command: ${command}`);
}

try {
  const { port, command, values } = parseArguments(process.argv.slice(2));
  const client = new SerialClient(port);
  const streaming = sendCommand(client, command, values);
  if (!streaming) client.close();
} catch (error) {
  usage();
  console.error(error.message);
  process.exitCode = 2;
}

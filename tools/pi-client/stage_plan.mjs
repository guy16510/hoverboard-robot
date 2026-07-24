// SPDX-License-Identifier: GPL-3.0-only

import crypto from "node:crypto";
import fs from "node:fs";
import path from "node:path";

import { MAX_TRANSPORT_TEST_COMMAND } from "./protocol.mjs";

const commands = new Set([
  "mode",
  "arm",
  "direct",
  "stop",
  "disarm",
  "clear-fault",
  "emergency-stop",
  "upright-offset",
]);

function fail(message) {
  throw new Error(`invalid command plan: ${message}`);
}

function requireInteger(value, name, minimum, maximum) {
  if (!Number.isInteger(value) || value < minimum || value > maximum) {
    fail(`${name} must be an integer from ${minimum} to ${maximum}`);
  }
}

export function normalizeStagePlan(value, {
  stage,
  durationSeconds,
} = {}) {
  if (!value || typeof value !== "object" || Array.isArray(value)) {
    fail("top level must be an object");
  }
  if (!Array.isArray(value.actions) || value.actions.length === 0) {
    fail("actions must be a non-empty array");
  }
  if (stage !== "motor-transport" && stage !== "lifted-wheel") {
    fail("command plans are allowed only for motor-transport or lifted-wheel");
  }
  const maximumAtMs = durationSeconds * 1000 - 500;
  let previousAtMs = -1;
  const actions = value.actions.map((action, index) => {
    if (!action || typeof action !== "object" || Array.isArray(action)) {
      fail(`action ${index} must be an object`);
    }
    requireInteger(action.atMs, `action ${index} atMs`, 0, maximumAtMs);
    if (action.atMs < previousAtMs) {
      fail("actions must be ordered by atMs");
    }
    previousAtMs = action.atMs;
    if (!commands.has(action.command)) {
      fail(`action ${index} has unsupported command ${action.command}`);
    }
    const normalized = { atMs: action.atMs, command: action.command };
    if (action.command === "mode") {
      requireInteger(action.value, `action ${index} mode`, 0, 3);
      normalized.value = action.value;
    }
    if (action.command === "direct") {
      requireInteger(action.left, `action ${index} left`,
        -MAX_TRANSPORT_TEST_COMMAND, MAX_TRANSPORT_TEST_COMMAND);
      requireInteger(action.right, `action ${index} right`,
        -MAX_TRANSPORT_TEST_COMMAND, MAX_TRANSPORT_TEST_COMMAND);
      const lifetimeMs = action.lifetimeMs ?? 500;
      requireInteger(lifetimeMs, `action ${index} lifetimeMs`, 100, 1000);
      normalized.left = action.left;
      normalized.right = action.right;
      normalized.lifetimeMs = lifetimeMs;
    }
    if (action.command === "upright-offset") {
      if (!Number.isFinite(action.value) ||
          action.value < -45 || action.value > 45) {
        fail(`action ${index} upright offset must be from -45 to 45 degrees`);
      }
      normalized.value = action.value;
    }
    return normalized;
  });

  const nonzeroDirect = actions.some(action =>
    action.command === "direct" &&
    (action.left !== 0 || action.right !== 0));
  if (stage === "motor-transport") {
    const modeIndex = actions.findIndex(action =>
      action.command === "mode" && action.value === 3);
    const zeroIndex = actions.findIndex(action =>
      action.command === "direct" &&
      action.left === 0 && action.right === 0);
    const armIndex = actions.findIndex(action => action.command === "arm");
    const offsetIndex = actions.findIndex(action =>
      action.command === "upright-offset");
    const firstNonzero = actions.findIndex(action =>
      action.command === "direct" &&
      (action.left !== 0 || action.right !== 0));
    if (modeIndex < 0) fail("motor-transport requires mode 3");
    if (!nonzeroDirect) fail("motor-transport requires a nonzero direct action");
    if (zeroIndex < 0 || zeroIndex > armIndex) {
      fail("motor-transport must command zero before arming");
    }
    if (armIndex < 0 || modeIndex > armIndex || armIndex > firstNonzero) {
      fail("motor-transport must select mode 3 and arm before nonzero");
    }
    if (offsetIndex >= armIndex) {
      fail("motor-transport upright offset must be set before arming");
    }
  }
  if (stage === "lifted-wheel") {
    const mode = actions.find(action => action.command === "mode");
    if (!mode || mode.value !== 1) fail("lifted-wheel requires mode 1");
    if (nonzeroDirect) fail("lifted-wheel cannot contain direct motor commands");
    if (!actions.some(action => action.command === "arm")) {
      fail("lifted-wheel requires an arm action");
    }
  }

  const ending = actions.slice(-3);
  if (ending.length !== 3 ||
      ending[0].command !== "direct" ||
      ending[0].left !== 0 || ending[0].right !== 0 ||
      ending[1].command !== "stop" ||
      ending[2].command !== "disarm") {
    fail("plan must end with direct zero, stop, and disarm");
  }
  return {
    schemaVersion: 1,
    actions,
    sendsArm: actions.some(action => action.command === "arm"),
    sendsMovement: nonzeroDirect,
    maximumAbsoluteCommand: actions
      .filter(action => action.command === "direct")
      .reduce((maximum, action) =>
        Math.max(maximum, Math.abs(action.left), Math.abs(action.right)), 0),
  };
}

export function loadStagePlan(file, options) {
  const bytes = fs.readFileSync(file);
  const plan = normalizeStagePlan(JSON.parse(bytes.toString("utf8")), options);
  return {
    file: path.basename(file),
    sha256: crypto.createHash("sha256").update(bytes).digest("hex"),
    ...plan,
  };
}

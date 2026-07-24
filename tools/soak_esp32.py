#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Prove the ESP32, MASTER, and SLAVE links remain stable before motion.

Run this after flashing and before the first nonzero wheel command. The gate keeps
both bridges off while requiring every telemetry snapshot to remain fully
acknowledged and healthy for the requested duration.
"""

from __future__ import annotations

import argparse
import queue
import time
from pathlib import Path
from typing import Any

from drive_esp32 import (
    SerialMonitor,
    default_log_path,
    fault_clear_confirmed,
    integer_pair,
    open_serial_without_reset,
    send_line,
    transport_overflows,
    validate_status,
    validate_transport_progress,
    zero_ready,
)

MAX_MASTER_COMMAND_AGE_MS = 300
MAX_SLAVE_AGE_MS = 80


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be greater than zero")
    return parsed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run a sustained zero-demand ESP32 dual-controller soak gate"
    )
    parser.add_argument("--port", required=True, help="ESP32 serial port")
    parser.add_argument(
        "--duration",
        type=positive_float,
        default=30.0,
        help="required continuously healthy time in seconds, default 30",
    )
    parser.add_argument("--ready-timeout", type=positive_float, default=5.0)
    parser.add_argument(
        "--log",
        type=Path,
        help="JSONL output path, defaults to the normal timestamped ESP32 log",
    )
    return parser.parse_args()


def remote_parser(status: dict[str, Any]) -> tuple[int, int, int, int]:
    raw = status.get("remote_parser", [-1, -1, -1, -1])
    if not isinstance(raw, list) or len(raw) != 4:
        raise RuntimeError(f"Malformed MASTER remote parser telemetry: {raw!r}")
    return tuple(int(value) for value in raw)  # type: ignore[return-value]


def validate_age_margins(status: dict[str, Any]) -> None:
    master_age = int(status.get("command_age_ms", 65535))
    slave_feedback_age = int(status.get("slave_feedback_age_ms", 65535))
    slave_command_age = int(status.get("slave_command_age_ms", 65535))
    if master_age > MAX_MASTER_COMMAND_AGE_MS:
        raise RuntimeError(
            f"MASTER command age margin is too small: {master_age} ms"
        )
    if slave_feedback_age > MAX_SLAVE_AGE_MS:
        raise RuntimeError(
            f"SLAVE feedback age margin is too small: {slave_feedback_age} ms"
        )
    if slave_command_age > MAX_SLAVE_AGE_MS:
        raise RuntimeError(
            f"SLAVE command age margin is too small: {slave_command_age} ms"
        )


def validate_soak_snapshot(
    baseline: dict[str, Any], status: dict[str, Any]
) -> None:
    validate_status(status)
    if not zero_ready(status):
        raise RuntimeError(
            "Zero-demand session lost READY, exact acknowledgment, peer health, "
            "or zero outputs"
        )
    validate_age_margins(status)
    if transport_overflows(status) != (0, 0, 0, 0):
        raise RuntimeError("Transport overflow appeared during zero-demand soak")

    baseline_remote = remote_parser(baseline)
    current_remote = remote_parser(status)
    if current_remote[2] != baseline_remote[2]:
        raise RuntimeError(
            "MASTER invalid command frames increased: "
            f"{baseline_remote[2]}->{current_remote[2]}"
        )
    if current_remote[3] != baseline_remote[3]:
        raise RuntimeError(
            "MASTER framing errors increased: "
            f"{baseline_remote[3]}->{current_remote[3]}"
        )
    if int(status.get("crc_errors", -1)) != int(baseline.get("crc_errors", -2)):
        raise RuntimeError("ESP32 feedback CRC errors increased during soak")
    if int(status.get("ack_timeouts", -1)) != int(
        baseline.get("ack_timeouts", -2)
    ):
        raise RuntimeError("ESP32 acknowledgment timeouts increased during soak")


def validate_soak_progress(
    baseline: dict[str, Any], final: dict[str, Any], samples: int
) -> None:
    if samples < 5:
        raise RuntimeError(f"Too few validated soak snapshots: {samples}")
    validate_soak_snapshot(baseline, final)
    validate_transport_progress(baseline, final)
    baseline_remote = remote_parser(baseline)
    final_remote = remote_parser(final)
    if final_remote[0] <= baseline_remote[0] or final_remote[1] <= baseline_remote[1]:
        raise RuntimeError(
            "MASTER command parser did not make valid progress: "
            f"bytes={baseline_remote[0]}->{final_remote[0]}, "
            f"frames={baseline_remote[1]}->{final_remote[1]}"
        )


def run_soak(
    duration: float, ready_timeout: float, port: Any, monitor: SerialMonitor
) -> int:
    monitor.set_stage("soak_clear")
    send_line(port, "disable")
    send_line(port, "clearfault")
    monitor.wait_for(
        fault_clear_confirmed,
        ready_timeout,
        "fault-clear confirmation while disabled",
        allow_faults=True,
        ignore_uninitialized=True,
    )

    monitor.set_stage("soak_ready")
    send_line(port, "enable")
    baseline = monitor.wait_for(
        zero_ready, ready_timeout, "READY,READY zero-demand acknowledgment"
    )
    validate_age_margins(baseline)

    monitor.set_stage("soak")
    deadline = time.monotonic() + duration
    samples = 0
    final = baseline
    while time.monotonic() < deadline:
        requested_at = time.monotonic()
        monitor.request_status()
        while True:
            remaining = min(0.25, max(0.0, deadline - time.monotonic()))
            if remaining == 0.0:
                break
            try:
                event = monitor.events.get(timeout=remaining)
            except queue.Empty as exc:
                if time.monotonic() >= deadline:
                    break
                raise RuntimeError("No ESP32 status received during soak") from exc
            if event.received_monotonic < requested_at:
                continue
            final = event.payload
            validate_soak_snapshot(baseline, final)
            samples += 1
            break
        time.sleep(0.05)

    validate_soak_progress(baseline, final, samples)
    return samples


def main() -> int:
    args = parse_args()
    log_path = args.log or default_log_path()
    failure: BaseException | None = None
    samples = 0

    with open_serial_without_reset(args.port) as port:
        time.sleep(1.0)
        port.reset_input_buffer()
        monitor = SerialMonitor(port, log_path)
        monitor.start()
        try:
            samples = run_soak(args.duration, args.ready_timeout, port, monitor)
        except BaseException as exc:
            failure = exc
        finally:
            try:
                monitor.set_stage("soak_stop")
                send_line(port, "stop")
                time.sleep(0.10)
                send_line(port, "disable")
                time.sleep(0.20)
                monitor.request_status()
                time.sleep(0.20)
            finally:
                monitor.close()

    print(
        f"zero-demand soak samples={samples} duration_s={args.duration:.1f} "
        f"log={log_path}"
    )
    if failure is not None:
        raise failure
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

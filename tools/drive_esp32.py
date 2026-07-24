#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Run a bounded, acknowledged ESP32 dual-wheel bench command.

Requires: python -m pip install pyserial

The tool refuses nonzero demand until MASTER and SLAVE acknowledge a zero-demand
READY session. It records every structured status line as JSONL and always sends
stop and disable before closing the serial port.
"""

from __future__ import annotations

import argparse
import json
import queue
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

try:
    import serial
except ImportError as exc:
    raise SystemExit("Install pyserial first: python -m pip install pyserial") from exc


STATUS_PREFIX = "STATUS "
READY = 1
ACTIVE = 2
FAULTED = 3
SHUTDOWN = 4
COMMAND_DEADBAND = 50
MAX_FEEDBACK_AGE_MS = 300


def command_value(value: str) -> int:
    parsed = int(value)
    if parsed < -1000 or parsed > 1000:
        raise argparse.ArgumentTypeError("wheel commands must be between -1000 and 1000")
    return parsed


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be greater than zero")
    return parsed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run an acknowledged independent-wheel ESP32 bench test"
    )
    parser.add_argument("--port", required=True, help="ESP32 serial port")
    parser.add_argument("--left", type=command_value, required=True)
    parser.add_argument("--right", type=command_value, required=True)
    parser.add_argument("--duration", type=positive_float, default=1.0)
    parser.add_argument("--rate-hz", type=positive_float, default=20.0)
    parser.add_argument("--ramp", type=int, default=10, choices=range(1, 1001))
    parser.add_argument("--ready-timeout", type=positive_float, default=3.0)
    parser.add_argument(
        "--log",
        type=Path,
        help="JSONL output path, defaults to logs/esp32-drive-<UTC timestamp>.jsonl",
    )
    parser.add_argument(
        "--allow-high-command",
        action="store_true",
        help="required when either absolute command exceeds 250",
    )
    parser.add_argument(
        "--confirm-wheels-lifted",
        action="store_true",
        help="required for every nonzero motion test",
    )
    return parser.parse_args()


def utc_timestamp() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def default_log_path() -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return Path("logs") / f"esp32-drive-{stamp}.jsonl"


def send_line(port: serial.Serial, line: str) -> None:
    port.write((line + "\n").encode("ascii"))
    port.flush()


def open_serial_without_reset(port_name: str) -> serial.Serial:
    """Open the ESP32 console without asserting DTR or RTS reset lines."""
    port = serial.Serial(
        port=None,
        baudrate=115200,
        timeout=0.05,
        write_timeout=0.5,
    )
    port.dtr = False
    port.rts = False
    port.port = port_name
    port.open()
    port.dtr = False
    port.rts = False
    return port


@dataclass
class StatusEvent:
    received_monotonic: float
    payload: dict[str, Any]


class SerialMonitor:
    def __init__(self, port: serial.Serial, log_path: Path) -> None:
        self.port = port
        self.events: queue.Queue[StatusEvent] = queue.Queue()
        self.latest: StatusEvent | None = None
        self._stop = threading.Event()
        self._stage = "startup"
        self._thread = threading.Thread(target=self._run, name="serial-monitor", daemon=True)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        self._log = log_path.open("w", encoding="utf-8")

    def start(self) -> None:
        self._thread.start()

    def set_stage(self, stage: str) -> None:
        self._stage = stage

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        self._log.close()

    def _record(self, payload: dict[str, Any]) -> None:
        record = {"host_time": utc_timestamp(), "stage": self._stage, "status": payload}
        self._log.write(json.dumps(record, sort_keys=True) + "\n")
        self._log.flush()

    def _run(self) -> None:
        while not self._stop.is_set():
            raw = self.port.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            print(line)
            if not line.startswith(STATUS_PREFIX):
                continue
            try:
                payload = json.loads(line[len(STATUS_PREFIX) :])
            except json.JSONDecodeError:
                print("WARN malformed STATUS JSON", file=sys.stderr)
                continue
            event = StatusEvent(time.monotonic(), payload)
            self.latest = event
            self.events.put(event)
            self._record(payload)

    def request_status(self) -> None:
        send_line(self.port, "status")

    def wait_for(
        self,
        predicate: Callable[[dict[str, Any]], bool],
        timeout: float,
        description: str,
        allow_faults: bool = False,
    ) -> dict[str, Any]:
        started = time.monotonic()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.request_status()
            wait = min(0.12, max(0.0, deadline - time.monotonic()))
            try:
                event = self.events.get(timeout=wait)
            except queue.Empty:
                continue
            if event.received_monotonic < started:
                continue
            validate_status(event.payload, allow_faults=allow_faults)
            if predicate(event.payload):
                return event.payload
        raise RuntimeError(f"Timed out waiting for {description}")


def states(status: dict[str, Any]) -> tuple[int, int]:
    raw = status.get("states", [-1, -1])
    return int(raw[0]), int(raw[1])


def faults(status: dict[str, Any]) -> tuple[int, int]:
    raw = status.get("faults", [1, 1])
    return int(raw[0]), int(raw[1])


def integer_pair(status: dict[str, Any], name: str, default: int) -> tuple[int, int]:
    raw = status.get(name, [default, default])
    if not isinstance(raw, list) or len(raw) != 2:
        raise RuntimeError(f"Malformed {name} telemetry: {raw!r}")
    return int(raw[0]), int(raw[1])


def transport_overflows(status: dict[str, Any]) -> tuple[int, int, int, int]:
    raw = status.get("transport_overflows", [1, 1, 1, 1])
    if not isinstance(raw, list) or len(raw) != 4:
        raise RuntimeError(f"Malformed transport overflow telemetry: {raw!r}")
    return tuple(int(value) for value in raw)  # type: ignore[return-value]


def validate_status(status: dict[str, Any], allow_faults: bool = False) -> None:
    if int(status.get("protocol", -1)) != 2:
        raise RuntimeError(f"Unexpected protocol version: {status.get('protocol')}")
    master_state, slave_state = states(status)
    master_fault, slave_fault = faults(status)
    if not allow_faults and (master_fault or slave_fault):
        raise RuntimeError(
            f"Controller fault, master=0x{master_fault:08x}, slave=0x{slave_fault:08x}"
        )
    if master_state == SHUTDOWN or slave_state == SHUTDOWN:
        raise RuntimeError(
            f"Unsafe controller state, master={master_state}, slave={slave_state}"
        )
    if not allow_faults and (
        master_state == FAULTED or slave_state == FAULTED
    ):
        raise RuntimeError(
            f"Unsafe controller state, master={master_state}, slave={slave_state}"
        )
    feedback_age = int(status.get("esp_feedback_age_ms", 999999))
    if feedback_age > MAX_FEEDBACK_AGE_MS:
        raise RuntimeError(
            f"ESP32 feedback is stale: age={feedback_age} ms, "
            f"limit={MAX_FEEDBACK_AGE_MS} ms"
        )
    overflows = transport_overflows(status)
    if not allow_faults and any(overflows):
        raise RuntimeError(f"Transport overflow reported: {overflows}")
    if not bool(status.get("pa4_raw_high")) and not bool(status.get("pa4_bypass")):
        raise RuntimeError("MASTER PA4 is low without the explicit bypass")
    if not bool(status.get("slave_pa4_raw_high")):
        raise RuntimeError("SLAVE PA4 is low")
    halls = integer_pair(status, "halls", 0)
    if any(hall < 1 or hall > 6 for hall in halls):
        raise RuntimeError(f"Invalid Hall telemetry: {halls}")
    compare = integer_pair(status, "compare", -1)
    bridge = integer_pair(status, "bridge", -1)
    applied = integer_pair(status, "applied", 1001)
    for wheel, (command, offset, enabled) in enumerate(
        zip(applied, compare, bridge), start=1
    ):
        if offset < 0 or offset > 100:
            raise RuntimeError(f"Wheel {wheel} compare offset is invalid: {offset}")
        if enabled not in (0, 1) or enabled != int(offset > 0):
            raise RuntimeError(
                f"Wheel {wheel} bridge/compare disagreement: bridge={enabled}, "
                f"compare={offset}"
            )
        magnitude = abs(command)
        applied_matches_bridge = (
            magnitude >= COMMAND_DEADBAND
            if enabled
            else magnitude < COMMAND_DEADBAND
        )
        if magnitude > 1000 or not applied_matches_bridge:
            raise RuntimeError(
                f"Wheel {wheel} applied/bridge disagreement: applied={command}, "
                f"bridge={enabled}"
            )


def validate_motion_result(
    commands: tuple[int, int],
    bridge_seen: tuple[bool, bool],
    odometer_delta: tuple[int, int],
) -> None:
    for wheel, (command, bridge_enabled, delta) in enumerate(
        zip(commands, bridge_seen, odometer_delta), start=1
    ):
        if command == 0:
            continue
        if not bridge_enabled:
            raise RuntimeError(f"Wheel {wheel} bridge never enabled")
        if delta == 0:
            raise RuntimeError(f"Wheel {wheel} produced no Hall movement")
        if command * delta < 0:
            raise RuntimeError(
                f"Wheel {wheel} moved opposite the commanded direction: "
                f"command={command}, odometer_delta={delta}"
            )


def validate_transport_progress(
    baseline: dict[str, Any], current: dict[str, Any]
) -> None:
    baseline_crc = int(baseline.get("crc_errors", -1))
    current_crc = int(current.get("crc_errors", -1))
    if baseline_crc < 0 or current_crc != baseline_crc:
        raise RuntimeError(
            f"ESP32 CRC errors increased: baseline={baseline_crc}, "
            f"current={current_crc}"
        )
    baseline_timeouts = int(baseline.get("ack_timeouts", -1))
    current_timeouts = int(current.get("ack_timeouts", -1))
    if baseline_timeouts < 0 or current_timeouts != baseline_timeouts:
        raise RuntimeError(
            "ESP32 acknowledgment timeout count increased: "
            f"baseline={baseline_timeouts}, current={current_timeouts}"
        )
    baseline_tx = int(baseline.get("tx", -1))
    baseline_rx = int(baseline.get("rx", -1))
    current_tx = int(current.get("tx", -1))
    current_rx = int(current.get("rx", -1))
    if (
        baseline_tx < 0
        or baseline_rx < 0
        or current_tx <= baseline_tx
        or current_rx <= baseline_rx
    ):
        raise RuntimeError(
            "Bidirectional transport did not progress: "
            f"tx={baseline_tx}->{current_tx}, rx={baseline_rx}->{current_rx}"
        )


def acknowledged(status: dict[str, Any]) -> bool:
    return bool(status.get("exact_ack"))


def outputs_off(status: dict[str, Any]) -> bool:
    return (
        integer_pair(status, "applied", 1) == (0, 0)
        and integer_pair(status, "compare", 1) == (0, 0)
        and integer_pair(status, "bridge", 1) == (0, 0)
    )


def zero_ready(status: dict[str, Any]) -> bool:
    return (
        bool(status.get("session_ready"))
        and bool(status.get("peer_healthy"))
        and acknowledged(status)
        and states(status) == (READY, READY)
        and transport_overflows(status) == (0, 0, 0, 0)
        and outputs_off(status)
    )


def motion_session_healthy(status: dict[str, Any]) -> bool:
    operational_states = {READY, ACTIVE}
    return (
        bool(status.get("session_ready"))
        and bool(status.get("peer_healthy"))
        and transport_overflows(status) == (0, 0, 0, 0)
        and all(state in operational_states for state in states(status))
    )


def disabled(status: dict[str, Any]) -> bool:
    return states(status) == (0, 0) and outputs_off(status)


def fault_clear_confirmed(status: dict[str, Any]) -> bool:
    return (
        disabled(status)
        and acknowledged(status)
        and faults(status) == (0, 0)
        and transport_overflows(status) == (0, 0, 0, 0)
        and not bool(status.get("clear_pending"))
    )


def run_test(
    args: argparse.Namespace, port: serial.Serial, monitor: SerialMonitor
) -> tuple[int, float, float, tuple[int, int]]:
    monitor.set_stage("pre_enable")
    send_line(port, "disable")
    send_line(port, f"ramp {args.ramp}")
    send_line(port, "clearfault")
    monitor.wait_for(
        fault_clear_confirmed,
        args.ready_timeout,
        "fault-clear confirmation while disabled",
        allow_faults=True,
    )

    monitor.set_stage("zero_enable")
    send_line(port, "enable")
    ready_status = monitor.wait_for(
        zero_ready, args.ready_timeout, "READY,READY zero-demand acknowledgment"
    )
    baseline_odometers = integer_pair(ready_status, "odometers", 0)

    moving = args.left != 0 or args.right != 0
    monitor.set_stage("demand")
    interval = 1.0 / args.rate_hz
    start = time.monotonic()
    next_send = start
    next_status = start
    send_times: list[float] = []
    bridge_seen = [False, False]

    while time.monotonic() - start < args.duration:
        now = time.monotonic()
        if now >= next_send:
            send_line(port, f"lr {args.left} {args.right}")
            send_times.append(now)
            while next_send <= now:
                next_send += interval
        if now >= next_status:
            monitor.request_status()
            next_status = now + 0.10
        latest = monitor.latest
        if latest is not None:
            status = latest.payload
            validate_status(status)
            for wheel, enabled in enumerate(integer_pair(status, "bridge", 0)):
                bridge_seen[wheel] = bridge_seen[wheel] or bool(enabled)
            if not acknowledged(status) and now - latest.received_monotonic > 0.10:
                raise RuntimeError("Command acknowledgment is missing")
            master_state, slave_state = states(status)
            if moving and not motion_session_healthy(status):
                raise RuntimeError(
                    "Motion session lost readiness or peer health: "
                    f"master={master_state}, slave={slave_state}"
                )
        sleep_for = min(0.002, max(0.0, next_send - time.monotonic()))
        if sleep_for:
            time.sleep(sleep_for)

    final_status = monitor.wait_for(
        acknowledged, min(args.ready_timeout, 1.0), "final command acknowledgment"
    )
    validate_transport_progress(ready_status, final_status)
    for wheel, enabled in enumerate(integer_pair(final_status, "bridge", 0)):
        bridge_seen[wheel] = bridge_seen[wheel] or bool(enabled)
    final_odometers = integer_pair(final_status, "odometers", 0)
    odometer_delta = tuple(
        final - baseline
        for final, baseline in zip(final_odometers, baseline_odometers)
    )
    validate_motion_result(
        (args.left, args.right),
        (bridge_seen[0], bridge_seen[1]),
        odometer_delta,
    )
    if send_times:
        intervals = [b - a for a, b in zip(send_times, send_times[1:])]
        elapsed = max(send_times[-1] - send_times[0], interval)
        achieved_rate = len(send_times) / (elapsed + interval)
        worst_interval = max(intervals, default=0.0)
    else:
        achieved_rate = 0.0
        worst_interval = 0.0
    return (
        len(send_times),
        max(worst_interval, 0.0),
        achieved_rate,
        odometer_delta,
    )


def main() -> int:
    args = parse_args()
    moving = args.left != 0 or args.right != 0
    if moving and not args.confirm_wheels_lifted:
        raise SystemExit("Refusing motion, pass --confirm-wheels-lifted for bench testing")
    if max(abs(args.left), abs(args.right)) > 250 and not args.allow_high_command:
        raise SystemExit(
            "Refusing command above 250, validate low-speed motion first or pass "
            "--allow-high-command"
        )

    log_path = args.log or default_log_path()
    sent = 0
    worst_interval = 0.0
    achieved_rate = 0.0
    odometer_delta = (0, 0)
    failure: BaseException | None = None

    with open_serial_without_reset(args.port) as port:
        time.sleep(1.0)
        port.reset_input_buffer()
        monitor = SerialMonitor(port, log_path)
        monitor.start()
        try:
            sent, worst_interval, achieved_rate, odometer_delta = run_test(
                args, port, monitor
            )
        except BaseException as exc:
            failure = exc
        finally:
            try:
                monitor.set_stage("pre_stop")
                monitor.request_status()
                time.sleep(0.10)
                send_line(port, "stop")
                monitor.set_stage("post_stop")
                time.sleep(0.10)
                monitor.request_status()
                time.sleep(0.10)
                send_line(port, "disable")
                monitor.set_stage("post_disable")
                time.sleep(0.10)
                monitor.request_status()
                time.sleep(0.20)
            finally:
                monitor.close()

    print(
        f"sent={sent} achieved_rate_hz={achieved_rate:.2f} "
        f"worst_command_interval_ms={worst_interval * 1000.0:.2f} "
        f"odometer_delta={odometer_delta[0]},{odometer_delta[1]} log={log_path}"
    )
    if failure is not None:
        raise failure
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

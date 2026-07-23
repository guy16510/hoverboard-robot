#!/usr/bin/env python3
"""Run a bounded, acknowledged ESP32 dual-wheel bench command.

Requires: python -m pip install pyserial

The tool refuses nonzero demand until MASTER and SLAVE acknowledge a zero-demand
READY session for the current enable epoch. It records every structured status
line as JSONL and always sends stop and disable before closing the serial port.
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
DISABLED = 0
READY = 1
ACTIVE = 2
FAULTED = 3
SHUTDOWN = 4
CLEAR_NONE = 0
CLEAR_OK = 1


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
        "--allow-pa4-bypass",
        action="store_true",
        help="required for motion when BENCH_MASTER_PA4_BYPASS is detected",
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
        *,
        allow_faults: bool = False,
    ) -> dict[str, Any]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.request_status()
            wait = min(0.12, max(0.0, deadline - time.monotonic()))
            try:
                event = self.events.get(timeout=wait)
            except queue.Empty:
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


def clear_results(status: dict[str, Any]) -> tuple[int, int]:
    raw = status.get("clear_results", [CLEAR_NONE, CLEAR_NONE])
    return int(raw[0]), int(raw[1])


def validate_status(status: dict[str, Any], *, allow_faults: bool = False) -> None:
    if int(status.get("protocol", -1)) != 3:
        raise RuntimeError(f"Unexpected protocol version: {status.get('protocol')}")
    master_state, slave_state = states(status)
    master_fault, slave_fault = faults(status)
    if not allow_faults and (master_fault or slave_fault):
        raise RuntimeError(
            f"Controller fault, master=0x{master_fault:08x}, slave=0x{slave_fault:08x}"
        )
    if master_state == SHUTDOWN or slave_state == SHUTDOWN:
        raise RuntimeError(
            f"Controller shutdown, master={master_state}, slave={slave_state}"
        )
    if not allow_faults and (master_state == FAULTED or slave_state == FAULTED):
        raise RuntimeError(
            f"Unsafe controller state, master={master_state}, slave={slave_state}"
        )
    if int(status.get("esp_feedback_age_ms", 999999)) > 150:
        raise RuntimeError("ESP32 feedback is stale")


def acknowledged(status: dict[str, Any]) -> bool:
    return bool(status.get("exact_ack"))


def zero_ready(status: dict[str, Any]) -> bool:
    enable_epoch = int(status.get("enable_epoch", 0))
    controller_epochs = [int(value) for value in status.get("controller_enable_epochs", [])]
    return (
        enable_epoch != 0
        and controller_epochs == [enable_epoch, enable_epoch]
        and bool(status.get("session_ready"))
        and bool(status.get("peer_healthy"))
        and acknowledged(status)
        and states(status) == (READY, READY)
        and list(status.get("applied", [1, 1])) == [0, 0]
    )


def disabled(status: dict[str, Any]) -> bool:
    return states(status) == (DISABLED, DISABLED) and list(
        status.get("applied", [1, 1])
    ) == [0, 0]


def wait_for_clear(monitor: SerialMonitor, timeout: float) -> dict[str, Any]:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        monitor.request_status()
        try:
            event = monitor.events.get(timeout=0.12)
        except queue.Empty:
            continue
        status = event.payload
        validate_status(status, allow_faults=True)
        results = clear_results(status)
        if any(result not in (CLEAR_NONE, CLEAR_OK) for result in results):
            raise RuntimeError(
                f"Controller rejected fault clear, master={results[0]}, slave={results[1]}"
            )
        if (
            results == (CLEAR_OK, CLEAR_OK)
            and faults(status) == (0, 0)
            and disabled(status)
            and not bool(status.get("clear_pending"))
        ):
            return status
    raise RuntimeError("Timed out waiting for atomic fault-clear confirmation")


def run_test(
    args: argparse.Namespace, port: serial.Serial, monitor: SerialMonitor
) -> tuple[int, float, float]:
    monitor.set_stage("pre_enable")
    send_line(port, "disable")
    send_line(port, f"ramp {args.ramp}")
    monitor.wait_for(disabled, args.ready_timeout, "disabled zero-output state", allow_faults=True)
    send_line(port, "clearfault")
    wait_for_clear(monitor, args.ready_timeout)

    monitor.set_stage("zero_enable")
    send_line(port, "enable")
    ready_status = monitor.wait_for(
        zero_ready, args.ready_timeout, "READY,READY zero-demand epoch acknowledgment"
    )

    moving = args.left != 0 or args.right != 0
    if moving and bool(ready_status.get("pa4_bypass")) and not args.allow_pa4_bypass:
        raise RuntimeError(
            "Refusing motion with PA4 bypass firmware, pass --allow-pa4-bypass only for the authorized bench image"
        )

    monitor.set_stage("demand")
    interval = 1.0 / args.rate_hz
    start = time.monotonic()
    next_send = start
    next_status = start
    send_times: list[float] = []

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
            if now - latest.received_monotonic > 0.15:
                raise RuntimeError("No fresh controller status during demand")
            status = latest.payload
            validate_status(status)
            if now - start > 0.15 and not acknowledged(status):
                raise RuntimeError("Command acknowledgment is missing")
            if int(status.get("enable_epoch", 0)) == 0:
                raise RuntimeError("Enable epoch is missing")
            master_state, slave_state = states(status)
            if moving and bool(status.get("session_ready")):
                if master_state not in (READY, ACTIVE) or slave_state not in (READY, ACTIVE):
                    raise RuntimeError(
                        f"Unexpected demand state, master={master_state}, slave={slave_state}"
                    )
        sleep_for = min(0.002, max(0.0, next_send - time.monotonic()))
        if sleep_for:
            time.sleep(sleep_for)

    monitor.request_status()
    if send_times:
        intervals = [b - a for a, b in zip(send_times, send_times[1:])]
        elapsed = max(send_times[-1] - send_times[0], interval)
        achieved_rate = len(send_times) / (elapsed + interval)
        worst_interval = max(intervals, default=0.0)
    else:
        achieved_rate = 0.0
        worst_interval = 0.0
    return len(send_times), max(worst_interval, 0.0), achieved_rate


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
    failure: BaseException | None = None

    with serial.Serial(args.port, 115200, timeout=0.05, write_timeout=0.5) as port:
        time.sleep(1.0)
        port.reset_input_buffer()
        monitor = SerialMonitor(port, log_path)
        monitor.start()
        try:
            sent, worst_interval, achieved_rate = run_test(args, port, monitor)
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
        f"worst_command_interval_ms={worst_interval * 1000.0:.2f} log={log_path}"
    )
    if failure is not None:
        raise failure
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

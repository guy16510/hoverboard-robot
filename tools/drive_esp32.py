#!/usr/bin/env python3
"""Stream independent wheel commands to the ESP32 coordinator.

Requires: python -m pip install pyserial

The ESP32 intentionally requires fresh motion commands. This tool repeatedly sends
`lr LEFT RIGHT`, then always sends stop and disable before closing the port.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError as exc:
    raise SystemExit("Install pyserial first: python -m pip install pyserial") from exc


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
        description="Stream independent left/right wheel commands to the ESP32"
    )
    parser.add_argument("--port", required=True, help="ESP32 serial port")
    parser.add_argument("--left", type=command_value, required=True)
    parser.add_argument("--right", type=command_value, required=True)
    parser.add_argument("--duration", type=positive_float, default=1.0)
    parser.add_argument("--rate-hz", type=positive_float, default=20.0)
    parser.add_argument("--ramp", type=int, default=10, choices=range(1, 1001))
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


def send_line(port: serial.Serial, line: str) -> None:
    port.write((line + "\n").encode("ascii"))
    port.flush()


def drain_output(port: serial.Serial) -> None:
    deadline = time.monotonic() + 0.15
    while time.monotonic() < deadline:
        waiting = port.in_waiting
        if waiting:
            data = port.read(waiting).decode("utf-8", errors="replace")
            sys.stdout.write(data)
            sys.stdout.flush()
        else:
            time.sleep(0.01)


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

    interval = 1.0 / args.rate_hz
    with serial.Serial(args.port, 115200, timeout=0.02, write_timeout=0.5) as port:
        time.sleep(1.0)
        send_line(port, "disable")
        send_line(port, f"ramp {args.ramp}")
        send_line(port, "clearfault")
        time.sleep(0.3)
        send_line(port, "enable")
        start = time.monotonic()
        next_send = start
        try:
            while time.monotonic() - start < args.duration:
                now = time.monotonic()
                if now >= next_send:
                    send_line(port, f"lr {args.left} {args.right}")
                    next_send += interval
                drain_output(port)
                time.sleep(0.001)
        finally:
            send_line(port, "stop")
            time.sleep(0.15)
            send_line(port, "disable")
            send_line(port, "status")
            drain_output(port)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

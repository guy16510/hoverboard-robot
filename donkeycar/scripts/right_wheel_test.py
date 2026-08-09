#!/usr/bin/env python3
"""Run one bounded, right-wheel-only WinXu commissioning pulse."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from trashcan_robot.config import SerialConfig
from trashcan_robot.protocol import ACK, DISARM
from trashcan_robot.transport import SerialMotorTransport

LINEAR_VELOCITY = -0.04375
ANGULAR_VELOCITY = 0.10
MAX_LINEAR_VELOCITY = 0.35
MAX_ANGULAR_VELOCITY = 0.8
COMMAND_HZ = 20
COMMAND_FRAMES = 20
COMMAND_SECONDS = 1.0


def expected_outputs() -> tuple[float, float]:
    linear = LINEAR_VELOCITY / MAX_LINEAR_VELOCITY
    yaw = ANGULAR_VELOCITY / MAX_ANGULAR_VELOCITY
    return linear + yaw, (linear - yaw) * -1.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="One 1-second, 25% right-wheel-only commissioning pulse",
    )
    parser.add_argument("--port", default="/dev/cu.usbserial-0001")
    parser.add_argument(
        "--execute",
        action="store_true",
        help="required to send the physical motion command",
    )
    parser.add_argument(
        "--wheel-lifted",
        action="store_true",
        help="confirm the right wheel is fully off the ground",
    )
    parser.add_argument(
        "--study-disconnected",
        action="store_true",
        help="confirm the WinXu Study connector is disconnected",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    left, right = expected_outputs()
    print(f"expected left={left:+.6f} right={right:+.6f}")
    if abs(left) > 1e-9 or abs(right - 0.25) > 1e-9:
        raise RuntimeError("mixer safety assertion failed")

    if not args.execute:
        print("calculation only; pass --execute to allow hardware motion")
        return 0
    if not args.wheel_lifted or not args.study_disconnected:
        raise SystemExit(
            "refusing motion: pass --wheel-lifted and --study-disconnected"
        )

    config = SerialConfig(
        port=args.port,
        baud=115200,
        timeout_seconds=0.10,
        reconnect_seconds=1.0,
        lease_ms=500,
        command_hz=COMMAND_HZ,
    )
    transport = SerialMotorTransport(config)
    acknowledged = 0
    try:
        transport.connect()
        for _ in range(5):
            transport.send_command(0.0, 0.0)
            time.sleep(1.0 / COMMAND_HZ)

        started = time.monotonic()
        deadline = started + COMMAND_SECONDS
        for index in range(COMMAND_FRAMES):
            delay = started + index / COMMAND_HZ - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            if time.monotonic() >= deadline:
                break
            transport.send_command(LINEAR_VELOCITY, ANGULAR_VELOCITY)
            acknowledged += 1

        delay = deadline - time.monotonic()
        if delay > 0:
            time.sleep(delay)
    finally:
        if transport.is_connected():
            try:
                transport.send_command(0.0, 0.0)
                print("final zero ACK: PASS")
            except Exception as exc:
                print(f"final zero ACK: FAIL ({exc})", file=sys.stderr)
            try:
                transport._request(DISARM, b"", ACK)
                print("disarm ACK: PASS")
            except Exception as exc:
                print(f"disarm ACK: FAIL ({exc})", file=sys.stderr)
        transport.disconnect()

    print(f"motion ACKs: {acknowledged}/{COMMAND_FRAMES}")
    if acknowledged != COMMAND_FRAMES:
        raise RuntimeError("the bounded command did not receive every ACK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

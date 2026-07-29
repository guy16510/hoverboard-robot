#!/usr/bin/env python3
"""Bounded lifted-wheel bring-up for the ESP32 drive coordinator."""

from __future__ import annotations

import argparse
import json
import random
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(SCRIPT_DIR))

from trashcan_robot.config import load_config
from trashcan_robot.protocol import (
    ACK,
    ARM,
    CAPABILITIES,
    CLEAR_FAULT,
    DISARM,
    DRIVE_MODE,
    DRIVE_TELEMETRY,
    EMERGENCY_STOP,
    ERROR,
    FAULTS,
    HELLO,
    MOTOR,
    ODOMETRY,
    SET_OPERATING_MODE,
    SET_VELOCITY_YAW,
    STATUS,
    STOP,
    FrameDecoder,
    decode_message,
    encode_frame,
    encode_motion,
)

from preflight import (
    PREARM_HEALTH_FLAGS,
    REQUIRED_HEALTH_FLAGS,
    SERIAL_SOURCE,
    open_serial_port,
)

LINEAR_GAIN = 650.0
ANGULAR_GAIN = 350.0
LEVELS = (25, 50, 100)
MANEUVERS = {
    "left-forward": (1, 0),
    "right-forward": (0, 1),
    "right-reverse": (0, -1),
    "both-forward": (1, 1),
    "both-reverse": (-1, -1),
    "opposite-steering": (1, -1),
}


def wheel_targets_to_motion(left: int, right: int) -> tuple[float, float]:
    """Invert the firmware mixer for targets within its unsaturated range."""
    if max(abs(left), abs(right)) > 250:
        raise ValueError("wheel target exceeds firmware limit")
    return (
        (left + right) / (2.0 * LINEAR_GAIN),
        (left - right) / (2.0 * ANGULAR_GAIN),
    )


def needs_escalation(left_delta: int, right_delta: int) -> bool:
    return left_delta == 0 and right_delta == 0


def accept_motion_or_escalate(
    name: str,
    level: int,
    final_level: int,
    odometry_delta: list[int],
) -> bool:
    if not needs_escalation(*odometry_delta):
        return True
    if level >= final_level:
        raise RuntimeError(
            f"{name} produced no Hall odometry at final level {level}: "
            f"{odometry_delta}"
        )
    return False


def command_levels(value: str) -> tuple[int, ...]:
    levels = tuple(int(item) for item in value.split(","))
    if not levels or any(level < 1 or level > 250 for level in levels):
        raise argparse.ArgumentTypeError("levels must be between 1 and 250")
    return levels


def maneuver_names(value: str) -> tuple[str, ...]:
    names = tuple(item.strip() for item in value.split(","))
    unknown = [name for name in names if name not in MANEUVERS]
    if not names or unknown:
        raise argparse.ArgumentTypeError(f"unknown maneuvers: {unknown}")
    return names


def write_log(path: Path, results: list[dict]) -> None:
    record = {
        "utc": datetime.now(timezone.utc).isoformat(),
        "results": results,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n")


class Session:
    def __init__(self, port) -> None:
        self.port = port
        self.decoder = FrameDecoder()
        self.sequence = 0

    def send(self, message_type: int, payload: bytes = b"") -> int:
        sequence = self.sequence
        self.port.write(encode_frame(message_type, sequence, payload))
        self.port.flush()
        self.sequence = (self.sequence + 1) & 0xFFFF
        return sequence

    def wait(self, message_type: int, sequence: int, timeout: float = 2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            data = self.port.read(self.port.in_waiting or 1)
            for frame in self.decoder.feed(data):
                if frame.sequence != sequence:
                    continue
                if frame.message_type == ERROR:
                    raise RuntimeError(
                        f"request 0x{message_type:02x} rejected: "
                        f"{decode_message(frame)}"
                    )
                if frame.message_type == message_type:
                    return frame
        raise TimeoutError(
            f"timeout waiting for 0x{message_type:02x} sequence {sequence}"
        )

    def command(self, message_type: int, payload: bytes = b"") -> None:
        sequence = self.send(message_type, payload)
        frame = self.wait(ACK, sequence)
        decoded = decode_message(frame)
        if (
            decoded.get("request_type") != message_type
            or decoded.get("status") != 0
        ):
            raise RuntimeError(f"unexpected acknowledgment: {decoded}")

    def query(self, message_type: int) -> dict:
        sequence = self.send(message_type)
        return decode_message(self.wait(message_type, sequence))

    def handshake(self, timeout: float = 5.0) -> dict:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            sequence = self.send(HELLO)
            try:
                return decode_message(
                    self.wait(CAPABILITIES, sequence, min(0.75, timeout))
                )
            except TimeoutError:
                continue
        raise TimeoutError("ESP32 handshake failed")


def establish_zero_and_arm(
    session: Session, lease_ms: int
) -> tuple[int, bytes, dict]:
    capabilities = session.handshake()
    if capabilities.get("dry_run") or not (
        int(capabilities.get("supported_modes", 0)) & (1 << DRIVE_MODE)
    ):
        raise RuntimeError(f"wrong firmware capabilities: {capabilities}")
    lease_id = random.getrandbits(32)
    zero = encode_motion(0.0, 0.0, lease_id, lease_ms)
    session.command(SET_OPERATING_MODE, bytes((DRIVE_MODE,)))
    session.command(SET_VELOCITY_YAW, zero)
    session.command(CLEAR_FAULT)
    deadline = time.monotonic() + 8.0
    latest = {}
    while time.monotonic() < deadline:
        session.command(SET_VELOCITY_YAW, zero)
        latest = session.query(STATUS)
        if (
            latest["operating_mode"] == DRIVE_MODE
            and latest["active_source"] == SERIAL_SOURCE
            and latest["health_flags"] & PREARM_HEALTH_FLAGS
            == PREARM_HEALTH_FLAGS
            and latest["faults"] == 0
        ):
            break
    else:
        raise RuntimeError(f"zero session did not become ready: {latest}")
    session.command(SET_VELOCITY_YAW, zero)
    session.command(ARM)
    status = session.query(STATUS)
    if (
        status["state"] != 4
        or status["operating_mode"] != DRIVE_MODE
        or status["active_source"] != SERIAL_SOURCE
        or status["health_flags"] & REQUIRED_HEALTH_FLAGS
        != REQUIRED_HEALTH_FLAGS
        or status["faults"]
    ):
        raise RuntimeError(f"ARM preflight failed: {status}")
    return lease_id, zero, status


def stop_and_verify(session: Session, zero: bytes) -> dict:
    session.command(SET_VELOCITY_YAW, zero)
    deadline = time.monotonic() + 1.0
    latest = {}
    while time.monotonic() < deadline:
        latest = session.query(MOTOR)
        if latest["calculated"] == [0, 0] and latest["applied"] == [0, 0]:
            break
        session.command(SET_VELOCITY_YAW, zero)
    else:
        session.command(EMERGENCY_STOP)
        raise RuntimeError(f"motor did not return to exact zero: {latest}")
    session.command(STOP)
    session.command(DISARM)
    return latest


def run_segment(
    session: Session,
    lease_ms: int,
    name: str,
    left: int,
    right: int,
    duration: float,
) -> dict:
    lease_id, zero, armed_status = establish_zero_and_arm(session, lease_ms)
    before = session.query(ODOMETRY)
    linear, yaw = wheel_targets_to_motion(left, right)
    demand = encode_motion(linear, yaw, lease_id, lease_ms)
    started = time.monotonic()
    drive = {}
    motor = {}
    try:
        while time.monotonic() - started < duration:
            session.command(SET_VELOCITY_YAW, demand)
            elapsed = time.monotonic() - started
            if not drive and elapsed >= duration / 3.0:
                drive = session.query(DRIVE_TELEMETRY)
                motor = session.query(MOTOR)
            time.sleep(0.04)
        session.command(SET_VELOCITY_YAW, zero)
        faults = session.query(FAULTS)
        if faults["balance"] or faults["master"] or faults["slave"]:
            session.command(EMERGENCY_STOP)
            raise RuntimeError(f"fault during {name}: {faults}")
    finally:
        final_motor = stop_and_verify(session, zero)
    after = session.query(ODOMETRY)
    delta = [after["left"] - before["left"], after["right"] - before["right"]]
    for requested, observed, wheel in zip((left, right), delta, ("left", "right")):
        if requested and observed and (requested > 0) != (observed > 0):
            raise RuntimeError(
                f"{name} {wheel} direction mismatch: requested={requested}, "
                f"odometry_delta={observed}"
            )
    return {
        "name": name,
        "requested_wheels": [left, right],
        "motion": [round(linear, 6), round(yaw, 6)],
        "duration_seconds": duration,
        "armed_status": armed_status,
        "drive": drive,
        "active_motor": motor,
        "odometry_before": before,
        "odometry_after": after,
        "odometry_delta": delta,
        "faults": faults,
        "final_motor": final_motor,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(ROOT / "config/robot.yaml"))
    parser.add_argument("--duration", type=float, default=0.75)
    parser.add_argument("--levels", type=command_levels, default=LEVELS)
    parser.add_argument(
        "--maneuvers",
        type=maneuver_names,
        default=(
            "left-forward",
            "right-forward",
            "both-forward",
            "both-reverse",
            "opposite-steering",
        ),
    )
    parser.add_argument("--confirm-wheels-lifted", action="store_true")
    parser.add_argument("--log", type=Path, required=True)
    args = parser.parse_args()
    if not args.confirm_wheels_lifted:
        raise SystemExit("--confirm-wheels-lifted is required")
    if not 0.1 <= args.duration <= 1.0:
        raise SystemExit("--duration must be between 0.1 and 1.0 seconds")
    config = load_config(args.config)
    results = []
    port = open_serial_port(config)
    session = Session(port)
    try:
        time.sleep(0.5)
        port.reset_input_buffer()
        for name in args.maneuvers:
            left_sign, right_sign = MANEUVERS[name]
            for level in args.levels:
                result = run_segment(
                    session,
                    config.serial.lease_ms,
                    name,
                    left_sign * level,
                    right_sign * level,
                    args.duration,
                )
                results.append(result)
                write_log(args.log, results)
                accepted = accept_motion_or_escalate(
                    name,
                    level,
                    args.levels[-1],
                    result["odometry_delta"],
                )
                print(
                    "SEGMENT PASS" if accepted else "SEGMENT NO MOTION",
                    name,
                    f"level={level}",
                    f"odometry_delta={result['odometry_delta']}",
                    flush=True,
                )
                if accepted:
                    break
    finally:
        try:
            session.command(EMERGENCY_STOP)
            session.command(DISARM)
        except Exception:
            pass
        port.close()
    write_log(args.log, results)
    record = json.loads(args.log.read_text())
    print(json.dumps(record, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

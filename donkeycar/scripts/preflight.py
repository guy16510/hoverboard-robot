#!/usr/bin/env python3
from __future__ import annotations

import argparse
import random
import struct
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import serial

from trashcan_robot.config import load_config
from trashcan_robot.protocol import (
    ARM,
    DISARM,
    DRIVE_MODE,
    HELLO,
    SET_OPERATING_MODE,
    SET_VELOCITY_YAW,
    STATUS,
    STOP,
    FrameDecoder,
    encode_frame,
    encode_motion,
)

DRIVING_STATE = 4
SERIAL_SOURCE = 2
REQUIRED_HEALTH_FLAGS = 0x55


def write_frame(port: serial.Serial, message_type: int, sequence: int, payload: bytes = b"") -> int:
    port.write(encode_frame(message_type, sequence, payload))
    port.flush()
    return (sequence + 1) & 0xFFFF


def wait_for(decoder: FrameDecoder, port: serial.Serial, message_type: int, timeout: float):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        data = port.read(port.in_waiting or 1)
        for frame in decoder.feed(data):
            if frame.message_type == message_type:
                return frame
    raise TimeoutError(f"timed out waiting for message type 0x{message_type:02x}")


def settle_serial_port(
    port: serial.Serial,
    delay_seconds: float,
    sleep=time.sleep,
) -> None:
    sleep(delay_seconds)
    port.reset_input_buffer()


def zero_demand_startup_requests(
    lease_id: int,
    lease_ms: int,
) -> list[tuple[int, bytes]]:
    return [
        (SET_OPERATING_MODE, bytes((DRIVE_MODE,))),
        (SET_VELOCITY_YAW, encode_motion(0.0, 0.0, lease_id, lease_ms)),
        (ARM, b""),
        (STATUS, b""),
    ]


def validate_preflight_status(
    state: int,
    mode: int,
    source: int,
    health: int,
    faults: int,
) -> None:
    context = (
        f"state={state} mode={mode} source={source} "
        f"health=0x{health:02x} faults=0x{faults:08x}"
    )
    if state != DRIVING_STATE:
        raise RuntimeError(f"{context}; expected state {DRIVING_STATE}")
    if mode != DRIVE_MODE:
        raise RuntimeError(f"{context}; expected drive mode")
    if source != SERIAL_SOURCE:
        raise RuntimeError(f"{context}; expected serial source")
    missing_health = REQUIRED_HEALTH_FLAGS & ~health
    if missing_health:
        raise RuntimeError(
            f"{context}; health is missing 0x{missing_health:02x}"
        )
    if faults:
        raise RuntimeError(f"{context}; ESP32 reports active faults")


def main() -> int:
    parser = argparse.ArgumentParser(description="Zero-motion Raspberry Pi to ESP32 preflight")
    parser.add_argument("--config", default=str(ROOT / "config/robot.yaml"))
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--reset-delay", type=float, default=2.0)
    args = parser.parse_args()

    config = load_config(args.config)
    decoder = FrameDecoder()
    sequence = 0

    with serial.Serial(
        config.serial.port,
        config.serial.baud,
        timeout=config.serial.timeout_seconds,
        write_timeout=config.serial.timeout_seconds,
    ) as port:
        try:
            settle_serial_port(port, args.reset_delay)
            sequence = write_frame(port, HELLO, sequence)
            capabilities = wait_for(decoder, port, 0x02, args.timeout)
            if len(capabilities.payload) != 12:
                raise RuntimeError(f"unexpected capabilities payload length {len(capabilities.payload)}")
            protocol, dry_run, _web, modes, control_hz, motor_hz, max_payload, _keys = struct.unpack(
                "<BBBBHHHH", capabilities.payload
            )
            if protocol != 1:
                raise RuntimeError(f"unsupported Pi protocol {protocol}")
            if dry_run:
                raise RuntimeError("ESP32 reports dry-run firmware, refusing drive preflight")
            if not modes & (1 << DRIVE_MODE):
                raise RuntimeError("ESP32 does not advertise drive mode")
            if max_payload < 10:
                raise RuntimeError("ESP32 payload limit cannot carry a movement lease")

            requests = zero_demand_startup_requests(
                lease_id=random.getrandbits(32),
                lease_ms=config.serial.lease_ms,
            )
            for message_type, payload in requests:
                sequence = write_frame(port, message_type, sequence, payload)
            status = wait_for(decoder, port, STATUS, args.timeout)
            if len(status.payload) != 16:
                raise RuntimeError(f"unexpected status payload length {len(status.payload)}")
            state, mode, source, health, faults, overruns, rejected = struct.unpack(
                "<BBBBIII", status.payload
            )
            validate_preflight_status(state, mode, source, health, faults)

            print(
                "PREFLIGHT PASS",
                f"state={state}",
                f"mode={mode}",
                f"source={source}",
                f"health=0x{health:02x}",
                f"control_hz={control_hz}",
                f"motor_hz={motor_hz}",
                f"overruns={overruns}",
                f"rejected={rejected}",
            )
        finally:
            try:
                sequence = write_frame(port, STOP, sequence)
                write_frame(port, DISARM, sequence)
            except Exception:
                pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

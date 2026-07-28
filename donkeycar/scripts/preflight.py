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
    CLEAR_FAULT,
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
CAPABILITIES = 0x02
IMU_HEALTHY = 1 << 0
FEEDBACK_FRESH = 1 << 2
FEEDBACK_HEALTHY = 1 << 3
OUTPUT_ENABLED = 1 << 4
ZERO_ACKNOWLEDGED = 1 << 5
SERIAL_CONNECTED = 1 << 6
PREARM_HEALTH_FLAGS = (
    IMU_HEALTHY
    | FEEDBACK_FRESH
    | FEEDBACK_HEALTHY
    | ZERO_ACKNOWLEDGED
    | SERIAL_CONNECTED
)
REQUIRED_HEALTH_FLAGS = PREARM_HEALTH_FLAGS | OUTPUT_ENABLED


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


def wait_for_sequence(
    decoder: FrameDecoder,
    port: serial.Serial,
    message_type: int,
    sequence: int,
    timeout: float,
):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        data = port.read(port.in_waiting or 1)
        for frame in decoder.feed(data):
            if frame.message_type == message_type and frame.sequence == sequence:
                return frame
    raise TimeoutError(
        f"timed out waiting for message type 0x{message_type:02x} "
        f"sequence {sequence}"
    )


def settle_serial_port(
    port: serial.Serial,
    delay_seconds: float,
    sleep=time.sleep,
) -> None:
    sleep(delay_seconds)
    port.reset_input_buffer()


def open_serial_port(config, serial_factory=serial.Serial):
    port = serial_factory(
        port=None,
        baudrate=config.serial.baud,
        timeout=config.serial.timeout_seconds,
        write_timeout=config.serial.timeout_seconds,
    )
    port.dtr = False
    port.rts = False
    port.port = config.serial.port
    port.open()
    port.dtr = False
    port.rts = False
    return port


def wait_for_handshake(
    decoder: FrameDecoder,
    port: serial.Serial,
    sequence: int,
    timeout: float,
):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        sequence = write_frame(port, HELLO, sequence)
        try:
            capabilities = wait_for(
                decoder,
                port,
                CAPABILITIES,
                min(0.75, max(0.01, deadline - time.monotonic())),
            )
            return sequence, capabilities
        except TimeoutError:
            continue
    raise TimeoutError("timed out waiting for ESP32 handshake")


def zero_demand_startup_requests(
    lease_id: int,
    lease_ms: int,
) -> list[tuple[int, bytes]]:
    return [
        (SET_OPERATING_MODE, bytes((DRIVE_MODE,))),
        (SET_VELOCITY_YAW, encode_motion(0.0, 0.0, lease_id, lease_ms)),
        (CLEAR_FAULT, b""),
        (STATUS, b""),
    ]


def zero_acknowledged(health: int) -> bool:
    return health & PREARM_HEALTH_FLAGS == PREARM_HEALTH_FLAGS


def decode_status(payload: bytes) -> tuple[int, int, int, int, int, int, int]:
    if len(payload) != 16:
        raise RuntimeError(f"unexpected status payload length {len(payload)}")
    return struct.unpack("<BBBBIII", payload)


def wait_for_zero_acknowledgment(
    decoder: FrameDecoder,
    port: serial.Serial,
    sequence: int,
    zero_payload: bytes,
    timeout: float,
) -> tuple[int, tuple[int, int, int, int, int, int, int]]:
    deadline = time.monotonic() + timeout
    latest_status = None
    while time.monotonic() < deadline:
        sequence = write_frame(port, SET_VELOCITY_YAW, sequence, zero_payload)
        sequence = write_frame(port, STATUS, sequence)
        try:
            frame = wait_for(
                decoder,
                port,
                STATUS,
                min(0.25, max(0.01, deadline - time.monotonic())),
            )
        except TimeoutError:
            continue
        latest_status = decode_status(frame.payload)
        _state, mode, source, health, _faults, _overruns, _rejected = latest_status
        if (
            mode == DRIVE_MODE
            and source == SERIAL_SOURCE
            and zero_acknowledged(health)
        ):
            return sequence, latest_status
        time.sleep(0.05)
    detail = "no status received" if latest_status is None else (
        f"last status state={latest_status[0]} mode={latest_status[1]} "
        f"source={latest_status[2]} health=0x{latest_status[3]:02x} "
        f"faults=0x{latest_status[4]:08x}"
    )
    raise TimeoutError(f"timed out waiting for exact zero acknowledgment; {detail}")


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

    with open_serial_port(config) as port:
        try:
            settle_serial_port(port, args.reset_delay)
            sequence, capabilities = wait_for_handshake(
                decoder,
                port,
                sequence,
                args.timeout,
            )
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
            mode_type, mode_payload = requests[0]
            sequence = write_frame(port, mode_type, sequence, mode_payload)
            zero_type, zero_payload = requests[1]
            sequence = write_frame(port, zero_type, sequence, zero_payload)
            clear_type, clear_payload = requests[2]
            sequence = write_frame(port, clear_type, sequence, clear_payload)
            sequence, _ = wait_for_zero_acknowledgment(
                decoder,
                port,
                sequence,
                zero_payload,
                args.timeout,
            )
            sequence = write_frame(
                port,
                SET_VELOCITY_YAW,
                sequence,
                zero_payload,
            )
            sequence = write_frame(port, ARM, sequence)
            sequence = write_frame(port, STATUS, sequence)
            status = wait_for_sequence(
                decoder,
                port,
                STATUS,
                (sequence - 1) & 0xFFFF,
                args.timeout,
            )
            state, mode, source, health, faults, overruns, rejected = decode_status(
                status.payload
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

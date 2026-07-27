#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
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
    HELLO,
    MOTOR,
    SET_OPERATING_MODE,
    STATUS,
    STOP,
    FrameDecoder,
    encode_frame,
)

CAPABILITIES = 0x02
SET_DIRECT_MOTOR = 0x24
DIRECT_MODE = 3


def encode_direct(
    left: int,
    right: int,
    lease_id: int,
    lifetime_ms: int,
) -> bytes:
    if max(abs(left), abs(right)) > 50:
        raise ValueError("direct motor command exceeds firmware limit")
    return struct.pack(
        "<hhIH",
        left,
        right,
        lease_id & 0xFFFFFFFF,
        lifetime_ms,
    )


def bounded_direct_requests(lease_id: int) -> list[tuple[int, bytes]]:
    return [
        (SET_OPERATING_MODE, bytes((DIRECT_MODE,))),
        (SET_DIRECT_MOTOR, encode_direct(0, 0, lease_id, 500)),
        (ARM, b""),
        (STATUS, b""),
        (SET_DIRECT_MOTOR, encode_direct(10, 10, lease_id, 250)),
        (SET_DIRECT_MOTOR, encode_direct(0, 0, lease_id, 500)),
        (STOP, b""),
        (DISARM, b""),
    ]


def write_frame(
    port: serial.Serial,
    message_type: int,
    sequence: int,
    payload: bytes = b"",
) -> int:
    port.write(encode_frame(message_type, sequence, payload))
    port.flush()
    return (sequence + 1) & 0xFFFF


def send_request(
    port: serial.Serial,
    request: tuple[int, bytes],
    sequence: int,
) -> int:
    message_type, payload = request
    return write_frame(port, message_type, sequence, payload)


def wait_for(
    decoder: FrameDecoder,
    port: serial.Serial,
    message_type: int,
    timeout: float,
):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        data = port.read(port.in_waiting or 1)
        for frame in decoder.feed(data):
            if frame.message_type == message_type:
                return frame
    raise TimeoutError(
        f"timed out waiting for message type 0x{message_type:02x}"
    )


def wait_for_handshake(
    decoder: FrameDecoder,
    port: serial.Serial,
    sequence: int,
    timeout: float,
) -> tuple[int, object]:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        sequence = write_frame(port, HELLO, sequence)
        try:
            return sequence, wait_for(decoder, port, CAPABILITIES, 0.75)
        except TimeoutError:
            continue
    raise TimeoutError("timed out waiting for ESP32 handshake")


def require_armed_status(payload: bytes) -> dict[str, int]:
    if len(payload) != 16:
        raise RuntimeError(f"unexpected status payload length {len(payload)}")
    state, mode, source, health, faults, overruns, rejected = struct.unpack(
        "<BBBBIII",
        payload,
    )
    context = (
        f"state={state} mode={mode} source={source} "
        f"health=0x{health:02x} faults=0x{faults:08x}"
    )
    if state not in (3, 4):
        raise RuntimeError(f"{context}; ESP32 refused ARM")
    if mode != DIRECT_MODE or source != 2:
        raise RuntimeError(f"{context}; direct serial control is not active")
    if not health & (1 << 4) or faults:
        raise RuntimeError(f"{context}; motor output is not safely enabled")
    return {
        "state": state,
        "mode": mode,
        "source": source,
        "health": health,
        "faults": faults,
        "overruns": overruns,
        "rejected": rejected,
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="One bounded low-demand direct-motor transport test",
    )
    parser.add_argument("--config", default=str(ROOT / "config/robot.yaml"))
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--reset-delay", type=float, default=1.0)
    parser.add_argument("--handshake-seconds", type=float, default=3.0)
    args = parser.parse_args()

    config = load_config(args.config)
    decoder = FrameDecoder()
    sequence = 0
    lease_id = random.getrandbits(32)
    requests = bounded_direct_requests(lease_id)
    zero_payload = requests[1][1]

    with serial.Serial(
        config.serial.port,
        config.serial.baud,
        timeout=config.serial.timeout_seconds,
        write_timeout=config.serial.timeout_seconds,
    ) as port:
        try:
            time.sleep(args.reset_delay)
            port.reset_input_buffer()
            sequence, _ = wait_for_handshake(
                decoder,
                port,
                sequence,
                args.handshake_seconds,
            )
            sequence = write_frame(port, DISARM, sequence)

            sequence = send_request(port, requests[0], sequence)
            sequence = send_request(port, requests[1], sequence)
            time.sleep(0.75)
            sequence = send_request(port, requests[2], sequence)
            sequence = send_request(port, requests[3], sequence)
            status = wait_for(decoder, port, STATUS, args.timeout)
            status_decoded = require_armed_status(status.payload)

            sequence = send_request(port, requests[4], sequence)
            time.sleep(0.125)
            sequence = write_frame(port, MOTOR, sequence)
            motor = wait_for(decoder, port, MOTOR, args.timeout)
            time.sleep(0.125)
            sequence = send_request(port, requests[5], sequence)
            sequence = send_request(port, requests[6], sequence)
            sequence = send_request(port, requests[7], sequence)
            print(
                json.dumps(
                    {
                        "command": [10, 10],
                        "duration_ms": 250,
                        "motor": list(motor.payload),
                        "status": status_decoded,
                    },
                    sort_keys=True,
                )
            )
        finally:
            try:
                sequence = write_frame(
                    port,
                    SET_DIRECT_MOTOR,
                    sequence,
                    zero_payload,
                )
                sequence = write_frame(port, STOP, sequence)
                write_frame(port, DISARM, sequence)
            except Exception:
                pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

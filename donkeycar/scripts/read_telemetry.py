#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import struct
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import serial

from trashcan_robot.config import load_config
from trashcan_robot.protocol import (
    DISARM,
    FAULTS,
    HELLO,
    IMU,
    MOTOR,
    ODOMETRY,
    STATUS,
    STOP,
    FrameDecoder,
    encode_frame,
)

CAPABILITIES = 0x02
TELEMETRY_TYPES = {
    "status": STATUS,
    "imu": IMU,
    "motor": MOTOR,
    "odometry": ODOMETRY,
    "faults": FAULTS,
}


def read_only_request_types() -> list[int]:
    return [
        HELLO,
        DISARM,
        *TELEMETRY_TYPES.values(),
        STOP,
        DISARM,
    ]


def write_frame(
    port: serial.Serial,
    message_type: int,
    sequence: int,
) -> int:
    port.write(encode_frame(message_type, sequence))
    port.flush()
    return (sequence + 1) & 0xFFFF


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


def decode_status(payload: bytes) -> dict[str, int] | None:
    if len(payload) != 16:
        return None
    state, mode, source, health, faults, overruns, rejected = struct.unpack(
        "<BBBBIII",
        payload,
    )
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
        description="Read ESP32 telemetry without arming or sending movement",
    )
    parser.add_argument("--config", default=str(ROOT / "config/robot.yaml"))
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--reset-delay", type=float, default=2.0)
    args = parser.parse_args()

    config = load_config(args.config)
    decoder = FrameDecoder()
    sequence = 0
    responses: dict[str, object] = {}

    with serial.Serial(
        config.serial.port,
        config.serial.baud,
        timeout=config.serial.timeout_seconds,
        write_timeout=config.serial.timeout_seconds,
    ) as port:
        try:
            time.sleep(args.reset_delay)
            port.reset_input_buffer()
            sequence = write_frame(port, HELLO, sequence)
            capabilities = wait_for(
                decoder,
                port,
                CAPABILITIES,
                args.timeout,
            )
            responses["capabilities"] = list(capabilities.payload)

            sequence = write_frame(port, DISARM, sequence)
            for name, message_type in TELEMETRY_TYPES.items():
                sequence = write_frame(port, message_type, sequence)
                response = wait_for(decoder, port, message_type, args.timeout)
                responses[name] = list(response.payload)
                if name == "status":
                    responses["status_decoded"] = decode_status(response.payload)
        finally:
            try:
                sequence = write_frame(port, STOP, sequence)
                write_frame(port, DISARM, sequence)
            except Exception:
                pass

    print(json.dumps(responses, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
from __future__ import annotations

import argparse
import struct
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from trashcan_robot.config import load_config
from trashcan_robot.protocol import (
    ARM,
    DRIVE_MODE,
    HELLO,
    SET_OPERATING_MODE,
    STATUS,
    FrameDecoder,
    encode_frame,
)

import serial


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


def main() -> int:
    parser = argparse.ArgumentParser(description="Zero-motion Raspberry Pi to ESP32 preflight")
    parser.add_argument("--config", default=str(ROOT / "config/robot.yaml"))
    parser.add_argument("--timeout", type=float, default=2.0)
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
        port.reset_input_buffer()
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

        sequence = write_frame(port, SET_OPERATING_MODE, sequence, bytes((DRIVE_MODE,)))
        sequence = write_frame(port, ARM, sequence)
        sequence = write_frame(port, STATUS, sequence)
        status = wait_for(decoder, port, STATUS, args.timeout)
        if len(status.payload) != 16:
            raise RuntimeError(f"unexpected status payload length {len(status.payload)}")
        state, mode, source, health, faults, overruns, rejected = struct.unpack("<BBBBIII", status.payload)
        if mode != DRIVE_MODE:
            raise RuntimeError(f"ESP32 remained in operating mode {mode}, expected drive mode")
        if faults:
            raise RuntimeError(f"ESP32 reports active fault mask 0x{faults:08x}")

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
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
from __future__ import annotations

import json
import random
import struct
import time

import serial

from direct_motor_test import (
    CAPABILITIES,
    DIRECT_MODE,
    SET_DIRECT_MOTOR,
    encode_direct,
    require_armed_status,
    wait_for,
    wait_for_handshake,
    write_frame,
)

from trashcan_robot.protocol import (
    ARM,
    CLEAR_FAULT,
    DISARM,
    MOTOR,
    ODOMETRY,
    SET_OPERATING_MODE,
    STATUS,
    STOP,
    FrameDecoder,
)

PORT = "/dev/cu.usbserial-0001"
BAUD = 115200


def query(decoder, port, message_type, sequence):
    sequence = write_frame(port, message_type, sequence)
    return sequence, wait_for(decoder, port, message_type, 2.0)


def odometers(payload: bytes) -> tuple[int, int]:
    if len(payload) != 20:
        raise RuntimeError(f"unexpected odometry payload length {len(payload)}")
    left, right, _, _ = struct.unpack("<iiiQ", payload)
    return left, right


def main() -> int:
    decoder = FrameDecoder()
    sequence = 0
    lease_id = random.getrandbits(32)
    zero = encode_direct(0, 0, lease_id, 1000)
    # Start at the SWD profile's full-scale command. Lower commands previously
    # reached only partial PWM, failed to overcome startup torque, and latched
    # the Hall startup timeout before a meaningful powered attempt occurred.
    tests = [
        ("both", 250, 250, 1500),
        ("left", 250, 0, 1000),
        ("right", 0, 250, 1000),
    ]
    results = []

    port = serial.Serial(
        port=None,
        baudrate=BAUD,
        timeout=0.05,
        write_timeout=0.5,
    )
    port.dtr = False
    port.rts = False
    port.port = PORT
    port.open()
    port.dtr = False
    port.rts = False
    try:
        time.sleep(0.25)
        port.reset_input_buffer()
        sequence, _ = wait_for_handshake(decoder, port, sequence, 4.0)
        sequence = write_frame(port, DISARM, sequence)
        deadline = time.monotonic() + 20.0
        while True:
            sequence, startup_status = query(decoder, port, STATUS, sequence)
            state, _, _, health, faults, _, _ = struct.unpack(
                "<BBBBIII", startup_status.payload
            )
            if faults & (1 << 5):
                sequence = write_frame(port, CLEAR_FAULT, sequence)
                time.sleep(0.25)
            if state == 2 and health & 0x0F == 0x0F and faults == 0:
                break
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    f"controller feedback did not become healthy: "
                    f"state={state} health=0x{health:02x} faults=0x{faults:08x}"
                )
            time.sleep(0.25)
        sequence = write_frame(
            port, SET_OPERATING_MODE, sequence, bytes((DIRECT_MODE,))
        )
        sequence = write_frame(port, SET_DIRECT_MOTOR, sequence, zero)
        time.sleep(0.75)
        arm_deadline = time.monotonic() + 3.0
        while True:
            sequence = write_frame(port, SET_DIRECT_MOTOR, sequence, zero)
            sequence = write_frame(port, ARM, sequence)
            time.sleep(0.15)
            sequence, status = query(decoder, port, STATUS, sequence)
            state, _, _, _, faults, _, _ = struct.unpack(
                "<BBBBIII", status.payload
            )
            if state in (3, 4):
                armed = require_armed_status(status.payload)
                break
            if faults or time.monotonic() >= arm_deadline:
                raise RuntimeError(
                    f"ESP32 refused ARM after zero acknowledgment wait: "
                    f"state={state} faults=0x{faults:08x}"
                )
        sequence, before_frame = query(decoder, port, ODOMETRY, sequence)
        previous = odometers(before_frame.payload)

        for name, left, right, duration_ms in tests:
            sequence = write_frame(
                port,
                SET_DIRECT_MOTOR,
                sequence,
                encode_direct(left, right, lease_id, max(1000, duration_ms + 500)),
            )
            time.sleep(min(0.4, duration_ms / 2000.0))
            sequence, active_motor = query(decoder, port, MOTOR, sequence)
            sequence, active_odometry = query(decoder, port, ODOMETRY, sequence)
            elapsed_ms = min(400, duration_ms // 2)
            while elapsed_ms < duration_ms:
                sequence = write_frame(
                    port,
                    SET_DIRECT_MOTOR,
                    sequence,
                    encode_direct(
                        left, right, lease_id, max(1000, duration_ms + 500)
                    ),
                )
                wait_ms = min(400, duration_ms - elapsed_ms)
                time.sleep(wait_ms / 1000.0)
                elapsed_ms += wait_ms
            sequence = write_frame(port, SET_DIRECT_MOTOR, sequence, zero)
            time.sleep(0.25)
            sequence, motor = query(decoder, port, MOTOR, sequence)
            sequence, odometry = query(decoder, port, ODOMETRY, sequence)
            current = odometers(odometry.payload)
            results.append(
                {
                    "test": name,
                    "demand": [left, right],
                    "duration_ms": duration_ms,
                    "odometry_before": list(previous),
                    "odometry_after": list(current),
                    "odometry_delta": [
                        current[0] - previous[0],
                        current[1] - previous[1],
                    ],
                    "active_motor": list(active_motor.payload),
                    "active_odometry": list(odometers(active_odometry.payload)),
                    "motor": list(motor.payload),
                }
            )
            previous = current

        sequence = write_frame(port, SET_DIRECT_MOTOR, sequence, zero)
        sequence = write_frame(port, STOP, sequence)
        sequence = write_frame(port, DISARM, sequence)
        time.sleep(0.25)
        sequence, final_status = query(decoder, port, STATUS, sequence)
        print(
            json.dumps(
                {
                    "armed": armed,
                    "results": results,
                    "final_status": list(final_status.payload),
                },
                sort_keys=True,
            )
        )
    finally:
        try:
            sequence = write_frame(port, SET_DIRECT_MOTOR, sequence, zero)
            sequence = write_frame(port, STOP, sequence)
            write_frame(port, DISARM, sequence)
        except Exception:
            pass
        port.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

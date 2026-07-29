#!/usr/bin/env python3
"""Exercise zero-demand ARM, lease expiry, and serial reconnect fail-safes."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(SCRIPT_DIR))

from preflight import open_serial_port
from safe_motion_bringup import Session, establish_zero_and_arm
from trashcan_robot.config import load_config
from trashcan_robot.protocol import (
    ARM,
    DISARM,
    DRIVE_MODE,
    EMERGENCY_STOP,
    MOTOR,
    SET_OPERATING_MODE,
    STATUS,
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(ROOT / "config/robot.yaml"))
    args = parser.parse_args()
    config = load_config(args.config)
    results = {}

    port = open_serial_port(config)
    session = Session(port)
    try:
        time.sleep(0.5)
        port.reset_input_buffer()
        session.handshake()
        session.command(SET_OPERATING_MODE, bytes((DRIVE_MODE,)))
        try:
            session.command(ARM)
        except RuntimeError as exc:
            results["arm_before_zero_rejected"] = str(exc)
        else:
            raise RuntimeError("ARM unexpectedly succeeded before zero acknowledgment")
        session.command(EMERGENCY_STOP)
        session.command(DISARM)

        _lease_id, _zero, armed = establish_zero_and_arm(
            session, config.serial.lease_ms
        )
        # Observe after the 500 ms movement lease but before the independent
        # 750 ms serial-connection watchdog.
        time.sleep(config.serial.lease_ms / 1000.0 + 0.1)
        expired = session.query(STATUS)
        expired_motor = session.query(MOTOR)
        if not expired["faults"] & (1 << 1):
            raise RuntimeError(f"lease expiry did not latch: {expired}")
        if expired_motor["calculated"] != [0, 0] or expired_motor["applied"] != [0, 0]:
            raise RuntimeError(f"lease expiry did not force zero: {expired_motor}")
        results["armed_zero"] = armed
        results["lease_expired"] = expired
        results["lease_expired_motor"] = expired_motor
    finally:
        # Deliberately close without STOP to exercise serial-loss behavior.
        port.close()

    time.sleep(1.1)
    port = open_serial_port(config)
    session = Session(port)
    try:
        time.sleep(0.25)
        port.reset_input_buffer()
        session.handshake()
        reconnect_status = session.query(STATUS)
        reconnect_motor = session.query(MOTOR)
        if (
            reconnect_status["state"] == 4
            or reconnect_status["operating_mode"] != 0
        ):
            raise RuntimeError(f"reconnect restored stale control: {reconnect_status}")
        if reconnect_motor["calculated"] != [0, 0] or reconnect_motor["applied"] != [0, 0]:
            raise RuntimeError(f"reconnect restored stale motion: {reconnect_motor}")
        session.command(EMERGENCY_STOP)
        session.command(DISARM)
        results["reconnect_status"] = reconnect_status
        results["reconnect_motor"] = reconnect_motor
        results["emergency_stop_acknowledged"] = True
    finally:
        port.close()

    print(json.dumps(results, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

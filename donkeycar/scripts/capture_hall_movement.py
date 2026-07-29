#!/usr/bin/env python3
"""Capture manual wheel Hall changes while both motor bridges stay disabled."""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(SCRIPT_DIR))

from preflight import open_serial_port
from safe_motion_bringup import Session
from trashcan_robot.config import load_config
from trashcan_robot.protocol import (
    CONTROLLER_TELEMETRY,
    DISARM,
    STOP,
)


class HallObservationRecorder:
    def __init__(self) -> None:
        self._observations: list[dict] = []
        self._previous_halls: list[int] | None = None

    def observe(self, elapsed_seconds: float, controller: dict) -> None:
        if any(controller["bridges_enabled"]):
            raise RuntimeError(
                f"bridge enabled during manual Hall capture: {controller}"
            )
        halls = [int(value) for value in controller["halls"]]
        if any(value < 1 or value > 6 for value in halls):
            raise RuntimeError(f"invalid Hall state during capture: {halls}")
        if halls != self._previous_halls:
            self._observations.append(
                {
                    "elapsed_seconds": round(elapsed_seconds, 3),
                    "halls": halls,
                    "command_ages_ms": controller["command_ages_ms"],
                    "states": controller.get("states"),
                    "compare_offsets": controller.get("compare_offsets"),
                    "remote_framing_errors": controller.get(
                        "remote_framing_errors"
                    ),
                }
            )
        self._previous_halls = halls

    def summary(self) -> dict:
        changes = [0, 0]
        for previous, current in zip(
            self._observations,
            self._observations[1:],
        ):
            for wheel in range(2):
                if previous["halls"][wheel] != current["halls"][wheel]:
                    changes[wheel] += 1
        return {
            "hall_changes": changes,
            "observations": list(self._observations),
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(ROOT / "config/robot.yaml"))
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--sample-hz", type=float, default=20.0)
    parser.add_argument("--log", type=Path, required=True)
    args = parser.parse_args()
    if not 5.0 <= args.duration <= 60.0:
        raise SystemExit("--duration must be between 5 and 60 seconds")
    if not 1.0 <= args.sample_hz <= 50.0:
        raise SystemExit("--sample-hz must be between 1 and 50")

    config = load_config(args.config)
    recorder = HallObservationRecorder()
    port = open_serial_port(config)
    session = Session(port)
    try:
        time.sleep(0.5)
        port.reset_input_buffer()
        session.handshake()
        session.command(STOP)
        session.command(DISARM)
        initial = session.query(CONTROLLER_TELEMETRY)
        recorder.observe(0.0, initial)
        print("CAPTURE_READY bridges disabled; spin wheels manually", flush=True)
        started = time.monotonic()
        period = 1.0 / args.sample_hz
        while time.monotonic() - started < args.duration:
            controller = session.query(CONTROLLER_TELEMETRY)
            recorder.observe(time.monotonic() - started, controller)
            time.sleep(period)
    finally:
        try:
            session.command(STOP)
            session.command(DISARM)
        except Exception:
            pass
        port.close()

    summary = recorder.summary()
    record = {
        "utc": datetime.now(timezone.utc).isoformat(),
        "duration_seconds": args.duration,
        "sample_hz": args.sample_hz,
        **summary,
    }
    args.log.parent.mkdir(parents=True, exist_ok=True)
    args.log.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n")
    print(json.dumps(record, sort_keys=True), flush=True)
    if any(changes == 0 for changes in summary["hall_changes"]):
        raise RuntimeError(
            f"manual capture missed a wheel: {summary['hall_changes']}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

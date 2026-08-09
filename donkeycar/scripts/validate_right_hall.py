#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from trashcan_robot.config import load_config
from trashcan_robot.right_wheel_hall_test import RightWheelHallTester, RightWheelHallTestPlan
from trashcan_robot.transport import SerialMotorTransport


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate the right Hall channel with an isolated low-demand wheel check")
    parser.add_argument("--config", default=str(ROOT / "config/robot.yaml"))
    parser.add_argument("--demand", type=float, default=0.25)
    parser.add_argument("--duration", type=float, default=1.5)
    parser.add_argument("--minimum-transitions", type=int, default=6)
    parser.add_argument("--confirm-lifted", action="store_true")
    args = parser.parse_args()

    if not args.confirm_lifted:
        parser.error("use --confirm-lifted only after the right wheel is safely off the ground")

    config = load_config(args.config)
    result = RightWheelHallTester(
        SerialMotorTransport(config.serial), config.limits
    ).run(
        RightWheelHallTestPlan(
            demand=args.demand,
            duration_seconds=args.duration,
            minimum_transitions=args.minimum_transitions,
            sample_period_seconds=max(0.02, 1.0 / max(config.serial.command_hz, 1)),
        )
    )

    print(
        "RIGHT_HALL",
        "PASS" if result.passed else "FAIL",
        f"transitions={result.transition_delta}",
        f"peak_tps={result.peak_transitions_per_second:.2f}",
        f"invalid_delta={result.invalid_state_delta}",
        f"skipped_delta={result.skipped_transition_delta}",
        f"state={result.final.state}",
    )
    return 0 if result.passed else 2


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os

from trashcan_robot.config import load_config
from trashcan_robot.pipeline import build_vehicle


def main() -> None:
    parser = argparse.ArgumentParser(description="Trashcan robot Donkeycar runner")
    parser.add_argument("drive", nargs="?", default="drive")
    parser.add_argument("--config", default=os.environ.get("TRASHCAN_CONFIG", "config/robot.yaml"))
    parser.add_argument("--mock", action="store_true")
    parser.add_argument("--model")
    args = parser.parse_args()

    import donkeycar as dk
    donkey_config = dk.load_config()
    config = load_config(args.config)
    if args.model:
        config.raw["model"]["path"] = args.model
        config.raw["model"]["name"] = os.path.basename(args.model)
    vehicle = build_vehicle(config, donkey_config, use_mock=args.mock or config.raw["runtime"]["mock_transport"])
    vehicle.start(rate_hz=config.serial.command_hz, max_loop_count=None)


if __name__ == "__main__":
    main()

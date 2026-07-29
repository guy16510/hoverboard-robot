#!/usr/bin/env python3
"""Exercise actual Donkeycar websocket release and disconnect behavior."""

from __future__ import annotations

import argparse
import asyncio
import json
import time
import urllib.request
from pathlib import Path

from tornado.websocket import websocket_connect


def state(url: str) -> dict:
    with urllib.request.urlopen(url, timeout=2.0) as response:
        return json.load(response)


def wait_for_zero(url: str, timeout: float = 6.0) -> dict:
    deadline = time.monotonic() + timeout
    latest = {}
    while time.monotonic() < deadline:
        latest = state(url)
        motor = latest.get("telemetry", {}).get("motor")
        if (
            latest.get("esp32_connected")
            and latest.get("faults") == []
            and latest.get("throttle") == 0.0
            and latest.get("steering") == 0.0
            and motor
            and motor.get("calculated") == [0, 0]
            and motor.get("applied") == [0, 0]
        ):
            return latest
        time.sleep(0.1)
    raise RuntimeError(f"web control did not return to exact zero: {latest}")


def settled_zero(url: str) -> dict:
    wait_for_zero(url)
    time.sleep(0.5)
    return wait_for_zero(url)


async def command(
    state_url: str,
    websocket_url: str,
    duration: float,
    *,
    explicit_release: bool,
) -> list[dict]:
    connection = await websocket_connect(websocket_url)
    observations = []
    demand = json.dumps(
        {
            "angle": 1.0,
            "throttle": 0.0,
            "drive_mode": "user",
            "recording": False,
        }
    )
    try:
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            connection.write_message(demand)
            await asyncio.sleep(0.1)
            observations.append(await asyncio.to_thread(state, state_url))
        if explicit_release:
            connection.write_message(
                json.dumps(
                    {
                        "angle": 0.0,
                        "throttle": 0.0,
                        "drive_mode": "user",
                        "recording": False,
                    }
                )
            )
            await asyncio.sleep(0.25)
    finally:
        connection.close()
    return observations


def odometry(snapshot: dict) -> tuple[int, int]:
    value = snapshot["telemetry"]["odometry"]
    return int(value["left"]), int(value["right"])


def result(
    label: str,
    before: dict,
    observations: list[dict],
    after: dict,
) -> dict:
    start = odometry(before)
    end = odometry(after)
    delta = [end[0] - start[0], end[1] - start[1]]
    motion = [
        snapshot
        for snapshot in observations
        if snapshot.get("steering", 0.0) > 0.0
        and snapshot.get("telemetry", {})
        .get("drive", {})
        .get("requested_yaw", 0.0)
        > 0.0
    ]
    if not motion:
        raise RuntimeError(f"{label} never reached live ESP32 drive telemetry")
    peak = max(
        motion,
        key=lambda snapshot: max(
            abs(value)
            for value in snapshot["telemetry"]["motor"]["calculated"]
        ),
    )
    peak_motor = peak["telemetry"]["motor"]
    if max(abs(value) for value in peak_motor["calculated"]) > 250:
        raise RuntimeError(f"{label} exceeded the motor hard limit: {peak_motor}")
    if peak_motor["calculated"] != [250, -250]:
        raise RuntimeError(f"{label} did not reach bounded steering demand: {peak_motor}")
    if delta[0] <= 0 or delta[1] >= 0:
        raise RuntimeError(f"{label} steering odometry was unexpected: {delta}")
    motor = after["telemetry"]["motor"]
    if motor["crc_errors"] or motor["acknowledgment_timeouts"]:
        raise RuntimeError(f"{label} transport counters are nonzero: {motor}")
    return {
        "test": label,
        "odometry_before": list(start),
        "odometry_after": list(end),
        "odometry_delta": delta,
        "live_observation_count": len(motion),
        "peak_drive": peak["telemetry"]["drive"],
        "peak_motor": peak_motor,
        "ending_motor": motor,
        "ending_status": after["telemetry"]["status"],
        "ending_faults": after["telemetry"]["faults"],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=0.75)
    parser.add_argument("--log", type=Path, required=True)
    args = parser.parse_args()
    if not 0.1 <= args.duration <= 1.0:
        raise SystemExit("--duration must be between 0.1 and 1.0 seconds")
    state_url = "http://127.0.0.1:8888/api/state"
    websocket_url = "ws://127.0.0.1:8887/wsDrive"

    before_release = settled_zero(state_url)
    release_observations = asyncio.run(
        command(
            state_url,
            websocket_url,
            args.duration,
            explicit_release=True,
        )
    )
    after_release = settled_zero(state_url)

    before_disconnect = after_release
    disconnect_observations = asyncio.run(
        command(
            state_url,
            websocket_url,
            args.duration,
            explicit_release=False,
        )
    )
    after_disconnect = settled_zero(state_url)

    record = {
        "duration_seconds": args.duration,
        "results": [
            result(
                "web-explicit-release",
                before_release,
                release_observations,
                after_release,
            ),
            result(
                "websocket-disconnect",
                before_disconnect,
                disconnect_observations,
                after_disconnect,
            ),
        ],
    }
    args.log.parent.mkdir(parents=True, exist_ok=True)
    args.log.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n")
    print(json.dumps(record, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

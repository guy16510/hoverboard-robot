from __future__ import annotations

import json
import os
import time
from collections.abc import Callable
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import psutil

from .state import RobotState
from .transport import MotorTransport


class JsonRunLogger:
    def __init__(
        self,
        state: RobotState,
        transport: MotorTransport,
        directory: str,
        model_name: str | None,
        telemetry_hz: float = 5.0,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if telemetry_hz <= 0:
            raise ValueError("telemetry_hz must be greater than zero")
        self._state = state
        self._transport = transport
        self._directory = Path(directory)
        self._directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self._path = self._directory / f"run-{stamp}.jsonl"
        self._model_name = model_name
        self._process = psutil.Process(os.getpid())
        self._minimum_interval_seconds = 1.0 / telemetry_hz
        self._clock = clock
        self._last_write_at: float | None = None

    @property
    def path(self) -> Path:
        return self._path

    def run(self, fps: float | None = None, inference_rate: float | None = None) -> str:
        now = self._clock()
        if (
            self._last_write_at is not None
            and now - self._last_write_at < self._minimum_interval_seconds
        ):
            return str(self._path)
        telemetry = [
            {"message_type": frame.message_type, "sequence": frame.sequence, "payload_hex": frame.payload.hex()}
            for frame in self._transport.read_telemetry()
        ]
        snapshot = self._state.snapshot()
        record: dict[str, Any] = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "camera_fps": float(fps or snapshot.get("fps") or 0.0),
            "inference_rate": float(inference_rate or snapshot.get("inference_rate") or 0.0),
            "cpu_percent": psutil.cpu_percent(interval=None),
            "ram_percent": psutil.virtual_memory().percent,
            "process_rss_bytes": self._process.memory_info().rss,
            "serial_latency_ms": snapshot.get("serial_latency_ms"),
            "esp32_heartbeat": snapshot.get("esp32_connected", False),
            "telemetry_packets": telemetry,
            "ultrasonic": snapshot.get("ultrasonic", {}),
            "model_name": self._model_name,
            "mode": snapshot.get("mode"),
            "faults": snapshot.get("faults", []),
        }
        with self._path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record, separators=(",", ":")) + "\n")
        self._last_write_at = now
        return str(self._path)

    def shutdown(self) -> None:
        self._state.update(mode="Stopped")

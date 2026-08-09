from __future__ import annotations

import threading
from copy import deepcopy
from datetime import datetime, timezone
from typing import Any


def empty_hall_state() -> dict[str, Any]:
    return {
        "state": 0,
        "valid": False,
        "moving": False,
        "transitions": 0,
        "transitions_per_second": 0.0,
        "invalid_states": 0,
        "skipped_transitions": 0,
        "last_transition_age_s": None,
    }


class RobotState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._state: dict[str, Any] = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "mode": "Stopped",
            "recording": False,
            "steering": 0.0,
            "throttle": 0.0,
            "esp32_connected": False,
            "fps": 0.0,
            "inference_rate": 0.0,
            "serial_latency_ms": None,
            "ultrasonic": {
                "front_m": None,
                "left_m": None,
                "right_m": None,
            },
            "right_hall": empty_hall_state(),
            "left_hall": empty_hall_state(),
            "telemetry": {},
            "faults": [],
            "model_name": None,
        }

    def update(self, **values: Any) -> None:
        with self._lock:
            self._state.update(values)
            self._state["timestamp"] = datetime.now(timezone.utc).isoformat()

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return deepcopy(self._state)

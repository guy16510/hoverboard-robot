from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class SerialConfig:
    port: str
    baud: int
    timeout_seconds: float
    reconnect_seconds: float
    lease_ms: int
    command_hz: int


@dataclass(frozen=True)
class LimitsConfig:
    max_linear_velocity: float
    max_angular_velocity: float
    throttle_deadband: float
    steering_deadband: float


@dataclass(frozen=True)
class WheelKinematicsConfig:
    wheel_diameter_m: float
    hall_transitions_per_revolution: int

    def validate(self) -> None:
        if self.wheel_diameter_m <= 0:
            raise ValueError("wheel_diameter_m must be greater than zero")
        if self.hall_transitions_per_revolution <= 0:
            raise ValueError("hall_transitions_per_revolution must be greater than zero")


@dataclass(frozen=True)
class AppConfig:
    raw: dict[str, Any]
    serial: SerialConfig
    limits: LimitsConfig
    wheel_kinematics: WheelKinematicsConfig


def load_config(path: str | Path) -> AppConfig:
    source = Path(path)
    with source.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}
    serial = raw["serial"]
    limits = raw["limits"]
    wheel_kinematics = WheelKinematicsConfig(**raw["wheel_kinematics"])
    wheel_kinematics.validate()
    return AppConfig(
        raw=raw,
        serial=SerialConfig(**serial),
        limits=LimitsConfig(**limits),
        wheel_kinematics=wheel_kinematics,
    )

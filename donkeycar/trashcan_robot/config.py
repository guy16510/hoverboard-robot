from __future__ import annotations

import os
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
class AppConfig:
    raw: dict[str, Any]
    serial: SerialConfig
    limits: LimitsConfig


def load_config(path: str | Path) -> AppConfig:
    source = Path(path)
    with source.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}
    serial = dict(raw["serial"])
    configured_port = os.environ.get("TRASHCAN_SERIAL_PORT", "").strip()
    if configured_port:
        serial["port"] = configured_port
    raw["serial"] = serial
    limits = raw["limits"]
    return AppConfig(
        raw=raw,
        serial=SerialConfig(**serial),
        limits=LimitsConfig(**limits),
    )

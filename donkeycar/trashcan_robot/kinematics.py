from __future__ import annotations

import math
from dataclasses import dataclass

from .config import WheelKinematicsConfig
from .protocol import HallReading

MILES_PER_METER = 0.000621371192237334
SECONDS_PER_HOUR = 3600.0


@dataclass(frozen=True)
class WheelMotion:
    rpm: float
    speed_mps: float
    speed_mph: float
    travel_m: float


class HallKinematics:
    """Convert raw Hall transition telemetry into wheel motion magnitudes."""

    def __init__(self, config: WheelKinematicsConfig) -> None:
        config.validate()
        self._transitions_per_revolution = config.hall_transitions_per_revolution
        self._circumference_m = math.pi * config.wheel_diameter_m

    def calculate(self, reading: HallReading) -> WheelMotion:
        revolutions = reading.transitions / self._transitions_per_revolution
        revolutions_per_second = (
            reading.transitions_per_second / self._transitions_per_revolution
        )
        speed_mps = revolutions_per_second * self._circumference_m
        return WheelMotion(
            rpm=revolutions_per_second * 60.0,
            speed_mps=speed_mps,
            speed_mph=speed_mps * MILES_PER_METER * SECONDS_PER_HOUR,
            travel_m=revolutions * self._circumference_m,
        )

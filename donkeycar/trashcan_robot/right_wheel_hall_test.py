from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable

from .config import LimitsConfig
from .protocol import HallReading
from .transport import MotorTransport


@dataclass(frozen=True)
class RightWheelHallTestPlan:
    demand: float = 0.25
    duration_seconds: float = 1.5
    sample_period_seconds: float = 0.05
    sensor_ready_timeout_seconds: float = 1.0
    minimum_transitions: int = 6

    def validate(self) -> None:
        if not 0.05 <= self.demand <= 0.5:
            raise ValueError("right-wheel demand must be between 0.05 and 0.50")
        if self.duration_seconds <= 0:
            raise ValueError("duration_seconds must be positive")
        if self.sample_period_seconds <= 0:
            raise ValueError("sample_period_seconds must be positive")
        if self.sensor_ready_timeout_seconds <= 0:
            raise ValueError("sensor_ready_timeout_seconds must be positive")
        if self.minimum_transitions <= 0:
            raise ValueError("minimum_transitions must be positive")


@dataclass(frozen=True)
class RightWheelHallTestResult:
    passed: bool
    baseline: HallReading
    final: HallReading
    transition_delta: int
    invalid_state_delta: int
    skipped_transition_delta: int
    peak_transitions_per_second: float
    observed_moving: bool
    linear_velocity: float
    angular_velocity: float


def right_wheel_only_command(limits: LimitsConfig, demand: float) -> tuple[float, float]:
    """Return robot linear/yaw commands that produce left=0, right=+demand.

    Differential mixing is left = linear + yaw and right = linear - yaw in
    robot coordinates. Setting linear=demand/2 and yaw=-demand/2 therefore
    leaves the left wheel at zero while commanding only the right wheel forward.
    The firmware's per-motor sign constants handle physical motor orientation.
    """
    if not 0.0 < demand <= 1.0:
        raise ValueError("demand must be in (0, 1]")
    half = demand / 2.0
    return (
        half * limits.max_linear_velocity,
        -half * limits.max_angular_velocity,
    )


def counter_delta(before: int, after: int) -> int:
    return (after - before) & 0xFFFFFFFF


class RightWheelHallTester:
    def __init__(
        self,
        transport: MotorTransport,
        limits: LimitsConfig,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self._transport = transport
        self._limits = limits
        self._clock = clock
        self._sleep = sleep

    def run(self, plan: RightWheelHallTestPlan) -> RightWheelHallTestResult:
        plan.validate()
        linear_velocity, angular_velocity = right_wheel_only_command(
            self._limits, plan.demand
        )

        self._transport.connect()
        baseline: HallReading | None = None
        final: HallReading | None = None
        observed_moving = False
        peak_tps = 0.0
        try:
            ready_deadline = self._clock() + plan.sensor_ready_timeout_seconds
            while self._clock() < ready_deadline:
                self._transport.read_telemetry()
                candidate = self._transport.latest_hall()
                if candidate.valid:
                    baseline = candidate
                    break
                self._sleep(plan.sample_period_seconds)
            if baseline is None:
                raise RuntimeError(
                    "right Hall sensor never reported a valid state; do not move the wheel"
                )

            final = baseline
            motion_deadline = self._clock() + plan.duration_seconds
            while self._clock() < motion_deadline:
                self._transport.send_command(linear_velocity, angular_velocity)
                self._transport.read_telemetry()
                sample = self._transport.latest_hall()
                if sample.valid:
                    final = sample
                    observed_moving = observed_moving or sample.moving
                    peak_tps = max(peak_tps, sample.transitions_per_second)
                self._sleep(plan.sample_period_seconds)

            # Explicit zero before disconnect; disconnect adds STOP + DISARM as a
            # second independent safety layer.
            self._transport.send_command(0.0, 0.0)
            self._transport.read_telemetry()
        finally:
            self._transport.disconnect()

        assert baseline is not None
        assert final is not None
        transition_delta = counter_delta(baseline.transitions, final.transitions)
        invalid_delta = counter_delta(baseline.invalid_states, final.invalid_states)
        skipped_delta = counter_delta(
            baseline.skipped_transitions, final.skipped_transitions
        )
        passed = (
            final.valid
            and observed_moving
            and transition_delta >= plan.minimum_transitions
            and invalid_delta == 0
            and skipped_delta == 0
        )
        return RightWheelHallTestResult(
            passed=passed,
            baseline=baseline,
            final=final,
            transition_delta=transition_delta,
            invalid_state_delta=invalid_delta,
            skipped_transition_delta=skipped_delta,
            peak_transitions_per_second=peak_tps,
            observed_moving=observed_moving,
            linear_velocity=linear_velocity,
            angular_velocity=angular_velocity,
        )

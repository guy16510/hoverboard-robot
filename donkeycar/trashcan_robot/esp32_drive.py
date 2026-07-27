from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable

from .config import LimitsConfig
from .transport import MotorTransport


@dataclass(frozen=True)
class DriveStatus:
    connected: bool
    serial_latency_ms: float | None
    linear_velocity: float
    angular_velocity: float
    fault: str | None


class ESP32Drive:
    """Donkeycar drivetrain Part translating normalized pilot output to velocity/yaw."""

    def __init__(
        self,
        transport: MotorTransport,
        limits: LimitsConfig,
        reconnect_seconds: float = 1.0,
        connection_change: Callable[[bool], None] | None = None,
    ) -> None:
        self._transport = transport
        self._limits = limits
        self._reconnect_seconds = reconnect_seconds
        self._connection_change = connection_change
        self._last_connect_attempt = 0.0
        self._last_connected = False
        self.status = DriveStatus(False, None, 0.0, 0.0, None)

    def run(self, throttle: float | None, steering: float | None) -> tuple[bool, float, float, float | None, str | None]:
        throttle = self._deadband(float(throttle or 0.0), self._limits.throttle_deadband)
        steering = self._deadband(float(steering or 0.0), self._limits.steering_deadband)
        linear = self._clamp(throttle) * self._limits.max_linear_velocity
        angular = self._clamp(steering) * self._limits.max_angular_velocity

        try:
            self._ensure_connected()
            latency = self._transport.send_command(linear, angular)
            self.status = DriveStatus(True, latency, linear, angular, None)
        except Exception as exc:
            self._safe_disconnect()
            linear = 0.0
            angular = 0.0
            self.status = DriveStatus(False, None, linear, angular, str(exc))

        self._publish_connection(self.status.connected)
        return (
            self.status.connected,
            self.status.linear_velocity,
            self.status.angular_velocity,
            self.status.serial_latency_ms,
            self.status.fault,
        )

    def shutdown(self) -> None:
        try:
            if self._transport.is_connected():
                self._transport.send_command(0.0, 0.0)
        finally:
            self._transport.disconnect()

    def _ensure_connected(self) -> None:
        if self._transport.is_connected():
            return
        now = time.monotonic()
        if now - self._last_connect_attempt < self._reconnect_seconds:
            raise ConnectionError("waiting to retry ESP32 connection")
        self._last_connect_attempt = now
        self._transport.connect()

    def _safe_disconnect(self) -> None:
        try:
            if self._transport.is_connected():
                self._transport.send_command(0.0, 0.0)
        except Exception:
            pass
        self._transport.disconnect()

    def _publish_connection(self, connected: bool) -> None:
        if connected != self._last_connected and self._connection_change is not None:
            self._connection_change(connected)
        self._last_connected = connected

    @staticmethod
    def _clamp(value: float) -> float:
        return max(-1.0, min(1.0, value))

    @staticmethod
    def _deadband(value: float, deadband: float) -> float:
        return 0.0 if abs(value) < deadband else value

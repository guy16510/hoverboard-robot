from __future__ import annotations

import abc
import random
import threading
import time
from dataclasses import dataclass, field
from typing import Any

import serial

from .config import SerialConfig
from .protocol import Frame, FrameDecoder, HELLO, SET_VELOCITY_YAW, STOP, encode_frame, encode_motion


class MotorTransport(abc.ABC):
    @abc.abstractmethod
    def connect(self) -> None: ...

    @abc.abstractmethod
    def disconnect(self) -> None: ...

    @abc.abstractmethod
    def send_command(self, linear_velocity: float, angular_velocity: float) -> float: ...

    @abc.abstractmethod
    def read_telemetry(self) -> list[Frame]: ...

    @abc.abstractmethod
    def is_connected(self) -> bool: ...


class SerialMotorTransport(MotorTransport):
    def __init__(self, config: SerialConfig) -> None:
        self._config = config
        self._serial: serial.Serial | None = None
        self._decoder = FrameDecoder()
        self._sequence = 0
        self._lease_id = random.getrandbits(32)
        self._lock = threading.Lock()

    def connect(self) -> None:
        with self._lock:
            if self.is_connected():
                return
            self._serial = serial.Serial(
                self._config.port,
                self._config.baud,
                timeout=self._config.timeout_seconds,
                write_timeout=self._config.timeout_seconds,
            )
            self._serial.reset_input_buffer()
            self._write(HELLO, b"")

    def disconnect(self) -> None:
        with self._lock:
            if self._serial is not None:
                try:
                    self._write(STOP, b"")
                except Exception:
                    pass
                self._serial.close()
            self._serial = None

    def send_command(self, linear_velocity: float, angular_velocity: float) -> float:
        started = time.monotonic_ns()
        with self._lock:
            if not self.is_connected():
                raise ConnectionError("ESP32 serial transport is disconnected")
            payload = encode_motion(linear_velocity, angular_velocity, self._lease_id, self._config.lease_ms)
            self._write(SET_VELOCITY_YAW, payload)
        return (time.monotonic_ns() - started) / 1_000_000.0

    def read_telemetry(self) -> list[Frame]:
        with self._lock:
            if not self.is_connected() or self._serial is None:
                return []
            waiting = self._serial.in_waiting
            data = self._serial.read(waiting or 1)
        return self._decoder.feed(data)

    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def _write(self, message_type: int, payload: bytes) -> None:
        assert self._serial is not None
        self._serial.write(encode_frame(message_type, self._sequence, payload))
        self._serial.flush()
        self._sequence = (self._sequence + 1) & 0xFFFF


@dataclass
class MockMotorTransport(MotorTransport):
    connected: bool = False
    commands: list[dict[str, Any]] = field(default_factory=list)
    telemetry: list[Frame] = field(default_factory=list)

    def connect(self) -> None:
        self.connected = True

    def disconnect(self) -> None:
        self.connected = False

    def send_command(self, linear_velocity: float, angular_velocity: float) -> float:
        if not self.connected:
            raise ConnectionError("mock transport disconnected")
        self.commands.append({
            "linear_velocity": linear_velocity,
            "angular_velocity": angular_velocity,
            "timestamp": time.time(),
        })
        return 0.1

    def read_telemetry(self) -> list[Frame]:
        frames = list(self.telemetry)
        self.telemetry.clear()
        return frames

    def is_connected(self) -> bool:
        return self.connected

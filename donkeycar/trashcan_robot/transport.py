from __future__ import annotations

import abc
import glob
import random
import threading
import time
from dataclasses import dataclass, field
from typing import Any

import serial

from .config import SerialConfig
from .protocol import (
    ARM,
    DISARM,
    DRIVE_MODE,
    HELLO,
    SET_OPERATING_MODE,
    SET_VELOCITY_YAW,
    STOP,
    ULTRASONIC,
    Frame,
    FrameDecoder,
    UltrasonicReading,
    decode_ultrasonic,
    encode_frame,
    encode_motion,
)


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
    def latest_ultrasonic(self) -> UltrasonicReading: ...

    @abc.abstractmethod
    def is_connected(self) -> bool: ...


class SerialMotorTransport(MotorTransport):
    _ARM_RETRY_COUNT = 20
    _ARM_RETRY_INTERVAL_SECONDS = 0.1

    def __init__(self, config: SerialConfig) -> None:
        self._config = config
        self._serial: serial.Serial | None = None
        self._decoder = FrameDecoder()
        self._sequence = 0
        self._lease_id = random.getrandbits(32)
        self._lock = threading.Lock()
        self._telemetry_queue: list[Frame] = []
        self._latest_ultrasonic = UltrasonicReading(None, None, None)

    def connect(self) -> None:
        with self._lock:
            if self.is_connected():
                return
            port = self._resolve_port(self._config.port)
            self._serial = serial.Serial(
                port,
                self._config.baud,
                timeout=self._config.timeout_seconds,
                write_timeout=self._config.timeout_seconds,
            )
            self._serial.reset_input_buffer()
            self._decoder = FrameDecoder()
            self._telemetry_queue.clear()
            self._latest_ultrasonic = UltrasonicReading(None, None, None)
            self._write(HELLO, b"")
            self._write(SET_OPERATING_MODE, bytes((DRIVE_MODE,)))

            # The ESP32 requires a current lease before it accepts ARM. Sending
            # zero demand before each retry guarantees startup never creates motion.
            for _ in range(self._ARM_RETRY_COUNT):
                zero_demand = encode_motion(
                    0.0,
                    0.0,
                    self._lease_id,
                    self._config.lease_ms,
                )
                self._write(SET_VELOCITY_YAW, zero_demand)
                self._write(ARM, b"")
                self._drain()
                time.sleep(self._ARM_RETRY_INTERVAL_SECONDS)

    def disconnect(self) -> None:
        with self._lock:
            if self._serial is not None:
                try:
                    self._write(STOP, b"")
                    self._write(DISARM, b"")
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
            self._drain()
        return (time.monotonic_ns() - started) / 1_000_000.0

    def read_telemetry(self) -> list[Frame]:
        with self._lock:
            if not self.is_connected() or self._serial is None:
                return []
            self._drain()
            frames = list(self._telemetry_queue)
            self._telemetry_queue.clear()
            return frames

    def latest_ultrasonic(self) -> UltrasonicReading:
        with self._lock:
            return self._latest_ultrasonic

    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    @staticmethod
    def _resolve_port(configured_port: str) -> str:
        if configured_port != "auto":
            return configured_port
        candidates: list[str] = []
        for pattern in (
            "/dev/serial/by-id/*",
            "/dev/ttyUSB*",
            "/dev/ttyACM*",
        ):
            candidates.extend(sorted(glob.glob(pattern)))
        if not candidates:
            raise ConnectionError("no ESP32 serial device found under /dev/serial/by-id, /dev/ttyUSB*, or /dev/ttyACM*")
        return candidates[0]

    def _drain(self) -> None:
        if self._serial is None or not self._serial.is_open:
            return
        waiting = self._serial.in_waiting
        if waiting <= 0:
            return
        frames = self._decoder.feed(self._serial.read(waiting))
        for frame in frames:
            if frame.message_type == ULTRASONIC:
                try:
                    self._latest_ultrasonic = decode_ultrasonic(frame.payload)
                except ValueError:
                    pass
        self._telemetry_queue.extend(frames)

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
    ultrasonic: UltrasonicReading = field(default_factory=lambda: UltrasonicReading(None, None, None))

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

    def latest_ultrasonic(self) -> UltrasonicReading:
        return self.ultrasonic

    def is_connected(self) -> bool:
        return self.connected

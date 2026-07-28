from __future__ import annotations

import abc
import random
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable

try:
    import serial as pyserial
except ImportError:  # Tests can inject a serial factory without pyserial installed.
    pyserial = None

from .config import SerialConfig
from .protocol import (
    ACK,
    ARM,
    DISARM,
    DRIVE_MODE,
    ERROR,
    HELLO,
    SET_OPERATING_MODE,
    SET_VELOCITY_YAW,
    STOP,
    Frame,
    FrameDecoder,
    decode_message,
    encode_frame,
    encode_motion,
)


class MotorTransport(abc.ABC):
    @abc.abstractmethod
    def connect(self) -> None: ...

    @abc.abstractmethod
    def disconnect(self) -> None: ...

    @abc.abstractmethod
    def shutdown(self) -> None: ...

    @abc.abstractmethod
    def send_command(self, linear_velocity: float, angular_velocity: float) -> float: ...

    @abc.abstractmethod
    def read_telemetry(self) -> list[Frame]: ...

    @abc.abstractmethod
    def is_connected(self) -> bool: ...


class SerialMotorTransport(MotorTransport):
    _ARM_RETRY_COUNT = 20
    _ARM_RETRY_INTERVAL_SECONDS = 0.1

    def __init__(
        self,
        config: SerialConfig,
        *,
        serial_factory: Callable[..., Any] | None = None,
        clock: Callable[[], float] = time.monotonic,
        sleeper: Callable[[float], None] = time.sleep,
        lease_id_factory: Callable[[], int] = lambda: random.getrandbits(32),
    ) -> None:
        self._config = config
        if serial_factory is None:
            if pyserial is None:
                raise RuntimeError("pyserial is required unless serial_factory is injected")
            serial_factory = pyserial.Serial
        self._serial_factory = serial_factory
        self._clock = clock
        self._sleep = sleeper
        self._lease_id_factory = lease_id_factory
        self._serial: Any | None = None
        self._decoder = FrameDecoder()
        self._sequence = 0
        self._lease_id = 0
        self._lock = threading.RLock()
        self._pending_telemetry: list[Frame] = []
        self._write_history: list[int] = []

    @property
    def write_history(self) -> tuple[int, ...]:
        return tuple(self._write_history)

    def connect(self) -> None:
        with self._lock:
            if self.is_connected():
                return
            endpoint = self._serial_factory(
                self._config.port,
                self._config.baud,
                timeout=self._config.timeout_seconds,
                write_timeout=self._config.timeout_seconds,
            )
            self._serial = endpoint
            self._decoder = FrameDecoder()
            self._sequence = 0
            self._lease_id = self._lease_id_factory() & 0xFFFFFFFF
            self._pending_telemetry.clear()
            self._write_history.clear()
            try:
                self._serial.reset_input_buffer()
                self._write(HELLO, b"")
                self._write(SET_OPERATING_MODE, bytes((DRIVE_MODE,)))
                armed = False
                for _ in range(self._ARM_RETRY_COUNT):
                    self._write(SET_VELOCITY_YAW, self._zero_payload())
                    arm_sequence = self._write(ARM, b"")
                    if self._wait_for_ack(arm_sequence, ARM):
                        armed = True
                        break
                    self._sleep(self._ARM_RETRY_INTERVAL_SECONDS)
                if not armed:
                    raise ConnectionError("ESP32 rejected ARM during safe startup handshake")
            except Exception:
                self._close_endpoint()
                raise

    def disconnect(self) -> None:
        self.shutdown()

    def shutdown(self) -> None:
        with self._lock:
            if self._serial is None:
                return
            for message_type, payload in (
                (SET_VELOCITY_YAW, self._zero_payload()),
                (STOP, b""),
                (DISARM, b""),
            ):
                try:
                    self._write(message_type, payload)
                except Exception:
                    pass
            self._close_endpoint()

    def send_command(self, linear_velocity: float, angular_velocity: float) -> float:
        started = self._clock()
        with self._lock:
            if not self.is_connected():
                raise ConnectionError("ESP32 serial transport is disconnected")
            payload = encode_motion(
                linear_velocity,
                angular_velocity,
                self._lease_id,
                self._config.lease_ms,
            )
            self._write(SET_VELOCITY_YAW, payload)
        return (self._clock() - started) * 1000.0

    def read_telemetry(self) -> list[Frame]:
        with self._lock:
            frames = list(self._pending_telemetry)
            self._pending_telemetry.clear()
            if not self.is_connected() or self._serial is None:
                return frames
            waiting = int(getattr(self._serial, "in_waiting", 0))
            if waiting:
                frames.extend(self._decoder.feed(self._serial.read(waiting)))
            return frames

    def is_connected(self) -> bool:
        return self._serial is not None and bool(getattr(self._serial, "is_open", False))

    def _zero_payload(self) -> bytes:
        return encode_motion(0.0, 0.0, self._lease_id, self._config.lease_ms)

    def _write(self, message_type: int, payload: bytes) -> int:
        if self._serial is None:
            raise ConnectionError("ESP32 serial endpoint is not open")
        sequence = self._sequence
        self._serial.write(encode_frame(message_type, sequence, payload))
        self._serial.flush()
        self._write_history.append(message_type)
        self._sequence = (sequence + 1) & 0xFFFF
        return sequence

    def _wait_for_ack(self, sequence: int, request_type: int) -> bool:
        if self._serial is None:
            return False
        deadline = self._clock() + self._config.timeout_seconds
        while self._clock() <= deadline:
            waiting = int(getattr(self._serial, "in_waiting", 0))
            if waiting:
                for frame in self._decoder.feed(self._serial.read(waiting)):
                    decoded = decode_message(frame)
                    if frame.sequence == sequence and frame.message_type == ACK:
                        if decoded.get("request_type") == request_type and decoded.get("status") == 0:
                            return True
                    if frame.sequence == sequence and frame.message_type == ERROR:
                        return False
                    self._pending_telemetry.append(frame)
            self._sleep(min(0.001, self._config.timeout_seconds))
        return False

    def _close_endpoint(self) -> None:
        endpoint = self._serial
        self._serial = None
        if endpoint is not None:
            try:
                endpoint.close()
            except Exception:
                pass


@dataclass
class MockMotorTransport(MotorTransport):
    connected: bool = False
    commands: list[dict[str, Any]] = field(default_factory=list)
    telemetry: list[Frame] = field(default_factory=list)
    control_events: list[str] = field(default_factory=list)

    def connect(self) -> None:
        self.connected = True
        self.control_events.extend(["HELLO", "MODE_2", "ZERO", "ARM"])

    def disconnect(self) -> None:
        self.shutdown()

    def shutdown(self) -> None:
        if self.connected:
            self.commands.append({"linear_velocity": 0.0, "angular_velocity": 0.0, "timestamp": time.time()})
            self.control_events.extend(["ZERO", "STOP", "DISARM"])
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

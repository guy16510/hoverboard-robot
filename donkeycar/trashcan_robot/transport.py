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
    ACK,
    ARM,
    CAPABILITIES,
    DISARM,
    DRIVE_MODE,
    ERROR,
    HALL,
    HELLO,
    LEFT_HALL,
    MAX_PAYLOAD,
    SET_OPERATING_MODE,
    SET_VELOCITY_YAW,
    STOP,
    ULTRASONIC,
    VERSION,
    Frame,
    FrameDecoder,
    HallReading,
    UltrasonicReading,
    decode_ack,
    decode_capabilities,
    decode_error,
    decode_hall,
    decode_ultrasonic,
    empty_hall_reading,
    encode_frame,
    encode_motion,
)


class ProtocolError(ConnectionError):
    pass


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
    def latest_hall(self) -> HallReading: ...

    @abc.abstractmethod
    def latest_left_hall(self) -> HallReading: ...

    @abc.abstractmethod
    def is_connected(self) -> bool: ...


class SerialMotorTransport(MotorTransport):
    _HANDSHAKE_ATTEMPTS = 5
    _HANDSHAKE_RETRY_SECONDS = 0.05
    _ULTRASONIC_STALE_SECONDS = 0.5
    _HALL_STALE_SECONDS = 0.5

    def __init__(self, config: SerialConfig) -> None:
        self._config = config
        self._serial: serial.Serial | None = None
        self._decoder = FrameDecoder()
        self._sequence = 0
        self._lease_id = random.getrandbits(32)
        self._lock = threading.Lock()
        self._telemetry_queue: list[Frame] = []
        self._latest_ultrasonic = UltrasonicReading(None, None, None)
        self._ultrasonic_updated_at: float | None = None
        self._latest_hall = empty_hall_reading()
        self._hall_updated_at: float | None = None
        self._latest_left_hall = empty_hall_reading()
        self._left_hall_updated_at: float | None = None
        self._ready = False

    def connect(self) -> None:
        with self._lock:
            if self.is_connected():
                return
            self._open_serial()
            last_error: Exception | None = None
            for attempt in range(self._HANDSHAKE_ATTEMPTS):
                try:
                    self._handshake()
                    self._ready = True
                    return
                except (TimeoutError, ProtocolError, ValueError) as exc:
                    last_error = exc
                    self._ready = False
                    if attempt + 1 < self._HANDSHAKE_ATTEMPTS:
                        time.sleep(self._HANDSHAKE_RETRY_SECONDS)
            self._close_serial()
            raise ConnectionError(f"ESP32 handshake failed: {last_error}")

    def disconnect(self) -> None:
        with self._lock:
            if self._serial_open():
                try:
                    self._write(STOP, b"")
                    self._write(DISARM, b"")
                except Exception:
                    pass
            self._ready = False
            self._close_serial()

    def send_command(self, linear_velocity: float, angular_velocity: float) -> float:
        started = time.monotonic_ns()
        with self._lock:
            if not self.is_connected():
                raise ConnectionError("ESP32 serial transport is not handshaken and armed")
            payload = encode_motion(
                linear_velocity,
                angular_velocity,
                self._lease_id,
                self._config.lease_ms,
            )
            self._request(SET_VELOCITY_YAW, payload, ACK)
        return (time.monotonic_ns() - started) / 1_000_000.0

    def read_telemetry(self) -> list[Frame]:
        with self._lock:
            if not self._serial_open():
                return []
            self._drain()
            frames = list(self._telemetry_queue)
            self._telemetry_queue.clear()
            return frames

    def latest_ultrasonic(self) -> UltrasonicReading:
        with self._lock:
            if self._ultrasonic_updated_at is None:
                return UltrasonicReading(None, None, None)
            if time.monotonic() - self._ultrasonic_updated_at > self._ULTRASONIC_STALE_SECONDS:
                return UltrasonicReading(None, None, None)
            return self._latest_ultrasonic

    def latest_hall(self) -> HallReading:
        with self._lock:
            return self._fresh_hall(self._latest_hall, self._hall_updated_at)

    def latest_left_hall(self) -> HallReading:
        with self._lock:
            return self._fresh_hall(self._latest_left_hall, self._left_hall_updated_at)

    def is_connected(self) -> bool:
        return self._ready and self._serial_open()

    def _fresh_hall(self, reading: HallReading, updated_at: float | None) -> HallReading:
        if updated_at is None:
            return empty_hall_reading()
        if time.monotonic() - updated_at > self._HALL_STALE_SECONDS:
            return empty_hall_reading()
        return reading

    def _open_serial(self) -> None:
        self._close_serial()
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
        self._ultrasonic_updated_at = None
        self._latest_hall = empty_hall_reading()
        self._hall_updated_at = None
        self._latest_left_hall = empty_hall_reading()
        self._left_hall_updated_at = None
        self._ready = False

    def _close_serial(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            finally:
                self._serial = None
        self._ready = False

    def _handshake(self) -> None:
        capabilities_frame = self._request(HELLO, b"", CAPABILITIES)
        capabilities = decode_capabilities(capabilities_frame.payload)
        if capabilities.protocol_version != VERSION:
            raise ProtocolError(
                f"ESP32 protocol version {capabilities.protocol_version} != Pi version {VERSION}"
            )
        if capabilities.maximum_payload != MAX_PAYLOAD:
            raise ProtocolError(
                f"ESP32 max payload {capabilities.maximum_payload} != Pi max payload {MAX_PAYLOAD}"
            )
        if not capabilities.supports_mode(DRIVE_MODE):
            raise ProtocolError("ESP32 does not advertise Donkeycar drive mode")

        self._request(SET_OPERATING_MODE, bytes((DRIVE_MODE,)), ACK)
        zero_demand = encode_motion(
            0.0,
            0.0,
            self._lease_id,
            self._config.lease_ms,
        )
        self._request(SET_VELOCITY_YAW, zero_demand, ACK)
        self._request(ARM, b"", ACK)

    def _request(self, message_type: int, payload: bytes, expected_type: int) -> Frame:
        sequence = self._write(message_type, payload)
        deadline = time.monotonic() + max(self._config.timeout_seconds, 0.05)
        while time.monotonic() < deadline:
            for frame in self._read_available():
                if frame.sequence == sequence and frame.message_type == ERROR:
                    error = decode_error(frame.payload)
                    raise ProtocolError(
                        f"ESP32 rejected 0x{error.request_type:02x}: "
                        f"code={error.code} detail={error.detail}"
                    )
                if frame.sequence == sequence and frame.message_type == expected_type:
                    if expected_type == ACK:
                        ack = decode_ack(frame.payload)
                        if ack.request_type != message_type or ack.status != 0:
                            raise ProtocolError(
                                f"invalid ACK for 0x{message_type:02x}: "
                                f"request=0x{ack.request_type:02x} status={ack.status}"
                            )
                    return frame
                self._queue_telemetry(frame)
            time.sleep(0.001)
        raise TimeoutError(
            f"timeout waiting for ESP32 response to 0x{message_type:02x} sequence={sequence}"
        )

    def _read_available(self) -> list[Frame]:
        if not self._serial_open() or self._serial is None:
            return []
        waiting = self._serial.in_waiting
        if waiting <= 0:
            return []
        return self._decoder.feed(self._serial.read(waiting))

    def _drain(self) -> None:
        for frame in self._read_available():
            self._queue_telemetry(frame)

    def _queue_telemetry(self, frame: Frame) -> None:
        if frame.message_type == ULTRASONIC:
            try:
                self._latest_ultrasonic = decode_ultrasonic(frame.payload)
                self._ultrasonic_updated_at = time.monotonic()
            except ValueError:
                return
        elif frame.message_type == HALL:
            try:
                self._latest_hall = decode_hall(frame.payload)
                self._hall_updated_at = time.monotonic()
            except ValueError:
                return
        elif frame.message_type == LEFT_HALL:
            try:
                self._latest_left_hall = decode_hall(frame.payload)
                self._left_hall_updated_at = time.monotonic()
            except ValueError:
                return
        self._telemetry_queue.append(frame)

    def _write(self, message_type: int, payload: bytes) -> int:
        if self._serial is None or not self._serial.is_open:
            raise ConnectionError("ESP32 serial port is closed")
        sequence = self._sequence
        self._serial.write(encode_frame(message_type, sequence, payload))
        self._serial.flush()
        self._sequence = (self._sequence + 1) & 0xFFFF
        return sequence

    def _serial_open(self) -> bool:
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
            raise ConnectionError(
                "no ESP32 serial device found under /dev/serial/by-id, /dev/ttyUSB*, or /dev/ttyACM*"
            )
        return candidates[0]


@dataclass
class MockMotorTransport(MotorTransport):
    connected: bool = False
    commands: list[dict[str, Any]] = field(default_factory=list)
    telemetry: list[Frame] = field(default_factory=list)
    ultrasonic: UltrasonicReading = field(default_factory=lambda: UltrasonicReading(None, None, None))
    hall: HallReading = field(default_factory=empty_hall_reading)
    left_hall: HallReading = field(default_factory=empty_hall_reading)

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

    def latest_hall(self) -> HallReading:
        return self.hall

    def latest_left_hall(self) -> HallReading:
        return self.left_hall

    def is_connected(self) -> bool:
        return self.connected

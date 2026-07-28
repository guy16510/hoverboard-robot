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
    CLEAR_FAULT,
    DISARM,
    DRIVE_MODE,
    ERROR,
    HELLO,
    SET_OPERATING_MODE,
    SET_VELOCITY_YAW,
    STATUS,
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
    _STARTUP_RETRY_COUNT = 80
    _STARTUP_RETRY_INTERVAL_SECONDS = 0.05
    _PREARM_HEALTH_FLAGS = 0x6D
    _RENEWAL_FRESHNESS_FRACTION = 0.5

    def __init__(
        self,
        config: SerialConfig,
        *,
        serial_factory: Callable[..., Any] | None = None,
        clock: Callable[[], float] = time.monotonic,
        sleeper: Callable[[float], None] = time.sleep,
        lease_id_factory: Callable[[], int] = lambda: random.getrandbits(32),
        renew_commands: bool = False,
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
        self._session_valid = False
        self._renew_commands = renew_commands
        self._renew_stop = threading.Event()
        self._renew_thread: threading.Thread | None = None
        self._latest_motion: tuple[float, float] | None = None
        self._latest_motion_at = float("-inf")
        self._last_motion_write_at = float("-inf")

    @property
    def write_history(self) -> tuple[int, ...]:
        return tuple(self._write_history)

    def connect(self) -> None:
        with self._lock:
            if self.is_connected():
                return
        self._stop_command_renewer()
        start_renewer = False
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
            self._session_valid = False
            try:
                self._serial.reset_input_buffer()
                self._write(HELLO, b"")
                mode_sequence = self._write(
                    SET_OPERATING_MODE, bytes((DRIVE_MODE,))
                )
                if not self._wait_for_ack(mode_sequence, SET_OPERATING_MODE):
                    raise ConnectionError("ESP32 rejected drive operating mode")
                zero_sequence = self._write(
                    SET_VELOCITY_YAW, self._zero_payload()
                )
                if not self._wait_for_ack(zero_sequence, SET_VELOCITY_YAW):
                    raise ConnectionError("ESP32 rejected initial zero demand")
                clear_sequence = self._write(CLEAR_FAULT, b"")
                if not self._wait_for_ack(clear_sequence, CLEAR_FAULT):
                    raise ConnectionError("ESP32 rejected controller fault clear")
                ready = False
                for _ in range(self._STARTUP_RETRY_COUNT):
                    zero_sequence = self._write(
                        SET_VELOCITY_YAW, self._zero_payload()
                    )
                    if not self._wait_for_ack(
                        zero_sequence, SET_VELOCITY_YAW
                    ):
                        self._sleep(self._STARTUP_RETRY_INTERVAL_SECONDS)
                        continue
                    status_sequence = self._write(STATUS, b"")
                    status = self._wait_for_status(status_sequence)
                    if status is not None and self._prearm_ready(status):
                        ready = True
                        break
                    self._sleep(self._STARTUP_RETRY_INTERVAL_SECONDS)
                if not ready:
                    raise ConnectionError(
                        "ESP32 did not acknowledge an exact healthy zero session"
                    )
                zero_sequence = self._write(
                    SET_VELOCITY_YAW, self._zero_payload()
                )
                if not self._wait_for_ack(zero_sequence, SET_VELOCITY_YAW):
                    raise ConnectionError("ESP32 rejected final zero demand")
                arm_sequence = self._write(ARM, b"")
                if not self._wait_for_ack(arm_sequence, ARM):
                    raise ConnectionError(
                        "ESP32 rejected ARM after healthy zero acknowledgment"
                    )
                self._session_valid = True
                now = self._clock()
                self._latest_motion = (0.0, 0.0)
                self._latest_motion_at = now
                self._last_motion_write_at = now
                start_renewer = self._renew_commands
            except Exception:
                self._close_endpoint()
                raise
        if start_renewer:
            self._start_command_renewer()

    def disconnect(self) -> None:
        self.shutdown()

    def shutdown(self) -> None:
        self._stop_command_renewer()
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
            now = self._clock()
            self._latest_motion = (linear_velocity, angular_velocity)
            self._latest_motion_at = now
            self._last_motion_write_at = now
        return (self._clock() - started) * 1000.0

    def read_telemetry(self) -> list[Frame]:
        with self._lock:
            frames = list(self._pending_telemetry)
            self._pending_telemetry.clear()
            if not self.is_connected() or self._serial is None:
                return frames
            waiting = int(getattr(self._serial, "in_waiting", 0))
            if waiting:
                received = self._decoder.feed(self._serial.read(waiting))
                frames.extend(received)
                decoded = [decode_message(frame) for frame in received]
                if any(frame.message_type == ERROR for frame in received) or any(
                    (
                        message.get("name") == "status"
                        and int(message.get("faults", 0)) != 0
                    )
                    or (
                        message.get("name") == "drive"
                        and int(message.get("safety_faults", 0)) != 0
                    )
                    for message in decoded
                ):
                    self._session_valid = False
            return frames

    def is_connected(self) -> bool:
        return (
            self._session_valid
            and self._serial is not None
            and bool(getattr(self._serial, "is_open", False))
        )

    def _zero_payload(self) -> bytes:
        return encode_motion(0.0, 0.0, self._lease_id, self._config.lease_ms)

    def _renew_command_once(self) -> bool:
        with self._lock:
            if not self.is_connected() or self._latest_motion is None:
                return False
            now = self._clock()
            period = 1.0 / max(1, self._config.command_hz)
            freshness = (
                self._config.lease_ms
                / 1000.0
                * self._RENEWAL_FRESHNESS_FRACTION
            )
            if (
                now - self._latest_motion_at > freshness
                or now - self._last_motion_write_at < period
            ):
                return False
            linear_velocity, angular_velocity = self._latest_motion
            payload = encode_motion(
                linear_velocity,
                angular_velocity,
                self._lease_id,
                self._config.lease_ms,
            )
            try:
                self._write(SET_VELOCITY_YAW, payload)
            except Exception:
                self._session_valid = False
                return False
            self._last_motion_write_at = now
            return True

    def _start_command_renewer(self) -> None:
        if self._renew_thread is not None and self._renew_thread.is_alive():
            return
        self._renew_stop.clear()
        self._renew_thread = threading.Thread(
            target=self._command_renewal_loop,
            name="esp32-command-renewer",
            daemon=True,
        )
        self._renew_thread.start()

    def _stop_command_renewer(self) -> None:
        thread = self._renew_thread
        if thread is None:
            return
        self._renew_stop.set()
        if thread is not threading.current_thread():
            thread.join(timeout=1.0)
        self._renew_thread = None

    def _command_renewal_loop(self) -> None:
        interval = 0.5 / max(1, self._config.command_hz)
        while not self._renew_stop.wait(interval):
            self._renew_command_once()

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

    def _wait_for_status(self, sequence: int) -> dict[str, Any] | None:
        if self._serial is None:
            return None
        deadline = self._clock() + self._config.timeout_seconds
        while self._clock() <= deadline:
            waiting = int(getattr(self._serial, "in_waiting", 0))
            if waiting:
                for frame in self._decoder.feed(self._serial.read(waiting)):
                    decoded = decode_message(frame)
                    if frame.sequence == sequence and frame.message_type == STATUS:
                        return decoded
                    if frame.sequence == sequence and frame.message_type == ERROR:
                        return None
                    self._pending_telemetry.append(frame)
            self._sleep(min(0.001, self._config.timeout_seconds))
        return None

    @classmethod
    def _prearm_ready(cls, status: dict[str, Any]) -> bool:
        health = int(status.get("health_flags", 0))
        return (
            status.get("operating_mode") == DRIVE_MODE
            and status.get("active_source") == 2
            and health & cls._PREARM_HEALTH_FLAGS
            == cls._PREARM_HEALTH_FLAGS
            and status.get("faults") == 0
        )

    def _close_endpoint(self) -> None:
        endpoint = self._serial
        self._serial = None
        self._session_valid = False
        self._latest_motion = None
        self._latest_motion_at = float("-inf")
        self._last_motion_write_at = float("-inf")
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

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field
from typing import Callable

from .protocol import (
    ACK,
    ARM,
    CAPABILITIES,
    CLEAR_FAULT,
    DISARM,
    DRIVE_MODE,
    DRIVE_TELEMETRY,
    EMERGENCY_STOP,
    ERROR,
    FAULTS,
    HELLO,
    IMU,
    MOTOR,
    ODOMETRY,
    SET_OPERATING_MODE,
    SET_VELOCITY_YAW,
    STATUS,
    STOP,
    Frame,
    FrameDecoder,
    encode_frame,
)

OUTPUT_LIMIT = 250
SLEW_PER_SECOND = 500.0
CONTROL_PERIOD_SECONDS = 0.010
MOTOR_PERIOD_SECONDS = 0.100
FEEDBACK_TIMEOUT_SECONDS = 0.500
ACK_TIMEOUT_SECONDS = 0.500
SERIAL_TIMEOUT_SECONDS = 0.750
IMU_TIMEOUT_SECONDS = 0.100
MAXIMUM_TILT_DEGREES = 45.0

FAULT_SERIAL_DISCONNECTED = 1 << 0
FAULT_LEASE_EXPIRED = 1 << 1
FAULT_TRANSPORT_UNAVAILABLE = 1 << 2
FAULT_FEEDBACK_LOST = 1 << 3
FAULT_CONTROLLER_UNHEALTHY = 1 << 4
FAULT_ZERO_NOT_ACKNOWLEDGED = 1 << 5
FAULT_IMU_UNHEALTHY = 1 << 6
FAULT_UNSAFE_ORIENTATION = 1 << 7
FAULT_ACK_TIMEOUT = 1 << 8
FAULT_MALFORMED_COMMAND = 1 << 9
FAULT_LOCAL_DISARM = 1 << 10
FAULT_MASTER = 1 << 11
FAULT_SLAVE = 1 << 12
FAULT_NON_NEUTRAL_ARM = 1 << 13
FAULT_WRONG_MODE = 1 << 14
FAULT_FEEDBACK_CRC = 1 << 15

ERROR_MALFORMED = 1
ERROR_INVALID_CRC = 2
ERROR_STALE_SEQUENCE = 3
ERROR_UNSUPPORTED = 4
ERROR_LEASE_CONFLICT = 5
ERROR_UNSAFE_STATE = 6


class FakeClock:
    def __init__(self) -> None:
        self._seconds = 0.0
        self._listeners: list[Callable[[float], None]] = []

    def monotonic(self) -> float:
        return self._seconds

    def add_listener(self, listener: Callable[[float], None]) -> None:
        self._listeners.append(listener)

    def advance(self, seconds: float) -> None:
        if seconds < 0:
            raise ValueError("time cannot move backward")
        target = self._seconds + seconds
        while self._seconds + CONTROL_PERIOD_SECONDS < target + 1e-12:
            self._seconds += CONTROL_PERIOD_SECONDS
            for listener in tuple(self._listeners):
                listener(self._seconds)
        self._seconds = target
        for listener in tuple(self._listeners):
            listener(self._seconds)

    def sleep(self, seconds: float) -> None:
        self.advance(seconds)


@dataclass
class SimulatedMpu6050:
    clock: FakeClock
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    samples_enabled: bool = True
    values_valid: bool = True
    last_sample_at: float = 0.0

    def sample(self) -> None:
        if self.samples_enabled:
            self.last_sample_at = self.clock.monotonic()

    def level(self) -> None:
        self.pitch_deg = 0.0
        self.roll_deg = 0.0
        self.samples_enabled = True
        self.values_valid = True
        self.sample()

    def set_pitch(self, degrees: float) -> None:
        self.pitch_deg = degrees
        self.sample()

    def set_roll(self, degrees: float) -> None:
        self.roll_deg = degrees
        self.sample()

    def missing(self) -> None:
        self.samples_enabled = False

    def invalid(self) -> None:
        self.values_valid = False
        self.sample()

    def healthy(self) -> bool:
        now = self.clock.monotonic()
        return (
            self.samples_enabled
            and self.values_valid
            and math.isfinite(self.pitch_deg)
            and math.isfinite(self.roll_deg)
            and now - self.last_sample_at <= IMU_TIMEOUT_SECONDS
        )

    def orientation_safe(self) -> bool:
        return (
            self.healthy()
            and abs(self.pitch_deg) < MAXIMUM_TILT_DEGREES
            and abs(self.roll_deg) < MAXIMUM_TILT_DEGREES
        )


@dataclass
class PendingGd32Command:
    due_at: float
    sequence: int
    left: int
    right: int
    disable: bool


@dataclass
class SimulatedGd32Boundary:
    clock: FakeClock
    ack_delay_seconds: float = 0.0
    feedback_enabled: bool = True
    drop_next_ack: bool = False
    stale_next_ack: bool = False
    malformed_feedback: bool = False
    crc_failure: bool = False
    master_faults: int = 0
    slave_faults: int = 0
    left_responds: bool = True
    right_responds: bool = True
    accepted_esp_sequence: int = 0
    forwarded_slave_sequence: int = 0
    accepted_slave_sequence: int = 0
    applied_left: int = 0
    applied_right: int = 0
    left_odometer: int = 0
    right_odometer: int = 0
    last_feedback_at: float = 0.0
    commands: list[tuple[int, int, int, bool]] = field(default_factory=list)
    pending: list[PendingGd32Command] = field(default_factory=list)

    def submit(self, sequence: int, left: int, right: int, disable: bool) -> None:
        self.commands.append((sequence, left, right, disable))
        self.pending.append(
            PendingGd32Command(
                self.clock.monotonic() + self.ack_delay_seconds,
                sequence,
                left,
                right,
                disable,
            )
        )

    def step(self) -> None:
        if not self.feedback_enabled:
            return
        now = self.clock.monotonic()
        ready = [command for command in self.pending if command.due_at <= now]
        self.pending = [command for command in self.pending if command.due_at > now]
        for command in ready:
            if self.drop_next_ack:
                self.drop_next_ack = False
                continue
            sequence = (command.sequence - 1) & 0xFFFF if self.stale_next_ack else command.sequence
            self.stale_next_ack = False
            self.accepted_esp_sequence = sequence
            self.forwarded_slave_sequence = sequence
            self.accepted_slave_sequence = sequence
            if command.disable:
                self.applied_left = 0
                self.applied_right = 0
            else:
                self.applied_left = command.left if self.left_responds else 0
                self.applied_right = command.right if self.right_responds else 0
                self.left_odometer += int(self.applied_left / 10)
                self.right_odometer += int(self.applied_right / 10)
            self.last_feedback_at = now

    def exact_ack(self, sequence: int) -> bool:
        return (
            self.accepted_esp_sequence == sequence
            and self.forwarded_slave_sequence == sequence
            and self.accepted_slave_sequence == sequence
        )

    def runtime_healthy(self) -> bool:
        return (
            self.feedback_enabled
            and not self.malformed_feedback
            and not self.crc_failure
            and self.master_faults == 0
            and self.slave_faults == 0
        )

    def feedback_fresh(self) -> bool:
        return self.feedback_enabled and self.clock.monotonic() - self.last_feedback_at <= FEEDBACK_TIMEOUT_SECONDS

    def zero_ready(self, sequence: int) -> bool:
        return (
            self.runtime_healthy()
            and self.feedback_fresh()
            and self.exact_ack(sequence)
            and self.applied_left == 0
            and self.applied_right == 0
        )

    def force_odometry_disagreement(self, left_delta: int, right_delta: int) -> None:
        self.left_odometer += left_delta
        self.right_odometer += right_delta


class SimulatedEsp32Serial:
    """PySerial-compatible ESP32 endpoint using the production Pi wire protocol."""

    def __init__(
        self,
        clock: FakeClock | None = None,
        gd32: SimulatedGd32Boundary | None = None,
        mpu: SimulatedMpu6050 | None = None,
    ) -> None:
        self.clock = clock or FakeClock()
        self.gd32 = gd32 or SimulatedGd32Boundary(self.clock)
        self.mpu = mpu or SimulatedMpu6050(self.clock)
        self.mpu.level()
        self.is_open = True
        self._input = bytearray()
        self._decoder = FrameDecoder()
        self._last_sequence: int | None = None
        self._last_serial_at = 0.0
        self._lease_id = 0
        self._lease_expires_at = 0.0
        self._requested_linear = 0.0
        self._requested_yaw = 0.0
        self._target_left = 0.0
        self._target_right = 0.0
        self._commanded_left = 0.0
        self._commanded_right = 0.0
        self._mode = 0
        self._neutral_seen = False
        self._armed = False
        self._faults = 0
        self._gd32_sequence = 0
        self._in_flight_sequence = 0
        self._in_flight_started_at = 0.0
        self._last_motor_at = -MOTOR_PERIOD_SECONDS
        self._last_telemetry_at = 0.0
        self._telemetry_sequence = 0
        self._crc_errors = 0
        self._ack_timeouts = 0
        self._feedback_frames = 0
        self._transmitted_frames = 0
        self.event_log: list[str] = []
        self.clock.add_listener(self._on_time)
        self._send_gd32(0, 0, disable=True)
        self.gd32.step()

    @property
    def in_waiting(self) -> int:
        return len(self._input)

    @property
    def armed(self) -> bool:
        return self._armed

    @property
    def faults(self) -> int:
        return self._faults

    @property
    def commanded(self) -> tuple[int, int]:
        return round(self._commanded_left), round(self._commanded_right)

    def reopen(self) -> SimulatedEsp32Serial:
        self.is_open = True
        self._decoder = FrameDecoder()
        self._last_sequence = None
        self._last_serial_at = self.clock.monotonic()
        self._armed = False
        self._neutral_seen = False
        self._mode = 0
        self._requested_linear = 0.0
        self._requested_yaw = 0.0
        self._commanded_left = 0.0
        self._commanded_right = 0.0
        self._send_gd32(0, 0, disable=True)
        self.gd32.step()
        return self

    def reset_input_buffer(self) -> None:
        self._input.clear()

    def flush(self) -> None:
        return

    def close(self) -> None:
        if self.is_open:
            self.event_log.append("SERIAL_DISCONNECT")
            self._trip(FAULT_SERIAL_DISCONNECTED)
        self.is_open = False

    def read(self, size: int = 1) -> bytes:
        size = min(size, len(self._input))
        data = bytes(self._input[:size])
        del self._input[:size]
        return data

    def write(self, data: bytes) -> int:
        if not self.is_open:
            raise OSError("simulated USB serial cable is disconnected")
        crc_before = self._decoder.crc_errors
        malformed_before = self._decoder.malformed_frames
        frames = self._decoder.feed(data)
        if self._decoder.crc_errors != crc_before:
            self._faults |= FAULT_MALFORMED_COMMAND
            self._trip(FAULT_MALFORMED_COMMAND)
            self._queue_error(0, HELLO, ERROR_INVALID_CRC)
        if self._decoder.malformed_frames != malformed_before:
            self._trip(FAULT_MALFORMED_COMMAND)
            self._queue_error(0, HELLO, ERROR_MALFORMED)
        for frame in frames:
            self._process(frame)
        self._on_time(self.clock.monotonic())
        return len(data)

    def advance(self, seconds: float) -> None:
        self.clock.advance(seconds)

    def inject_usb_disconnect(self) -> None:
        self.close()

    def inject_local_disarm(self) -> None:
        self.event_log.append("LOCAL_DISARM")
        self._trip(FAULT_LOCAL_DISARM)

    def inject_master_fault(self, value: int = 1) -> None:
        self.gd32.master_faults = value
        self._on_time(self.clock.monotonic())

    def inject_slave_fault(self, value: int = 1) -> None:
        self.gd32.slave_faults = value
        self._on_time(self.clock.monotonic())

    def inject_feedback_crc(self) -> None:
        self.gd32.crc_failure = True
        self._on_time(self.clock.monotonic())

    def inject_malformed_feedback(self) -> None:
        self.gd32.malformed_feedback = True
        self._on_time(self.clock.monotonic())

    def _process(self, frame: Frame) -> None:
        now = self.clock.monotonic()
        if frame.message_type == HELLO:
            self._last_sequence = frame.sequence
            self._last_serial_at = now
            self._armed = False
            self._mode = 0
            self._neutral_seen = False
            self._faults &= ~(FAULT_SERIAL_DISCONNECTED | FAULT_LEASE_EXPIRED | FAULT_WRONG_MODE)
            self._zero_and_disable()
            self.event_log.append("HELLO")
            self._queue_capabilities(frame.sequence)
            return
        if not self._sequence_is_fresh(frame.sequence):
            self.event_log.append("STALE_SEQUENCE")
            self._trip(FAULT_MALFORMED_COMMAND)
            self._queue_error(frame.sequence, frame.message_type, ERROR_STALE_SEQUENCE)
            return
        self._last_sequence = frame.sequence
        self._last_serial_at = now

        if frame.message_type == SET_OPERATING_MODE:
            if len(frame.payload) != 1 or frame.payload[0] > 3:
                self._trip(FAULT_MALFORMED_COMMAND)
                self._queue_error(frame.sequence, frame.message_type, ERROR_MALFORMED)
                return
            self._mode = frame.payload[0]
            self._armed = False
            self._neutral_seen = False
            self._zero_and_disable()
            if self._mode != DRIVE_MODE:
                self._faults |= FAULT_WRONG_MODE
            else:
                self._faults &= ~FAULT_WRONG_MODE
            self.event_log.append(f"MODE_{self._mode}")
            self._queue_ack(frame)
            return

        if frame.message_type == SET_VELOCITY_YAW:
            if len(frame.payload) != 10:
                self._trip(FAULT_MALFORMED_COMMAND)
                self._queue_error(frame.sequence, frame.message_type, ERROR_MALFORMED)
                return
            linear_milli, yaw_milli, lease_id, lease_ms = struct.unpack("<hhIH", frame.payload)
            if self._lease_id and now <= self._lease_expires_at and lease_id != self._lease_id:
                self._trip(FAULT_MALFORMED_COMMAND)
                self._queue_error(frame.sequence, frame.message_type, ERROR_LEASE_CONFLICT)
                return
            self._lease_id = lease_id
            self._lease_expires_at = now + lease_ms / 1000.0
            self._requested_linear = linear_milli / 1000.0
            self._requested_yaw = yaw_milli / 1000.0
            if linear_milli == 0 and yaw_milli == 0:
                self._neutral_seen = True
                self._faults &= ~FAULT_NON_NEUTRAL_ARM
                if not self._armed:
                    self._zero_and_disable()
            self.event_log.append(f"MOVE:{linear_milli}:{yaw_milli}")
            self._queue_ack(frame)
            return

        if frame.message_type == ARM:
            self.gd32.step()
            reason = self._arm_rejection_reason()
            if reason == 0 and self._faults == 0:
                self._armed = True
                self.event_log.append("ARM")
                self._queue_ack(frame)
            else:
                if reason == FAULT_NON_NEUTRAL_ARM:
                    self._faults |= reason
                self._armed = False
                self._queue_error(frame.sequence, frame.message_type, ERROR_UNSAFE_STATE)
            return

        if frame.message_type in (STOP, DISARM, EMERGENCY_STOP):
            self.event_log.append("STOP" if frame.message_type == STOP else "DISARM")
            self._armed = False
            self._lease_id = 0
            self._lease_expires_at = 0.0
            self._zero_and_disable()
            self._queue_ack(frame)
            return

        if frame.message_type == CLEAR_FAULT:
            if self._arm_rejection_reason(require_zero=True) == 0:
                self._faults = 0
                self.gd32.crc_failure = False
                self.gd32.malformed_feedback = False
                self.event_log.append("CLEAR_FAULT")
                self._queue_ack(frame)
            else:
                self._queue_error(frame.sequence, frame.message_type, ERROR_UNSAFE_STATE)
            return

        if frame.message_type == CAPABILITIES:
            self._queue_capabilities(frame.sequence)
        elif frame.message_type == STATUS:
            self._queue_status(frame.sequence)
        elif frame.message_type == IMU:
            self._queue_imu(frame.sequence)
        elif frame.message_type == MOTOR:
            self._queue_motor(frame.sequence)
        elif frame.message_type == ODOMETRY:
            self._queue_odometry(frame.sequence)
        elif frame.message_type == FAULTS:
            self._queue_faults(frame.sequence)
        elif frame.message_type == DRIVE_TELEMETRY:
            self._queue_drive(frame.sequence)
        else:
            self._queue_error(frame.sequence, frame.message_type, ERROR_UNSUPPORTED)

    def _sequence_is_fresh(self, sequence: int) -> bool:
        if self._last_sequence is None:
            return False
        delta = (sequence - self._last_sequence) & 0xFFFF
        return 0 < delta < 0x8000

    def _arm_rejection_reason(self, require_zero: bool = True) -> int:
        now = self.clock.monotonic()
        if self._mode != DRIVE_MODE:
            return FAULT_WRONG_MODE
        if not self._neutral_seen:
            return FAULT_NON_NEUTRAL_ARM
        if not self.mpu.healthy():
            return FAULT_IMU_UNHEALTHY
        if not self.mpu.orientation_safe():
            return FAULT_UNSAFE_ORIENTATION
        if not self.gd32.feedback_fresh():
            return FAULT_FEEDBACK_LOST
        if not self.gd32.runtime_healthy():
            return FAULT_CONTROLLER_UNHEALTHY
        if self.gd32.master_faults:
            return FAULT_MASTER
        if self.gd32.slave_faults:
            return FAULT_SLAVE
        if require_zero and not self.gd32.zero_ready(self._in_flight_sequence):
            return FAULT_ZERO_NOT_ACKNOWLEDGED
        if now - self._last_serial_at > SERIAL_TIMEOUT_SECONDS:
            return FAULT_SERIAL_DISCONNECTED
        return 0

    def _on_time(self, now: float) -> None:
        if not self.is_open:
            return
        self.mpu.sample()
        self.gd32.step()
        if self.gd32.feedback_enabled:
            self._feedback_frames += 1
        if self._armed:
            if now > self._lease_expires_at:
                self._trip(FAULT_LEASE_EXPIRED)
            elif now - self._last_serial_at > SERIAL_TIMEOUT_SECONDS:
                self._trip(FAULT_SERIAL_DISCONNECTED)
            elif not self.mpu.healthy():
                self._trip(FAULT_IMU_UNHEALTHY)
            elif not self.mpu.orientation_safe():
                self._trip(FAULT_UNSAFE_ORIENTATION)
            elif self.gd32.master_faults:
                self._trip(FAULT_MASTER)
            elif self.gd32.slave_faults:
                self._trip(FAULT_SLAVE)
            elif self.gd32.crc_failure:
                self._crc_errors += 1
                self._trip(FAULT_FEEDBACK_CRC)
            elif self.gd32.malformed_feedback:
                self._trip(FAULT_CONTROLLER_UNHEALTHY)
            elif not self.gd32.feedback_fresh():
                self._trip(FAULT_FEEDBACK_LOST)
            else:
                self._mix_and_slew(CONTROL_PERIOD_SECONDS)
        else:
            self._commanded_left = 0.0
            self._commanded_right = 0.0

        if now - self._last_motor_at >= MOTOR_PERIOD_SECONDS:
            self._last_motor_at = now
            if self._armed and self._faults == 0:
                self._send_gd32(round(self._commanded_left), round(self._commanded_right), disable=False)
            else:
                self._send_gd32(0, 0, disable=True)
            self.gd32.step()

        if (
            self._in_flight_sequence
            and not self.gd32.exact_ack(self._in_flight_sequence)
            and now - self._in_flight_started_at > ACK_TIMEOUT_SECONDS
        ):
            self._ack_timeouts += 1
            self._trip(FAULT_ACK_TIMEOUT)

        if self._last_sequence is not None and now - self._last_telemetry_at >= MOTOR_PERIOD_SECONDS:
            self._last_telemetry_at = now
            self._telemetry_sequence = (self._telemetry_sequence + 1) & 0xFFFF
            sequence = self._telemetry_sequence
            self._queue_status(sequence)
            self._queue_drive(sequence)
            self._queue_motor(sequence)
            self._queue_odometry(sequence)
            self._queue_faults(sequence)

    def _mix_and_slew(self, dt: float) -> None:
        linear = self._requested_linear * 650.0
        yaw = self._requested_yaw * 350.0
        left = linear + yaw
        right = linear - yaw
        peak = max(abs(left), abs(right))
        if peak > OUTPUT_LIMIT:
            scale = OUTPUT_LIMIT / peak
            left *= scale
            right *= scale
        self._target_left = max(-OUTPUT_LIMIT, min(OUTPUT_LIMIT, left))
        self._target_right = max(-OUTPUT_LIMIT, min(OUTPUT_LIMIT, right))
        maximum_delta = SLEW_PER_SECOND * dt
        self._commanded_left = self._move_toward(self._commanded_left, self._target_left, maximum_delta)
        self._commanded_right = self._move_toward(self._commanded_right, self._target_right, maximum_delta)

    @staticmethod
    def _move_toward(current: float, target: float, maximum_delta: float) -> float:
        if target > current:
            return min(target, current + maximum_delta)
        return max(target, current - maximum_delta)

    def _send_gd32(self, left: int, right: int, disable: bool) -> None:
        self._gd32_sequence = (self._gd32_sequence + 1) & 0xFFFF
        if self._gd32_sequence == 0:
            self._gd32_sequence = 1
        self._in_flight_sequence = self._gd32_sequence
        self._in_flight_started_at = self.clock.monotonic()
        self._transmitted_frames += 1
        self.gd32.submit(self._gd32_sequence, left, right, disable)

    def _zero_and_disable(self) -> None:
        self._requested_linear = 0.0
        self._requested_yaw = 0.0
        self._target_left = 0.0
        self._target_right = 0.0
        self._commanded_left = 0.0
        self._commanded_right = 0.0
        self._send_gd32(0, 0, disable=True)
        self.gd32.step()

    def _trip(self, fault: int) -> None:
        self._faults |= fault
        self._armed = False
        self._zero_and_disable()

    def _queue(self, message_type: int, sequence: int, payload: bytes) -> None:
        self._input.extend(encode_frame(message_type, sequence, payload))

    def _queue_ack(self, frame: Frame) -> None:
        self._queue(ACK, frame.sequence, bytes((frame.message_type, 0)))

    def _queue_error(self, sequence: int, request_type: int, code: int) -> None:
        self._queue(ERROR, sequence, struct.pack("<BBH", request_type, code, 0))

    def _queue_capabilities(self, sequence: int) -> None:
        payload = struct.pack("<BBBBHHHH", 1, 0, 0, 1 << DRIVE_MODE, 100, 10, 48, 0)
        self._queue(CAPABILITIES, sequence, payload)

    def _queue_status(self, sequence: int) -> None:
        state = 6 if self._faults else (4 if self._armed else 2)
        health = (
            (1 if self.mpu.healthy() else 0)
            | (4 if self.gd32.feedback_fresh() else 0)
            | (8 if self.gd32.runtime_healthy() else 0)
            | (16 if self._armed else 0)
            | (32 if self.gd32.zero_ready(self._in_flight_sequence) else 0)
            | (64 if self.is_open else 0)
        )
        self._queue(STATUS, sequence, struct.pack("<BBBBIII", state, self._mode, 2, health, self._faults, 0, 0))

    def _queue_imu(self, sequence: int) -> None:
        payload = bytearray(44)
        payload[0] = 0x68
        payload[1] = 1
        payload[2] = int(self.mpu.healthy())
        struct.pack_into("<h", payload, 18, round(self.mpu.pitch_deg * 100))
        struct.pack_into("<I", payload, 32, round((self.clock.monotonic() - self.mpu.last_sample_at) * 1_000_000))
        self._queue(IMU, sequence, bytes(payload))

    def _queue_motor(self, sequence: int) -> None:
        payload = struct.pack(
            "<hhhhHHIIIIIIIIH",
            round(self._commanded_left),
            round(self._commanded_right),
            self.gd32.applied_left,
            self.gd32.applied_right,
            self._in_flight_sequence,
            int(self._armed),
            self._transmitted_frames,
            self._feedback_frames,
            self._crc_errors,
            self._ack_timeouts,
            0,
            0,
            0,
            0,
            1000,
        )
        self._queue(MOTOR, sequence, payload)

    def _queue_drive(self, sequence: int) -> None:
        flags = (
            (1 if self._neutral_seen else 0)
            | (2 if self.gd32.feedback_fresh() else 0)
            | (4 if self.gd32.zero_ready(self._in_flight_sequence) else 0)
            | (8 if self.is_open else 0)
        )
        payload = struct.pack(
            "<hhhhhhhhIBBBB",
            round(self._requested_linear * 1000),
            round(self._requested_yaw * 1000),
            round(self._target_left),
            round(self._target_right),
            round(self._commanded_left),
            round(self._commanded_right),
            self.gd32.applied_left,
            self.gd32.applied_right,
            self._faults,
            2,
            self._mode,
            int(self._armed),
            flags,
        )
        self._queue(DRIVE_TELEMETRY, sequence, payload)

    def _queue_odometry(self, sequence: int) -> None:
        payload = struct.pack(
            "<iiiQ",
            self.gd32.left_odometer,
            self.gd32.right_odometer,
            0,
            round(self.clock.monotonic() * 1_000_000),
        )
        self._queue(ODOMETRY, sequence, payload)

    def _queue_faults(self, sequence: int) -> None:
        payload = struct.pack("<IIII", self._faults, self.gd32.master_faults, self.gd32.slave_faults, 0)
        self._queue(FAULTS, sequence, payload)


class SimulatedSerialFactory:
    def __init__(self, endpoint: SimulatedEsp32Serial, available: bool = True) -> None:
        self.endpoint = endpoint
        self.available = available
        self.open_attempts = 0

    def __call__(self, *args: object, **kwargs: object) -> SimulatedEsp32Serial:
        del args, kwargs
        self.open_attempts += 1
        if not self.available:
            raise FileNotFoundError("simulated ESP32 serial device is absent")
        return self.endpoint.reopen()

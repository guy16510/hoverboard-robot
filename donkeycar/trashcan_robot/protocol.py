from __future__ import annotations

import struct
from dataclasses import dataclass

MARKER = b"\xA5\x5A"
VERSION = 1
MAX_PAYLOAD = 48
HELLO = 0x01
CAPABILITIES = 0x02
ARM = 0x10
DISARM = 0x11
STOP = 0x12
EMERGENCY_STOP = 0x13
CLEAR_FAULT = 0x14
SET_OPERATING_MODE = 0x15
SET_VELOCITY_YAW = 0x22
HEARTBEAT = 0x23
STATUS = 0x30
IMU = 0x31
MOTOR = 0x32
ODOMETRY = 0x33
FAULTS = 0x34
ULTRASONIC = 0x35
ACK = 0x7E
ERROR = 0x7F
DRIVE_MODE = 2

MOTION_PAYLOAD_BYTES = 10
CAPABILITIES_PAYLOAD_BYTES = 12
ULTRASONIC_PAYLOAD_BYTES = 8
ACK_PAYLOAD_BYTES = 2
ERROR_PAYLOAD_BYTES = 4


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(message_type: int, sequence: int, payload: bytes = b"", flags: int = 0) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload exceeds protocol maximum")
    body = struct.pack("<BBBHH", VERSION, message_type, flags, sequence & 0xFFFF, len(payload)) + payload
    return MARKER + body + struct.pack("<H", crc16_ccitt_false(body))


def encode_motion(linear_velocity: float, angular_velocity: float, lease_id: int, lease_ms: int) -> bytes:
    linear = max(-32768, min(32767, round(linear_velocity * 1000)))
    angular = max(-32768, min(32767, round(angular_velocity * 1000)))
    payload = struct.pack("<hhIH", linear, angular, lease_id & 0xFFFFFFFF, lease_ms)
    assert len(payload) == MOTION_PAYLOAD_BYTES
    return payload


@dataclass(frozen=True)
class Frame:
    message_type: int
    flags: int
    sequence: int
    payload: bytes


@dataclass(frozen=True)
class Capabilities:
    protocol_version: int
    dry_run: bool
    web_control: bool
    operating_mode_mask: int
    control_hz: int
    motor_output_hz: int
    maximum_payload: int
    runtime_config_keys: int

    def supports_mode(self, mode: int) -> bool:
        return bool(self.operating_mode_mask & (1 << mode))


@dataclass(frozen=True)
class Acknowledgment:
    request_type: int
    status: int


@dataclass(frozen=True)
class ErrorResponse:
    request_type: int
    code: int
    detail: int


@dataclass(frozen=True)
class UltrasonicReading:
    front_m: float | None
    left_m: float | None
    right_m: float | None


def decode_capabilities(payload: bytes) -> Capabilities:
    if len(payload) != CAPABILITIES_PAYLOAD_BYTES:
        raise ValueError("capabilities payload must be 12 bytes")
    version, dry_run, web_control, mode_mask, control_hz, motor_hz, max_payload, runtime_keys = struct.unpack(
        "<BBBBHHHH", payload
    )
    return Capabilities(
        protocol_version=version,
        dry_run=bool(dry_run),
        web_control=bool(web_control),
        operating_mode_mask=mode_mask,
        control_hz=control_hz,
        motor_output_hz=motor_hz,
        maximum_payload=max_payload,
        runtime_config_keys=runtime_keys,
    )


def decode_ack(payload: bytes) -> Acknowledgment:
    if len(payload) != ACK_PAYLOAD_BYTES:
        raise ValueError("acknowledgment payload must be 2 bytes")
    request_type, status = struct.unpack("<BB", payload)
    return Acknowledgment(request_type=request_type, status=status)


def decode_error(payload: bytes) -> ErrorResponse:
    if len(payload) != ERROR_PAYLOAD_BYTES:
        raise ValueError("error payload must be 4 bytes")
    request_type, code, detail = struct.unpack("<BBH", payload)
    return ErrorResponse(request_type=request_type, code=code, detail=detail)


def decode_ultrasonic(payload: bytes) -> UltrasonicReading:
    if len(payload) != ULTRASONIC_PAYLOAD_BYTES:
        raise ValueError("ultrasonic payload must be 8 bytes")
    front_mm, left_mm, right_mm, valid_mask, _reserved = struct.unpack("<HHHBB", payload)

    def value(index: int, millimeters: int) -> float | None:
        if not valid_mask & (1 << index) or millimeters == 0xFFFF:
            return None
        return millimeters / 1000.0

    return UltrasonicReading(
        front_m=value(0, front_mm),
        left_m=value(1, left_mm),
        right_m=value(2, right_mm),
    )


class FrameDecoder:
    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[Frame]:
        self._buffer.extend(data)
        frames: list[Frame] = []
        while True:
            marker = self._buffer.find(MARKER)
            if marker < 0:
                self._buffer[:] = self._buffer[-1:]
                break
            if marker:
                del self._buffer[:marker]
            if len(self._buffer) < 11:
                break
            version, message_type, flags, sequence, payload_len = struct.unpack_from("<BBBHH", self._buffer, 2)
            if version != VERSION or payload_len > MAX_PAYLOAD:
                del self._buffer[0]
                continue
            frame_len = 11 + payload_len
            if len(self._buffer) < frame_len:
                break
            body = bytes(self._buffer[2:9 + payload_len])
            expected = struct.unpack_from("<H", self._buffer, 9 + payload_len)[0]
            if crc16_ccitt_false(body) != expected:
                del self._buffer[0]
                continue
            payload = bytes(self._buffer[9:9 + payload_len])
            frames.append(Frame(message_type, flags, sequence, payload))
            del self._buffer[:frame_len]
        return frames

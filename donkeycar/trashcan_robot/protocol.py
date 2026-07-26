from __future__ import annotations

import struct
from dataclasses import dataclass

MARKER = b"\xA5\x5A"
VERSION = 1
MAX_PAYLOAD = 48
HELLO = 0x01
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
ACK = 0x7E
ERROR = 0x7F
DRIVE_MODE = 2


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
    return struct.pack("<hhIH", linear, angular, lease_id & 0xFFFFFFFF, lease_ms)


@dataclass(frozen=True)
class Frame:
    message_type: int
    flags: int
    sequence: int
    payload: bytes


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

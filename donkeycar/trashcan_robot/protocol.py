from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Any

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
DRIVE_TELEMETRY = 0x35
CONTROLLER_TELEMETRY = 0x36
RESILIENCE_TELEMETRY = 0x37
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
    if not 0 <= message_type <= 0xFF:
        raise ValueError("message type must fit uint8")
    if not 0 <= sequence <= 0xFFFF:
        raise ValueError("sequence must fit uint16")
    if not 0 <= flags <= 0xFF:
        raise ValueError("flags must fit uint8")
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload exceeds protocol maximum")
    body = struct.pack("<BBBHH", VERSION, message_type, flags, sequence, len(payload)) + payload
    return MARKER + body + struct.pack("<H", crc16_ccitt_false(body))


def encode_motion(linear_velocity: float, angular_velocity: float, lease_id: int, lease_ms: int) -> bytes:
    linear = max(-32768, min(32767, round(linear_velocity * 1000)))
    angular = max(-32768, min(32767, round(angular_velocity * 1000)))
    if not 0 <= lease_id <= 0xFFFFFFFF:
        raise ValueError("lease_id must fit uint32")
    if not 1 <= lease_ms <= 0xFFFF:
        raise ValueError("lease_ms must be between 1 and 65535")
    return struct.pack("<hhIH", linear, angular, lease_id, lease_ms)


@dataclass(frozen=True)
class Frame:
    message_type: int
    flags: int
    sequence: int
    payload: bytes


class FrameDecoder:
    def __init__(self) -> None:
        self._buffer = bytearray()
        self.crc_errors = 0
        self.malformed_frames = 0

    def feed(self, data: bytes) -> list[Frame]:
        self._buffer.extend(data)
        frames: list[Frame] = []
        while True:
            marker = self._buffer.find(MARKER)
            if marker < 0:
                self._buffer[:] = self._buffer[-1:] if self._buffer[-1:] == MARKER[:1] else b""
                break
            if marker:
                del self._buffer[:marker]
            if len(self._buffer) < 11:
                break
            version, message_type, flags, sequence, payload_len = struct.unpack_from("<BBBHH", self._buffer, 2)
            if version != VERSION or payload_len > MAX_PAYLOAD:
                self.malformed_frames += 1
                del self._buffer[0]
                continue
            frame_len = 11 + payload_len
            if len(self._buffer) < frame_len:
                break
            body = bytes(self._buffer[2:9 + payload_len])
            expected = struct.unpack_from("<H", self._buffer, 9 + payload_len)[0]
            if crc16_ccitt_false(body) != expected:
                self.crc_errors += 1
                del self._buffer[0]
                continue
            payload = bytes(self._buffer[9:9 + payload_len])
            frames.append(Frame(message_type, flags, sequence, payload))
            del self._buffer[:frame_len]
        return frames


def decode_message(frame: Frame) -> dict[str, Any]:
    payload = frame.payload
    if frame.message_type == CAPABILITIES and len(payload) == 12:
        version, dry_run, web_enabled, supported_modes, control_rate, motor_rate, maximum_payload, config_keys = struct.unpack(
            "<BBBBHHHH", payload
        )
        return {
            "name": "capabilities",
            "protocol_version": version,
            "dry_run": bool(dry_run),
            "web_enabled": bool(web_enabled),
            "supported_modes": supported_modes,
            "control_rate_hz": control_rate,
            "motor_rate_hz": motor_rate,
            "maximum_payload": maximum_payload,
            "configuration_keys": config_keys,
        }
    if frame.message_type == STATUS and len(payload) == 16:
        state, mode, source, health, faults, overruns, rejected = struct.unpack("<BBBBIII", payload)
        return {
            "name": "status",
            "state": state,
            "operating_mode": mode,
            "active_source": source,
            "health_flags": health,
            "faults": faults,
            "loop_overruns": overruns,
            "rejected_serial_frames": rejected,
        }
    if frame.message_type == MOTOR and len(payload) == 46:
        values = struct.unpack("<hhhhHHIIIIIIIIH", payload)
        return {
            "name": "motor",
            "calculated": [values[0], values[1]],
            "applied": [values[2], values[3]],
            "sequence": values[4],
            "flags": values[5],
            "transmitted_frames": values[6],
            "feedback_frames": values[7],
            "crc_errors": values[8],
            "acknowledgment_timeouts": values[9],
            "last_ack_latency_us": values[10],
            "maximum_ack_latency_us": values[11],
            "last_apply_latency_us": values[12],
            "maximum_apply_latency_us": values[13],
            "transmit_rate_hz": values[14] / 100.0,
        }
    if frame.message_type == DRIVE_TELEMETRY and len(payload) == 24:
        requested_linear, requested_yaw, mixed_left, mixed_right, commanded_left, commanded_right, applied_left, applied_right, faults, source, mode, arm_state, flags = struct.unpack(
            "<hhhhhhhhIBBBB", payload
        )
        return {
            "name": "drive",
            "requested_linear": requested_linear / 1000.0,
            "requested_yaw": requested_yaw / 1000.0,
            "mixed": [mixed_left, mixed_right],
            "commanded": [commanded_left, commanded_right],
            "applied": [applied_left, applied_right],
            "safety_faults": faults,
            "active_source": source,
            "operating_mode": mode,
            "armed": bool(arm_state),
            "flags": flags,
        }
    if frame.message_type == ODOMETRY and len(payload) == 20:
        left, right, velocity_milli, timestamp_us = struct.unpack("<iiiQ", payload)
        return {
            "name": "odometry",
            "left": left,
            "right": right,
            "velocity": velocity_milli / 1000.0,
            "timestamp_us": timestamp_us,
        }
    if frame.message_type == FAULTS and len(payload) == 16:
        balance, master, slave, feedback_health = struct.unpack("<IIII", payload)
        controller_status_flags = feedback_health & 0xFF
        motor_status_flags = (feedback_health >> 8) & 0xFF
        return {
            "name": "faults",
            "balance": balance,
            "master": master,
            "slave": slave,
            "feedback_health": feedback_health,
            "controller_status_flags": controller_status_flags,
            "motor_status_flags": motor_status_flags,
            "peer_healthy": bool(controller_status_flags & (1 << 0)),
            "master_pa4_high": bool(controller_status_flags & (1 << 1)),
            "pa4_bypass": bool(controller_status_flags & (1 << 2)),
            "clear_pending": bool(controller_status_flags & (1 << 3)),
            "transport_overflow": bool(controller_status_flags & 0xF0),
            "left_bridge_enabled": bool(motor_status_flags & (1 << 0)),
            "right_bridge_enabled": bool(motor_status_flags & (1 << 1)),
            "slave_pa4_high": bool(motor_status_flags & (1 << 2)),
        }
    if frame.message_type == CONTROLLER_TELEMETRY and len(payload) == 24:
        values = struct.unpack("<BBBBHHHBBHHHHHH", payload)
        motor_status_flags = values[3]
        return {
            "name": "controller",
            "states": [values[0], values[1]],
            "status_flags": values[2],
            "motor_status_flags": motor_status_flags,
            "command_ages_ms": {
                "master": values[4],
                "slave_feedback": values[5],
                "slave_command": values[6],
            },
            "halls": [values[7], values[8]],
            "bridges_enabled": [
                bool(motor_status_flags & (1 << 0)),
                bool(motor_status_flags & (1 << 1)),
            ],
            "compare_offsets": [values[9], values[10]],
            "remote_rx_bytes": values[11],
            "remote_valid_frames": values[12],
            "remote_invalid_frames": values[13],
            "remote_framing_errors": values[14],
        }
    if frame.message_type == RESILIENCE_TELEMETRY and len(payload) == 48:
        values = struct.unpack("<HBBIHHHHHHIIIBBBBhhhhI", payload)
        return {
            "name": "resilience",
            "warning_flags": values[0],
            "feedback_crc": {
                "streak": values[1],
                "threshold": values[2],
                "total": values[3],
            },
            "hall_glitches": [values[4], values[5]],
            "inter_controller_link": {
                "slave_feedback_invalid": values[6],
                "slave_feedback_framing": values[7],
                "slave_command_invalid": values[8],
                "slave_command_framing": values[9],
            },
            "first_fault": {
                "drive": values[10],
                "master": values[11],
                "slave": values[12],
                "states": [values[13], values[14]],
                "halls": [values[15], values[16]],
                "commanded": [values[17], values[18]],
                "applied": [values[19], values[20]],
                "esp32_uptime_ms": values[21],
            },
        }
    if frame.message_type == ACK and len(payload) == 2:
        return {"name": "acknowledgment", "request_type": payload[0], "status": payload[1]}
    if frame.message_type == ERROR and len(payload) == 4:
        return {
            "name": "error",
            "request_type": payload[0],
            "error_code": payload[1],
            "detail": struct.unpack_from("<H", payload, 2)[0],
        }
    return {"name": "unknown", "message_type": frame.message_type, "payload": payload}

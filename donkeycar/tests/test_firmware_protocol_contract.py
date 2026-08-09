import re
from pathlib import Path

from trashcan_robot import protocol


CONTRACT = Path(__file__).parents[2] / "firmware" / "esp32" / "protocol_contract.h"


def cpp_int(name: str) -> int:
    text = CONTRACT.read_text(encoding="utf-8")
    match = re.search(rf"\b{name}\s*=\s*(0x[0-9A-Fa-f]+|\d+)", text)
    assert match is not None, f"missing {name} in ESP32 protocol contract"
    return int(match.group(1), 0)


def test_protocol_scalar_contract_matches_firmware() -> None:
    assert cpp_int("kMarker0") == protocol.MARKER[0]
    assert cpp_int("kMarker1") == protocol.MARKER[1]
    assert cpp_int("kVersion") == protocol.VERSION
    assert cpp_int("kMaximumPayload") == protocol.MAX_PAYLOAD
    assert cpp_int("kDriveMode") == protocol.DRIVE_MODE
    assert cpp_int("kMotionPayloadBytes") == protocol.MOTION_PAYLOAD_BYTES
    assert cpp_int("kCapabilitiesPayloadBytes") == protocol.CAPABILITIES_PAYLOAD_BYTES
    assert cpp_int("kUltrasonicPayloadBytes") == protocol.ULTRASONIC_PAYLOAD_BYTES
    assert cpp_int("kHallPayloadBytes") == protocol.HALL_PAYLOAD_BYTES
    assert cpp_int("kAcknowledgmentPayloadBytes") == protocol.ACK_PAYLOAD_BYTES
    assert cpp_int("kErrorPayloadBytes") == protocol.ERROR_PAYLOAD_BYTES


def test_protocol_message_ids_match_firmware() -> None:
    expected = {
        "kHello": protocol.HELLO,
        "kCapabilities": protocol.CAPABILITIES,
        "kArm": protocol.ARM,
        "kDisarm": protocol.DISARM,
        "kStop": protocol.STOP,
        "kEmergencyStop": protocol.EMERGENCY_STOP,
        "kClearFault": protocol.CLEAR_FAULT,
        "kSetOperatingMode": protocol.SET_OPERATING_MODE,
        "kSetVelocityYaw": protocol.SET_VELOCITY_YAW,
        "kHeartbeat": protocol.HEARTBEAT,
        "kStatus": protocol.STATUS,
        "kUltrasonic": protocol.ULTRASONIC,
        "kHall": protocol.HALL,
        "kAcknowledgment": protocol.ACK,
        "kError": protocol.ERROR,
    }
    for name, value in expected.items():
        assert cpp_int(name) == value

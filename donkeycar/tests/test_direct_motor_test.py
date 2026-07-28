import importlib.util
import struct
import sys
import types
from pathlib import Path

from trashcan_robot.protocol import (
    ARM,
    DISARM,
    SET_OPERATING_MODE,
    STATUS,
    STOP,
)

if "serial" not in sys.modules:
    sys.modules["serial"] = types.SimpleNamespace(Serial=object)

SCRIPT_PATH = Path(__file__).parents[1] / "scripts" / "direct_motor_test.py"
SPEC = importlib.util.spec_from_file_location("trashcan_direct_motor_test", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
direct_motor_test = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(direct_motor_test)


def test_direct_test_is_bounded_and_stops_after_one_full_power_command() -> None:
    requests = direct_motor_test.bounded_direct_requests(lease_id=0x12345678)

    assert [message_type for message_type, _ in requests] == [
        SET_OPERATING_MODE,
        direct_motor_test.SET_DIRECT_MOTOR,
        ARM,
        STATUS,
        direct_motor_test.SET_DIRECT_MOTOR,
        direct_motor_test.SET_DIRECT_MOTOR,
        STOP,
        DISARM,
    ]
    assert requests[0][1] == b"\x03"
    decoded = [
        struct.unpack("<hhIH", payload)
        for message_type, payload in requests
        if message_type == direct_motor_test.SET_DIRECT_MOTOR
    ]
    assert decoded == [
        (0, 0, 0x12345678, 500),
        (250, 250, 0x12345678, 2000),
        (0, 0, 0x12345678, 500),
    ]


def test_direct_encoder_rejects_above_full_profile_power() -> None:
    direct_motor_test.encode_direct(250, -250, 1, 500)
    try:
        direct_motor_test.encode_direct(251, 0, 1, 500)
    except ValueError:
        pass
    else:
        raise AssertionError("command above the full-power ceiling was accepted")

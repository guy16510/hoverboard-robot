import importlib.util
import sys
import types
from pathlib import Path

from trashcan_robot.protocol import (
    ARM,
    DISARM,
    FAULTS,
    HELLO,
    IMU,
    MOTOR,
    ODOMETRY,
    RESILIENCE_TELEMETRY,
    SET_OPERATING_MODE,
    SET_VELOCITY_YAW,
    STATUS,
    STOP,
)

if "serial" not in sys.modules:
    sys.modules["serial"] = types.SimpleNamespace(Serial=object)

SCRIPT_PATH = Path(__file__).parents[1] / "scripts" / "read_telemetry.py"
SPEC = importlib.util.spec_from_file_location("trashcan_read_telemetry", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
read_telemetry = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(read_telemetry)


def test_read_only_capture_cannot_arm_or_send_movement() -> None:
    requests = read_telemetry.read_only_request_types()

    assert requests == [
        HELLO,
        DISARM,
        STATUS,
        IMU,
        MOTOR,
        ODOMETRY,
        FAULTS,
        RESILIENCE_TELEMETRY,
        STOP,
        DISARM,
    ]
    assert not {ARM, SET_OPERATING_MODE, SET_VELOCITY_YAW}.intersection(requests)

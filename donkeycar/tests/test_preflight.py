import importlib.util
import struct
import sys
import types
from pathlib import Path

import pytest

from trashcan_robot.protocol import (
    ARM,
    SET_OPERATING_MODE,
    SET_VELOCITY_YAW,
    STATUS,
)

if "serial" not in sys.modules:
    sys.modules["serial"] = types.SimpleNamespace(Serial=object)

SCRIPT_PATH = Path(__file__).parents[1] / "scripts" / "preflight.py"
SPEC = importlib.util.spec_from_file_location("trashcan_preflight", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
preflight = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(preflight)


def test_preflight_sends_zero_demand_immediately_before_arm() -> None:
    requests = preflight.zero_demand_startup_requests(
        lease_id=0x12345678,
        lease_ms=500,
    )

    assert [message_type for message_type, _ in requests] == [
        SET_OPERATING_MODE,
        SET_VELOCITY_YAW,
        ARM,
        STATUS,
    ]
    linear, angular, lease_id, lease_ms = struct.unpack("<hhIH", requests[1][1])
    assert (linear, angular, lease_id, lease_ms) == (0, 0, 0x12345678, 500)


def test_preflight_requires_driving_state_serial_source_and_health() -> None:
    preflight.validate_preflight_status(
        state=4,
        mode=2,
        source=2,
        health=0x55,
        faults=0,
    )

    with pytest.raises(RuntimeError, match="state"):
        preflight.validate_preflight_status(2, 2, 2, 0x55, 0)
    with pytest.raises(RuntimeError, match="source"):
        preflight.validate_preflight_status(4, 2, 0, 0x55, 0)
    with pytest.raises(RuntimeError, match="health"):
        preflight.validate_preflight_status(4, 2, 2, 0x45, 0)


def test_preflight_failure_includes_complete_status_context() -> None:
    with pytest.raises(
        RuntimeError,
        match=(
            r"state=2 mode=2 source=0 health=0x45 "
            r"faults=0x00000008"
        ),
    ):
        preflight.validate_preflight_status(2, 2, 0, 0x45, 8)


def test_preflight_waits_for_esp32_boot_before_clearing_input() -> None:
    events: list[object] = []
    port = types.SimpleNamespace(
        reset_input_buffer=lambda: events.append("reset_input"),
    )

    preflight.settle_serial_port(
        port,
        2.0,
        sleep=lambda seconds: events.append(("sleep", seconds)),
    )

    assert events == [("sleep", 2.0), "reset_input"]

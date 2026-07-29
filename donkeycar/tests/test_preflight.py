import importlib.util
import struct
import sys
import types
from pathlib import Path

import pytest

from trashcan_robot.protocol import (
    ARM,
    CLEAR_FAULT,
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


def test_preflight_waits_for_zero_status_before_arm() -> None:
    requests = preflight.zero_demand_startup_requests(
        lease_id=0x12345678,
        lease_ms=500,
    )

    assert [message_type for message_type, _ in requests] == [
        SET_OPERATING_MODE,
        SET_VELOCITY_YAW,
        CLEAR_FAULT,
        STATUS,
    ]
    linear, angular, lease_id, lease_ms = struct.unpack("<hhIH", requests[1][1])
    assert (linear, angular, lease_id, lease_ms) == (0, 0, 0x12345678, 500)
    assert ARM not in [message_type for message_type, _ in requests]


def test_preflight_requires_exact_zero_ack_and_healthy_feedback() -> None:
    assert preflight.zero_acknowledged(0x7D)
    assert not preflight.zero_acknowledged(0x5D)
    assert not preflight.zero_acknowledged(0x75)


def test_preflight_requires_driving_state_serial_source_and_health() -> None:
    preflight.validate_preflight_status(
        state=4,
        mode=2,
        source=2,
        health=0x7D,
        faults=0,
    )

    with pytest.raises(RuntimeError, match="state"):
        preflight.validate_preflight_status(2, 2, 2, 0x7D, 0)
    with pytest.raises(RuntimeError, match="source"):
        preflight.validate_preflight_status(4, 2, 0, 0x7D, 0)
    with pytest.raises(RuntimeError, match="health"):
        preflight.validate_preflight_status(4, 2, 2, 0x5D, 0)


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


def test_preflight_retries_hello_until_capabilities(monkeypatch) -> None:
    sent_sequences: list[int] = []
    responses = iter(
        [
            TimeoutError("ESP32 still booting"),
            types.SimpleNamespace(payload=bytes(12)),
        ]
    )

    def fake_write_frame(_port, message_type, sequence, payload=b""):
        assert message_type == preflight.HELLO
        assert payload == b""
        sent_sequences.append(sequence)
        return sequence + 1

    def fake_wait_for(_decoder, _port, message_type, _timeout):
        assert message_type == preflight.CAPABILITIES
        response = next(responses)
        if isinstance(response, Exception):
            raise response
        return response

    monkeypatch.setattr(preflight, "write_frame", fake_write_frame)
    monkeypatch.setattr(preflight, "wait_for", fake_wait_for)

    sequence, capabilities = preflight.wait_for_handshake(
        decoder=object(),
        port=object(),
        sequence=10,
        timeout=1.0,
    )

    assert sent_sequences == [10, 11]
    assert sequence == 12
    assert capabilities.payload == bytes(12)


def test_preflight_opens_serial_without_resetting_esp32() -> None:
    events: list[tuple[bool, bool, str | None]] = []
    fake_port = types.SimpleNamespace(dtr=True, rts=True, port=None)
    fake_port.open = lambda: events.append(
        (fake_port.dtr, fake_port.rts, fake_port.port)
    )
    factory_arguments: dict[str, object] = {}

    def serial_factory(**arguments):
        factory_arguments.update(arguments)
        return fake_port

    config = types.SimpleNamespace(
        serial=types.SimpleNamespace(
            port="/dev/serial/by-id/esp32",
            baud=115200,
            timeout_seconds=0.05,
        )
    )

    opened = preflight.open_serial_port(config, serial_factory=serial_factory)

    assert factory_arguments["port"] is None
    assert events == [(False, False, "/dev/serial/by-id/esp32")]
    assert opened is fake_port
    assert not opened.dtr
    assert not opened.rts


def test_wait_for_sequence_ignores_streamed_status_with_other_sequence() -> None:
    frames = [
        types.SimpleNamespace(message_type=preflight.STATUS, sequence=99),
        types.SimpleNamespace(message_type=preflight.STATUS, sequence=12),
    ]
    decoder = types.SimpleNamespace(feed=lambda _data: frames)
    port = types.SimpleNamespace(in_waiting=1, read=lambda _count: b"x")

    result = preflight.wait_for_sequence(
        decoder,
        port,
        preflight.STATUS,
        sequence=12,
        timeout=0.1,
    )

    assert result.sequence == 12

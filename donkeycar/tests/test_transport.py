import struct

import pytest

from trashcan_robot.config import SerialConfig
from trashcan_robot.protocol import (
    ARM,
    CAPABILITIES,
    CLEAR_FAULT,
    DISARM,
    HELLO,
    SET_OPERATING_MODE,
    SET_VELOCITY_YAW,
    STATUS,
    STOP,
    decode_message,
    encode_frame,
)
from trashcan_robot.simulation import FakeClock, SimulatedEsp32Serial, SimulatedSerialFactory
from trashcan_robot.transport import SerialMotorTransport


def config() -> SerialConfig:
    return SerialConfig(
        port="/dev/serial/by-id/simulated",
        baud=115200,
        timeout_seconds=0.05,
        reconnect_seconds=1.0,
        lease_ms=500,
        command_hz=20,
    )


def build_transport(endpoint: SimulatedEsp32Serial, clock: FakeClock):
    factory = SimulatedSerialFactory(endpoint)
    transport = SerialMotorTransport(
        config(),
        serial_factory=factory,
        clock=clock.monotonic,
        sleeper=clock.sleep,
        lease_id_factory=lambda: 1,
    )
    return factory, transport


class IncompatibleCapabilitiesEsp32(SimulatedEsp32Serial):
    def _queue_capabilities(self, sequence: int) -> None:
        payload = struct.pack("<BBBBHHHH", 1, 1, 0, 1 << 2, 100, 10, 48, 0)
        self._queue(CAPABILITIES, sequence, payload)


class ArmAcknowledgedButNotHeldEsp32(SimulatedEsp32Serial):
    def _process(self, frame) -> None:
        super()._process(frame)
        if frame.message_type == ARM:
            self._armed = False


def test_connect_sends_hello_mode_zero_and_arm_with_real_acknowledgments() -> None:
    clock = FakeClock()
    endpoint = SimulatedEsp32Serial(clock)
    _, transport = build_transport(endpoint, clock)

    transport.connect()

    assert transport.write_history[:10] == (
        HELLO,
        SET_OPERATING_MODE,
        SET_VELOCITY_YAW,
        CLEAR_FAULT,
        SET_VELOCITY_YAW,
        STATUS,
        SET_VELOCITY_YAW,
        ARM,
        SET_VELOCITY_YAW,
        STATUS,
    )
    assert transport.write_history.index(STATUS) < transport.write_history.index(
        ARM,
    )
    assert transport.write_history.index(ARM) < len(transport.write_history) - 1
    assert endpoint.armed
    assert transport.is_connected()


def test_incompatible_capabilities_never_arm_and_close_safely() -> None:
    clock = FakeClock()
    endpoint = IncompatibleCapabilitiesEsp32(clock)
    _, transport = build_transport(endpoint, clock)

    with pytest.raises(ConnectionError, match="capabilities are incompatible"):
        transport.connect()

    assert ARM not in transport.write_history
    assert STOP in transport.write_history
    assert DISARM in transport.write_history
    assert not endpoint.armed
    assert not endpoint.is_open
    assert not transport.is_connected()


def test_arm_ack_without_persistent_armed_status_fails_closed() -> None:
    clock = FakeClock()
    endpoint = ArmAcknowledgedButNotHeldEsp32(clock)
    _, transport = build_transport(endpoint, clock)

    with pytest.raises(ConnectionError, match="did not remain safely armed"):
        transport.connect()

    assert ARM in transport.write_history
    assert transport.write_history[-3:] == (SET_VELOCITY_YAW, STOP, DISARM)
    assert not endpoint.armed
    assert not endpoint.is_open
    assert not transport.is_connected()


def test_shutdown_attempts_zero_stop_and_disarm() -> None:
    clock = FakeClock()
    endpoint = SimulatedEsp32Serial(clock)
    _, transport = build_transport(endpoint, clock)
    transport.connect()

    transport.shutdown()

    assert transport.write_history[-3:] == (SET_VELOCITY_YAW, STOP, DISARM)
    assert not transport.is_connected()


def test_protocol_error_immediately_zeroes_disarms_and_closes_session() -> None:
    clock = FakeClock()
    endpoint = SimulatedEsp32Serial(clock)
    _, transport = build_transport(endpoint, clock)
    transport.connect()

    endpoint._queue_error(999, SET_VELOCITY_YAW, 6)
    decoded = [decode_message(frame) for frame in transport.read_telemetry()]

    assert any(message.get("name") == "error" for message in decoded)
    assert transport.write_history[-3:] == (SET_VELOCITY_YAW, STOP, DISARM)
    assert not endpoint.armed
    assert not endpoint.is_open
    assert not transport.is_connected()


def test_safety_fault_status_immediately_zeroes_disarms_and_closes_session() -> None:
    clock = FakeClock()
    endpoint = SimulatedEsp32Serial(clock)
    _, transport = build_transport(endpoint, clock)
    transport.connect()

    endpoint._faults = 2
    endpoint._queue_status(999)
    decoded = [decode_message(frame) for frame in transport.read_telemetry()]

    assert any(message.get("faults") == 2 for message in decoded)
    assert transport.write_history[-3:] == (SET_VELOCITY_YAW, STOP, DISARM)
    assert not endpoint.armed
    assert not endpoint.is_open
    assert not transport.is_connected()


def test_corrupted_telemetry_immediately_zeroes_disarms_and_closes_session() -> None:
    clock = FakeClock()
    endpoint = SimulatedEsp32Serial(clock)
    _, transport = build_transport(endpoint, clock)
    transport.connect()

    corrupted = bytearray(encode_frame(STATUS, 0xBEEF))
    corrupted[-1] ^= 0x01
    endpoint._input.extend(corrupted)
    transport.read_telemetry()

    assert transport.write_history[-3:] == (SET_VELOCITY_YAW, STOP, DISARM)
    assert not endpoint.armed
    assert not endpoint.is_open
    assert not transport.is_connected()


def test_repeated_connect_shutdown_cycles_remain_zero_and_recoverable() -> None:
    clock = FakeClock()
    endpoint = SimulatedEsp32Serial(clock)
    factory, transport = build_transport(endpoint, clock)

    for _ in range(50):
        transport.connect()
        assert transport.is_connected()
        assert endpoint.armed
        transport.shutdown()
        assert not transport.is_connected()
        assert not endpoint.armed

    assert factory.open_attempts == 50
    assert all(command[1:3] == (0, 0) for command in endpoint.gd32.commands)


def test_command_renewal_bridges_short_jitter_but_not_stale_input() -> None:
    clock = FakeClock()
    endpoint = SimulatedEsp32Serial(clock)
    _, transport = build_transport(endpoint, clock)
    transport.connect()
    transport.send_command(0.0, 0.8)
    writes_after_command = len(transport.write_history)

    clock.advance(0.06)
    assert transport._renew_command_once()
    assert len(transport.write_history) == writes_after_command + 1

    clock.advance(0.20)
    writes_before_stale_attempt = len(transport.write_history)
    assert not transport._renew_command_once()
    assert len(transport.write_history) == writes_before_stale_attempt

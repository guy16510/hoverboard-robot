from trashcan_robot.config import SerialConfig
from trashcan_robot.protocol import (
    ARM,
    CLEAR_FAULT,
    DISARM,
    HELLO,
    SET_OPERATING_MODE,
    SET_VELOCITY_YAW,
    STATUS,
    STOP,
    decode_message,
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


def test_connect_sends_hello_mode_zero_and_arm_with_real_acknowledgments() -> None:
    clock = FakeClock()
    endpoint = SimulatedEsp32Serial(clock)
    transport = SerialMotorTransport(
        config(),
        serial_factory=SimulatedSerialFactory(endpoint),
        clock=clock.monotonic,
        sleeper=clock.sleep,
        lease_id_factory=lambda: 1,
    )

    transport.connect()

    assert transport.write_history[:7] == (
        HELLO,
        SET_OPERATING_MODE,
        SET_VELOCITY_YAW,
        CLEAR_FAULT,
        SET_VELOCITY_YAW,
        STATUS,
        SET_VELOCITY_YAW,
    )
    assert transport.write_history[7] == ARM
    assert transport.write_history.index(STATUS) < transport.write_history.index(
        ARM,
    )
    assert endpoint.armed


def test_shutdown_attempts_zero_stop_and_disarm() -> None:
    clock = FakeClock()
    endpoint = SimulatedEsp32Serial(clock)
    transport = SerialMotorTransport(
        config(),
        serial_factory=SimulatedSerialFactory(endpoint),
        clock=clock.monotonic,
        sleeper=clock.sleep,
        lease_id_factory=lambda: 1,
    )
    transport.connect()

    transport.shutdown()

    assert transport.write_history[-3:] == (SET_VELOCITY_YAW, STOP, DISARM)
    assert not transport.is_connected()


def test_protocol_error_invalidates_open_serial_session() -> None:
    clock = FakeClock()
    endpoint = SimulatedEsp32Serial(clock)
    transport = SerialMotorTransport(
        config(),
        serial_factory=SimulatedSerialFactory(endpoint),
        clock=clock.monotonic,
        sleeper=clock.sleep,
        lease_id_factory=lambda: 1,
    )
    transport.connect()

    endpoint._queue_error(999, SET_VELOCITY_YAW, 6)
    decoded = [decode_message(frame) for frame in transport.read_telemetry()]

    assert any(message.get("name") == "error" for message in decoded)
    assert not transport.is_connected()


def test_safety_fault_status_invalidates_open_serial_session() -> None:
    clock = FakeClock()
    endpoint = SimulatedEsp32Serial(clock)
    transport = SerialMotorTransport(
        config(),
        serial_factory=SimulatedSerialFactory(endpoint),
        clock=clock.monotonic,
        sleeper=clock.sleep,
        lease_id_factory=lambda: 1,
    )
    transport.connect()

    endpoint._faults = 2
    endpoint._queue_status(999)
    decoded = [decode_message(frame) for frame in transport.read_telemetry()]

    assert any(message.get("faults") == 2 for message in decoded)
    assert not transport.is_connected()


def test_command_renewal_bridges_short_jitter_but_not_stale_input() -> None:
    clock = FakeClock()
    endpoint = SimulatedEsp32Serial(clock)
    transport = SerialMotorTransport(
        config(),
        serial_factory=SimulatedSerialFactory(endpoint),
        clock=clock.monotonic,
        sleeper=clock.sleep,
        lease_id_factory=lambda: 1,
    )
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

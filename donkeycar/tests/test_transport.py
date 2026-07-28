from trashcan_robot.config import SerialConfig
from trashcan_robot.protocol import ARM, DISARM, HELLO, SET_OPERATING_MODE, SET_VELOCITY_YAW, STOP
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

    assert transport.write_history[:4] == (
        HELLO,
        SET_OPERATING_MODE,
        SET_VELOCITY_YAW,
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

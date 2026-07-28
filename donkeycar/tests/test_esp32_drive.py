from trashcan_robot.config import LimitsConfig
from trashcan_robot.esp32_drive import ESP32Drive
from trashcan_robot.transport import MockMotorTransport


def limits() -> LimitsConfig:
    return LimitsConfig(
        max_linear_velocity=0.35,
        max_angular_velocity=0.8,
        throttle_deadband=0.03,
        steering_deadband=0.03,
    )


def test_drive_converts_normalized_commands() -> None:
    transport = MockMotorTransport()
    drive = ESP32Drive(transport, limits(), reconnect_seconds=0)
    drive.run(0.0, 0.0)
    connected, linear, angular, latency, fault = drive.run(0.5, -0.25)
    assert connected is True
    assert linear == 0.175
    assert angular == -0.2
    assert latency == 0.1
    assert fault is None


def test_connection_requires_neutral_input_before_motion() -> None:
    transport = MockMotorTransport()
    drive = ESP32Drive(transport, limits(), reconnect_seconds=0)

    connected, linear, angular, _, fault = drive.run(0.5, -0.25)

    assert connected is True
    assert (linear, angular) == (0.0, 0.0)
    assert fault == "waiting for neutral input after ESP32 connection"
    assert transport.commands[-1]["linear_velocity"] == 0.0
    assert transport.commands[-1]["angular_velocity"] == 0.0

    drive.run(0.0, 0.0)
    connected, linear, angular, _, fault = drive.run(0.5, -0.25)

    assert connected is True
    assert (linear, angular) == (0.175, -0.2)
    assert fault is None


def test_reconnect_requires_neutral_input_again() -> None:
    transport = MockMotorTransport()
    drive = ESP32Drive(transport, limits(), reconnect_seconds=0)
    drive.run(0.0, 0.0)
    drive.run(0.5, 0.25)
    transport.disconnect()

    connected, linear, angular, _, fault = drive.run(0.5, 0.25)
    assert connected is False
    assert (linear, angular) == (0.0, 0.0)
    assert "waiting to reconnect" in fault

    connected, linear, angular, _, fault = drive.run(0.5, 0.25)
    assert connected is True
    assert (linear, angular) == (0.0, 0.0)
    assert fault == "waiting for neutral input after ESP32 connection"


def test_missing_serial_device_keeps_zero_output() -> None:
    class MissingTransport(MockMotorTransport):
        def connect(self) -> None:
            raise FileNotFoundError("/dev/ttyACM0 is absent")

    drive = ESP32Drive(MissingTransport(), limits(), reconnect_seconds=0)

    connected, linear, angular, latency, fault = drive.run(0.0, 0.0)

    assert connected is False
    assert (linear, angular) == (0.0, 0.0)
    assert latency is None
    assert "/dev/ttyACM0 is absent" in fault


def test_drive_commands_zero_when_transport_fails() -> None:
    class FailingTransport(MockMotorTransport):
        def send_command(self, linear_velocity: float, angular_velocity: float) -> float:
            raise OSError("cable removed")

    drive = ESP32Drive(FailingTransport(), limits(), reconnect_seconds=0)
    connected, linear, angular, latency, fault = drive.run(1.0, 1.0)
    assert connected is False
    assert linear == 0.0
    assert angular == 0.0
    assert latency is None
    assert "cable removed" in fault


def test_shutdown_sends_zero() -> None:
    transport = MockMotorTransport()
    drive = ESP32Drive(transport, limits(), reconnect_seconds=0)
    drive.run(0.5, 0.5)
    drive.shutdown()
    assert transport.commands[-1]["linear_velocity"] == 0.0
    assert transport.commands[-1]["angular_velocity"] == 0.0
    assert transport.connected is False

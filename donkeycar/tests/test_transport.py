from trashcan_robot.config import SerialConfig
from trashcan_robot.protocol import (
    ARM,
    HELLO,
    SET_OPERATING_MODE,
    SET_VELOCITY_YAW,
    FrameDecoder,
)
from trashcan_robot.transport import SerialMotorTransport


def test_connect_sends_zero_demand_before_every_arm_attempt(monkeypatch) -> None:
    writes: list[bytes] = []

    class FakeSerial:
        is_open = True
        in_waiting = 0

        def reset_input_buffer(self) -> None:
            return

        def write(self, data: bytes) -> None:
            writes.append(data)

        def flush(self) -> None:
            return

        def close(self) -> None:
            self.is_open = False

    monkeypatch.setattr(
        "trashcan_robot.transport.serial.Serial",
        lambda *args, **kwargs: FakeSerial(),
    )
    monkeypatch.setattr("trashcan_robot.transport.time.sleep", lambda _: None)
    transport = SerialMotorTransport(
        SerialConfig(
            port="/dev/ttyACM0",
            baud=115200,
            timeout_seconds=0.05,
            reconnect_seconds=1.0,
            lease_ms=500,
            command_hz=20,
        )
    )

    transport.connect()

    frames = FrameDecoder().feed(b"".join(writes))
    message_types = [frame.message_type for frame in frames]
    assert message_types[:2] == [HELLO, SET_OPERATING_MODE]
    assert message_types.count(ARM) == 20
    assert all(
        message_types[index - 1] == SET_VELOCITY_YAW
        for index, message_type in enumerate(message_types)
        if message_type == ARM
    )

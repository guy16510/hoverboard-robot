import struct

import pytest

from trashcan_robot.config import SerialConfig
from trashcan_robot.protocol import (
    ARM,
    HELLO,
    SET_OPERATING_MODE,
    SET_VELOCITY_YAW,
    ULTRASONIC,
    FrameDecoder,
    encode_frame,
)
from trashcan_robot.transport import SerialMotorTransport


def make_config(port: str = "/dev/ttyACM0") -> SerialConfig:
    return SerialConfig(
        port=port,
        baud=115200,
        timeout_seconds=0.05,
        reconnect_seconds=1.0,
        lease_ms=500,
        command_hz=20,
    )


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
    transport = SerialMotorTransport(make_config())

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


def test_transport_caches_ultrasonic_frames(monkeypatch) -> None:
    instances: list[object] = []

    class FakeSerial:
        def __init__(self) -> None:
            self.is_open = True
            self.rx = bytearray()
            instances.append(self)

        @property
        def in_waiting(self) -> int:
            return len(self.rx)

        def reset_input_buffer(self) -> None:
            self.rx.clear()

        def write(self, data: bytes) -> None:
            return

        def read(self, count: int) -> bytes:
            data = bytes(self.rx[:count])
            del self.rx[:count]
            return data

        def flush(self) -> None:
            return

        def close(self) -> None:
            self.is_open = False

    monkeypatch.setattr(
        "trashcan_robot.transport.serial.Serial",
        lambda *args, **kwargs: FakeSerial(),
    )
    monkeypatch.setattr("trashcan_robot.transport.time.sleep", lambda _: None)
    transport = SerialMotorTransport(make_config())
    transport.connect()

    fake = instances[0]
    assert isinstance(fake, FakeSerial)
    payload = struct.pack("<HHHBB", 500, 750, 1250, 0b111, 0)
    fake.rx.extend(encode_frame(ULTRASONIC, 123, payload))

    transport.send_command(0.1, 0.0)
    reading = transport.latest_ultrasonic()
    assert reading.front_m == pytest.approx(0.5)
    assert reading.left_m == pytest.approx(0.75)
    assert reading.right_m == pytest.approx(1.25)

    telemetry = transport.read_telemetry()
    assert any(frame.message_type == ULTRASONIC for frame in telemetry)


def test_auto_port_prefers_stable_by_id_path(monkeypatch) -> None:
    def fake_glob(pattern: str) -> list[str]:
        if pattern == "/dev/serial/by-id/*":
            return ["/dev/serial/by-id/usb-ESP32"]
        if pattern == "/dev/ttyUSB*":
            return ["/dev/ttyUSB0"]
        return []

    monkeypatch.setattr("trashcan_robot.transport.glob.glob", fake_glob)
    assert SerialMotorTransport._resolve_port("auto") == "/dev/serial/by-id/usb-ESP32"

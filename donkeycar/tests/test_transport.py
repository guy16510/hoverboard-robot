import struct

import pytest

from trashcan_robot.config import SerialConfig
from trashcan_robot.protocol import (
    ACK,
    ARM,
    CAPABILITIES,
    DRIVE_MODE,
    ERROR,
    HELLO,
    MAX_PAYLOAD,
    SET_OPERATING_MODE,
    SET_VELOCITY_YAW,
    ULTRASONIC,
    VERSION,
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


class RespondingSerial:
    def __init__(self, fail_message_type: int | None = None) -> None:
        self.is_open = True
        self.rx = bytearray()
        self.writes: list[bytes] = []
        self.fail_message_type = fail_message_type

    @property
    def in_waiting(self) -> int:
        return len(self.rx)

    def reset_input_buffer(self) -> None:
        self.rx.clear()

    def write(self, data: bytes) -> None:
        self.writes.append(data)
        frame = FrameDecoder().feed(data)[0]
        if frame.message_type == self.fail_message_type:
            payload = struct.pack("<BBH", frame.message_type, 6, DRIVE_MODE)
            self.rx.extend(encode_frame(ERROR, frame.sequence, payload))
            return
        if frame.message_type == HELLO:
            payload = struct.pack(
                "<BBBBHHHH",
                VERSION,
                0,
                0,
                1 << DRIVE_MODE,
                50,
                50,
                MAX_PAYLOAD,
                0,
            )
            self.rx.extend(encode_frame(CAPABILITIES, frame.sequence, payload))
            return
        self.rx.extend(
            encode_frame(ACK, frame.sequence, bytes((frame.message_type, 0)))
        )

    def read(self, count: int) -> bytes:
        data = bytes(self.rx[:count])
        del self.rx[:count]
        return data

    def flush(self) -> None:
        return

    def close(self) -> None:
        self.is_open = False


def install_serial(monkeypatch, fake: RespondingSerial) -> None:
    monkeypatch.setattr(
        "trashcan_robot.transport.serial.Serial",
        lambda *args, **kwargs: fake,
    )


def written_message_types(fake: RespondingSerial) -> list[int]:
    frames = FrameDecoder().feed(b"".join(fake.writes))
    return [frame.message_type for frame in frames]


def test_connect_requires_capabilities_mode_zero_lease_and_arm_ack(monkeypatch) -> None:
    fake = RespondingSerial()
    install_serial(monkeypatch, fake)
    transport = SerialMotorTransport(make_config())

    transport.connect()

    assert transport.is_connected()
    assert written_message_types(fake)[:4] == [
        HELLO,
        SET_OPERATING_MODE,
        SET_VELOCITY_YAW,
        ARM,
    ]


def test_connect_fails_if_esp32_rejects_arm(monkeypatch) -> None:
    fake = RespondingSerial(fail_message_type=ARM)
    install_serial(monkeypatch, fake)
    monkeypatch.setattr("trashcan_robot.transport.time.sleep", lambda _: None)
    transport = SerialMotorTransport(make_config())

    with pytest.raises(ConnectionError, match="handshake failed"):
        transport.connect()

    assert not transport.is_connected()


def test_send_command_waits_for_matching_ack(monkeypatch) -> None:
    fake = RespondingSerial()
    install_serial(monkeypatch, fake)
    transport = SerialMotorTransport(make_config())
    transport.connect()

    latency = transport.send_command(0.1, -0.2)

    assert latency >= 0.0
    assert written_message_types(fake)[-1] == SET_VELOCITY_YAW


def test_transport_caches_ultrasonic_frames(monkeypatch) -> None:
    fake = RespondingSerial()
    install_serial(monkeypatch, fake)
    transport = SerialMotorTransport(make_config())
    transport.connect()

    payload = struct.pack("<HHHBB", 500, 750, 1250, 0b111, 0)
    fake.rx.extend(encode_frame(ULTRASONIC, 123, payload))

    transport.send_command(0.1, 0.0)
    reading = transport.latest_ultrasonic()
    assert reading.front_m == pytest.approx(0.5)
    assert reading.left_m == pytest.approx(0.75)
    assert reading.right_m == pytest.approx(1.25)

    telemetry = transport.read_telemetry()
    assert any(frame.message_type == ULTRASONIC for frame in telemetry)


def test_stale_ultrasonic_is_not_reused_forever(monkeypatch) -> None:
    fake = RespondingSerial()
    install_serial(monkeypatch, fake)
    transport = SerialMotorTransport(make_config())
    transport.connect()

    payload = struct.pack("<HHHBB", 500, 750, 1250, 0b111, 0)
    fake.rx.extend(encode_frame(ULTRASONIC, 123, payload))
    transport.send_command(0.0, 0.0)
    assert transport.latest_ultrasonic().front_m == pytest.approx(0.5)

    transport._ultrasonic_updated_at = 0.0
    assert transport.latest_ultrasonic().front_m is None


def test_auto_port_prefers_stable_by_id_path(monkeypatch) -> None:
    def fake_glob(pattern: str) -> list[str]:
        if pattern == "/dev/serial/by-id/*":
            return ["/dev/serial/by-id/usb-ESP32"]
        if pattern == "/dev/ttyUSB*":
            return ["/dev/ttyUSB0"]
        return []

    monkeypatch.setattr("trashcan_robot.transport.glob.glob", fake_glob)
    assert SerialMotorTransport._resolve_port("auto") == "/dev/serial/by-id/usb-ESP32"

from __future__ import annotations

from pathlib import Path

import pytest

from trashcan_robot.config import LimitsConfig, SerialConfig, load_config
from trashcan_robot.esp32_drive import ESP32Drive
from trashcan_robot.pipeline import DriveMode
from trashcan_robot.protocol import (
    ARM,
    ERROR,
    HELLO,
    ODOMETRY,
    SET_OPERATING_MODE,
    SET_VELOCITY_YAW,
    STOP,
    FrameDecoder,
    decode_message,
    encode_frame,
    encode_motion,
)
from trashcan_robot.simulation import (
    FAULT_ACK_TIMEOUT,
    FAULT_FEEDBACK_CRC,
    FAULT_FEEDBACK_LOST,
    FAULT_LEASE_EXPIRED,
    FAULT_LOCAL_DISARM,
    FAULT_MALFORMED_COMMAND,
    FAULT_MASTER,
    FAULT_SLAVE,
    FAULT_UNSAFE_ORIENTATION,
    FakeClock,
    SimulatedEsp32Serial,
    SimulatedGd32Boundary,
    SimulatedMpu6050,
    SimulatedSerialFactory,
)
from trashcan_robot.transport import SerialMotorTransport


def serial_config() -> SerialConfig:
    return SerialConfig(
        port="/dev/serial/by-id/simulated-esp32",
        baud=115200,
        timeout_seconds=0.05,
        reconnect_seconds=0.1,
        lease_ms=500,
        command_hz=20,
    )


def limits() -> LimitsConfig:
    return LimitsConfig(
        max_linear_velocity=0.35,
        max_angular_velocity=0.8,
        throttle_deadband=0.03,
        steering_deadband=0.03,
    )


def build_stack(*, available: bool = True):
    clock = FakeClock()
    gd32 = SimulatedGd32Boundary(clock)
    mpu = SimulatedMpu6050(clock)
    endpoint = SimulatedEsp32Serial(clock, gd32, mpu)
    factory = SimulatedSerialFactory(endpoint, available=available)
    transport = SerialMotorTransport(
        serial_config(),
        serial_factory=factory,
        clock=clock.monotonic,
        sleeper=clock.sleep,
        lease_id_factory=lambda: 0x12345678,
    )
    return clock, gd32, mpu, endpoint, factory, transport


def drain(endpoint: SimulatedEsp32Serial):
    return FrameDecoder().feed(endpoint.read(endpoint.in_waiting))


def assert_disabled(endpoint: SimulatedEsp32Serial, gd32: SimulatedGd32Boundary) -> None:
    assert endpoint.armed is False
    assert endpoint.commanded == (0, 0)
    assert gd32.commands[-1][3] is True
    assert gd32.commands[-1][1:3] == (0, 0)


def test_safe_handshake_uses_real_binary_protocol_and_arms_only_after_zero_ack() -> None:
    _, gd32, _, endpoint, _, transport = build_stack()
    transport.connect()

    assert endpoint.armed is True
    assert endpoint.event_log[:4] == [
        "HELLO",
        "MODE_2",
        "MOVE:0:0",
        "CLEAR_FAULT",
    ]
    arm_index = endpoint.event_log.index("ARM")
    assert endpoint.event_log[arm_index - 1] == "MOVE:0:0"
    assert all(command[3] for command in gd32.commands[: arm_index + 1])
    assert all(abs(command[1]) <= 250 and abs(command[2]) <= 250 for command in gd32.commands)


def test_arm_is_rejected_for_non_neutral_input() -> None:
    endpoint = SimulatedEsp32Serial()
    endpoint.write(encode_frame(HELLO, 0))
    endpoint.write(encode_frame(SET_OPERATING_MODE, 1, b"\x02"))
    endpoint.write(encode_frame(SET_VELOCITY_YAW, 2, encode_motion(0.2, 0.0, 7, 500)))
    endpoint.write(encode_frame(ARM, 3))

    frames = drain(endpoint)
    arm_response = [frame for frame in frames if frame.sequence == 3][-1]
    assert arm_response.message_type == ERROR
    assert endpoint.armed is False
    assert all(command[1:3] == (0, 0) for command in endpoint.gd32.commands)


def test_straight_steering_combined_and_slew_are_bounded() -> None:
    clock, gd32, _, endpoint, _, transport = build_stack()
    transport.connect()
    transport.send_command(0.35, 0.0)
    first_left, first_right = endpoint.commanded
    assert (first_left, first_right) == (5, 5)
    endpoint._on_time(clock.monotonic())
    second_left, second_right = endpoint.commanded
    assert second_left - first_left == 5
    assert second_right - first_right == 5
    endpoint.advance(0.28)
    left, right = endpoint.commanded
    assert left == right
    assert 0 < left <= 250

    transport.send_command(0.0, 0.8)
    endpoint.advance(0.30)
    left, right = endpoint.commanded
    assert left > right
    assert max(abs(left), abs(right)) <= 250

    transport.send_command(0.35, 0.8)
    endpoint.advance(0.30)
    assert max(abs(value) for value in endpoint.commanded) <= 250
    assert all(abs(command[1]) <= 250 and abs(command[2]) <= 250 for command in gd32.commands)
    assert clock.monotonic() > 0


def test_web_input_flows_through_drive_mode_and_serial_lease() -> None:
    clock, gd32, _, endpoint, _, transport = build_stack()
    drive = ESP32Drive(transport, limits(), reconnect_seconds=0, clock=clock.monotonic)
    mode = DriveMode()

    angle, throttle, label = mode.run("user", 0.0, 0.0, 0.0, 0.0, False)
    assert label == "Manual"
    drive.run(throttle, angle)
    angle, throttle, _ = mode.run("user", 0.25, 0.5, 0.0, 0.0, True)
    connected, linear, angular, _, fault = drive.run(throttle, angle)
    endpoint.advance(0.30)

    assert connected and fault is None
    assert linear == pytest.approx(0.175)
    assert angular == pytest.approx(0.2)
    assert gd32.applied_left > gd32.applied_right
    assert endpoint.armed


def test_releasing_throttle_commands_zero() -> None:
    _, gd32, _, endpoint, _, transport = build_stack()
    transport.connect()
    transport.send_command(0.35, 0.0)
    endpoint.advance(0.30)
    assert gd32.applied_left != 0
    transport.send_command(0.0, 0.0)
    endpoint.advance(0.60)
    assert gd32.applied_left == 0
    assert gd32.applied_right == 0


def test_lease_expiration_stops_and_disarms() -> None:
    _, gd32, _, endpoint, _, transport = build_stack()
    transport.connect()
    transport.send_command(0.2, 0.0)
    endpoint.advance(0.60)
    assert endpoint.faults & FAULT_LEASE_EXPIRED
    assert_disabled(endpoint, gd32)


@pytest.mark.parametrize(
    ("inject", "fault"),
    [
        (lambda endpoint, gd32, mpu: endpoint.inject_local_disarm(), FAULT_LOCAL_DISARM),
        (lambda endpoint, gd32, mpu: endpoint.inject_master_fault(), FAULT_MASTER),
        (lambda endpoint, gd32, mpu: endpoint.inject_slave_fault(), FAULT_SLAVE),
        (lambda endpoint, gd32, mpu: endpoint.inject_feedback_crc(), FAULT_FEEDBACK_CRC),
        (lambda endpoint, gd32, mpu: mpu.set_pitch(50.0), FAULT_UNSAFE_ORIENTATION),
        (lambda endpoint, gd32, mpu: mpu.set_roll(-50.0), FAULT_UNSAFE_ORIENTATION),
        (lambda endpoint, gd32, mpu: mpu.invalid(), None),
        (lambda endpoint, gd32, mpu: mpu.missing(), None),
    ],
)
def test_fault_injection_zeroes_and_disarms(inject, fault) -> None:
    _, gd32, mpu, endpoint, _, transport = build_stack()
    transport.connect()
    transport.send_command(0.2, 0.1)
    endpoint.advance(0.20)
    inject(endpoint, gd32, mpu)
    endpoint.advance(0.02)

    assert_disabled(endpoint, gd32)
    if fault is not None:
        assert endpoint.faults & fault


def test_feedback_timeout_zeroes_and_disarms() -> None:
    _, gd32, _, endpoint, _, transport = build_stack()
    transport.connect()
    transport.send_command(0.2, 0.0)
    endpoint.advance(0.20)
    gd32.feedback_enabled = False
    endpoint.advance(0.60)

    assert endpoint.faults & (FAULT_FEEDBACK_LOST | FAULT_ACK_TIMEOUT)
    assert_disabled(endpoint, gd32)


def test_ack_delay_drop_stale_and_timeout_paths() -> None:
    clock, gd32, _, endpoint, _, transport = build_stack()
    transport.connect()

    gd32.ack_delay_seconds = 0.20
    transport.send_command(0.2, 0.0)
    endpoint.advance(0.10)
    delayed_sequence = endpoint._in_flight_sequence
    assert not gd32.exact_ack(delayed_sequence)
    endpoint.advance(0.20)
    assert gd32.exact_ack(delayed_sequence) or endpoint._in_flight_sequence != delayed_sequence

    gd32.drop_next_ack = True
    transport.send_command(0.25, 0.0)
    endpoint.advance(0.11)
    assert gd32.pending or not gd32.exact_ack(endpoint._in_flight_sequence)

    gd32.stale_next_ack = True
    transport.send_command(0.30, 0.0)
    endpoint.advance(0.21)
    assert gd32.accepted_esp_sequence != endpoint._in_flight_sequence or gd32.pending

    endpoint._last_motor_at = clock.monotonic()
    endpoint._in_flight_started_at = clock.monotonic() - 1.0
    gd32.accepted_esp_sequence = (endpoint._in_flight_sequence - 1) & 0xFFFF
    gd32.forwarded_slave_sequence = gd32.accepted_esp_sequence
    gd32.accepted_slave_sequence = gd32.accepted_esp_sequence
    endpoint.advance(0.01)
    assert endpoint.faults & FAULT_ACK_TIMEOUT
    assert_disabled(endpoint, gd32)


def test_malformed_crc_stale_and_duplicate_pi_commands_are_rejected() -> None:
    endpoint = SimulatedEsp32Serial()
    endpoint.write(encode_frame(HELLO, 10))
    endpoint.write(encode_frame(SET_OPERATING_MODE, 11, b"\x02"))
    duplicate = encode_frame(SET_VELOCITY_YAW, 12, encode_motion(0.0, 0.0, 1, 500))
    endpoint.write(duplicate)
    endpoint.write(duplicate)
    frames = drain(endpoint)
    duplicate_response = [frame for frame in frames if frame.sequence == 12][-1]
    assert duplicate_response.message_type == ERROR
    assert endpoint.faults & FAULT_MALFORMED_COMMAND

    corrupted = bytearray(encode_frame(STOP, 13))
    corrupted[-1] ^= 0x01
    endpoint.write(bytes(corrupted))
    assert endpoint.faults & FAULT_MALFORMED_COMMAND
    assert_disabled(endpoint, endpoint.gd32)


def test_stop_disarm_and_shutdown_attempt_zero_stop_disarm() -> None:
    _, gd32, _, endpoint, _, transport = build_stack()
    transport.connect()
    transport.send_command(0.2, 0.0)
    endpoint.advance(0.20)
    transport.shutdown()

    shutdown_events = endpoint.event_log[-4:]
    assert shutdown_events[0].startswith("MOVE:0:0")
    assert "STOP" in shutdown_events
    assert "DISARM" in shutdown_events
    assert shutdown_events[-1] == "SERIAL_DISCONNECT"
    assert_disabled(endpoint, gd32)


def test_disconnect_reconnect_requires_hello_mode_zero_and_neutral_again() -> None:
    clock, gd32, _, endpoint, factory, transport = build_stack()
    drive = ESP32Drive(transport, limits(), reconnect_seconds=0.1, clock=clock.monotonic)
    drive.run(0.0, 0.0)
    drive.run(0.5, 0.0)
    endpoint.advance(0.20)
    assert gd32.applied_left != 0

    endpoint.inject_usb_disconnect()
    connected, linear, angular, _, _ = drive.run(0.5, 0.0)
    assert not connected and (linear, angular) == (0.0, 0.0)
    factory.available = True
    clock.advance(0.11)
    connected, linear, angular, _, fault = drive.run(0.5, 0.0)
    assert connected and (linear, angular) == (0.0, 0.0)
    assert fault == "waiting for neutral input after ESP32 connection"
    assert endpoint.event_log.count("HELLO") == 2
    second_hello = len(endpoint.event_log) - 1 - endpoint.event_log[::-1].index("HELLO")
    reconnect_events = endpoint.event_log[second_hello:]
    assert reconnect_events[:4] == [
        "HELLO",
        "MODE_2",
        "MOVE:0:0",
        "CLEAR_FAULT",
    ]
    assert reconnect_events[reconnect_events.index("ARM") - 1] == "MOVE:0:0"
    assert gd32.applied_left == 0 and gd32.applied_right == 0


def test_application_starts_absent_then_connects_when_device_appears() -> None:
    clock, _, _, endpoint, factory, transport = build_stack(available=False)
    drive = ESP32Drive(transport, limits(), reconnect_seconds=0.1, clock=clock.monotonic)

    connected, linear, angular, _, fault = drive.run(0.0, 0.0)
    assert not connected and (linear, angular) == (0.0, 0.0)
    assert "absent" in fault

    factory.available = True
    clock.advance(0.11)
    connected, linear, angular, _, fault = drive.run(0.0, 0.0)
    assert connected and (linear, angular) == (0.0, 0.0)
    assert fault is None
    assert endpoint.armed


def test_telemetry_streams_while_commands_are_active() -> None:
    _, _, _, endpoint, _, transport = build_stack()
    transport.connect()
    transport.send_command(0.2, 0.1)
    endpoint.advance(0.30)
    frames = transport.read_telemetry()
    decoded = [decode_message(frame) for frame in frames]
    names = {message["name"] for message in decoded}
    assert {"status", "drive", "motor", "odometry", "faults"} <= names
    drive = [message for message in decoded if message["name"] == "drive"][-1]
    assert drive["requested_linear"] == pytest.approx(0.2)
    assert max(abs(value) for value in drive["commanded"]) <= 250


def test_both_motor_one_motor_failure_and_odometry_disagreement() -> None:
    _, gd32, _, endpoint, _, transport = build_stack()
    transport.connect()
    transport.send_command(0.2, 0.0)
    endpoint.advance(0.30)
    assert gd32.applied_left == gd32.applied_right != 0

    gd32.right_responds = False
    transport.send_command(0.25, 0.0)
    endpoint.advance(0.30)
    assert gd32.applied_left != 0 and gd32.applied_right == 0

    gd32.force_odometry_disagreement(100, -50)
    endpoint.advance(0.11)
    odometry = [
        decode_message(frame)
        for frame in transport.read_telemetry()
        if frame.message_type == ODOMETRY
    ][-1]
    assert odometry["left"] != odometry["right"]


def test_web_cannot_bypass_latched_esp32_fault() -> None:
    clock, gd32, _, endpoint, _, transport = build_stack()
    drive = ESP32Drive(transport, limits(), reconnect_seconds=0, clock=clock.monotonic)
    drive.run(0.0, 0.0)
    drive.run(0.5, 0.0)
    endpoint.advance(0.20)
    endpoint.inject_master_fault()
    endpoint.advance(0.01)
    assert_disabled(endpoint, gd32)

    for _ in range(5):
        drive.run(1.0, 1.0)
        endpoint.advance(0.02)
    assert endpoint.faults & FAULT_MASTER
    assert_disabled(endpoint, gd32)


def test_production_config_uses_linux_by_id_and_explicit_ports(monkeypatch) -> None:
    config_path = Path(__file__).parents[1] / "config" / "robot.yaml"
    config = load_config(config_path)
    assert config.serial.port.startswith("/dev/serial/by-id/")
    assert config.serial.baud == 115200
    assert config.raw["controller"] == {"web_host": "0.0.0.0", "web_port": 8887}
    assert config.raw["dashboard"] == {"host": "0.0.0.0", "port": 8888}

    monkeypatch.setenv("TRASHCAN_SERIAL_PORT", "/dev/serial/by-id/custom-controller")
    assert load_config(config_path).serial.port == "/dev/serial/by-id/custom-controller"


def test_systemd_unit_has_no_serial_device_dependency() -> None:
    unit = (Path(__file__).parents[1] / "systemd" / "trashcan-donkeycar.service").read_text()
    assert "dev-tty" not in unit
    assert ".device" not in unit
    assert "Restart=always" in unit
    assert "EnvironmentFile=-/etc/default/trashcan-donkeycar" in unit

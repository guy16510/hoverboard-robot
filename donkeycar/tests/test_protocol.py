import struct

import pytest

from trashcan_robot.protocol import (
    ACK, CAPABILITIES, DRIVE_MODE, ERROR, HALL, IMU, LEFT_HALL, MAX_PAYLOAD,
    SET_SERVO, SET_VELOCITY_YAW, ULTRASONIC, VERSION, FrameDecoder,
    crc16_ccitt_false, decode_ack, decode_capabilities, decode_error, decode_hall,
    decode_imu, decode_ultrasonic, encode_frame, encode_motion, encode_servo,
)


def test_crc_check_value() -> None:
    assert crc16_ccitt_false(b"123456789") == 0x29B1


def test_frame_round_trip() -> None:
    payload = encode_motion(0.25, -0.5, 1234, 500)
    encoded = encode_frame(SET_VELOCITY_YAW, 42, payload)
    frames = FrameDecoder().feed(encoded)
    assert len(frames) == 1
    assert frames[0].message_type == SET_VELOCITY_YAW
    assert frames[0].sequence == 42
    assert frames[0].payload == payload


def test_servo_payload_is_clamped_and_encoded() -> None:
    assert struct.unpack("<h", encode_servo(0.5))[0] == 500
    assert struct.unpack("<h", encode_servo(4.0))[0] == 1000
    assert FrameDecoder().feed(encode_frame(SET_SERVO, 12, encode_servo(-0.25)))[0].message_type == SET_SERVO


def test_decoder_resynchronizes_after_noise() -> None:
    encoded = encode_frame(SET_VELOCITY_YAW, 7, b"abc")
    decoder = FrameDecoder()
    assert decoder.feed(b"debug noise\n" + encoded[:5]) == []
    frames = decoder.feed(encoded[5:])
    assert frames[0].sequence == 7


def test_capabilities_decode_matches_drive_contract() -> None:
    payload = struct.pack("<BBBBHHHH", VERSION, 0, 0, 1 << DRIVE_MODE, 50, 50, MAX_PAYLOAD, 0)
    capabilities = decode_capabilities(payload)
    assert capabilities.protocol_version == VERSION
    assert capabilities.supports_mode(DRIVE_MODE)
    assert capabilities.maximum_payload == MAX_PAYLOAD
    assert FrameDecoder().feed(encode_frame(CAPABILITIES, 3, payload))[0].message_type == CAPABILITIES


def test_ack_and_error_payloads_are_explicit() -> None:
    ack = decode_ack(struct.pack("<BB", SET_VELOCITY_YAW, 0))
    assert ack.request_type == SET_VELOCITY_YAW
    assert ack.status == 0
    error = decode_error(struct.pack("<BBH", SET_VELOCITY_YAW, 6, 2))
    assert error.request_type == SET_VELOCITY_YAW
    assert error.code == 6
    assert error.detail == 2
    assert ACK == 0x7E
    assert ERROR == 0x7F


def test_ultrasonic_payload_decodes_to_meters() -> None:
    payload = struct.pack("<HHHBB", 420, 1000, 0xFFFF, 0b011, 0)
    reading = decode_ultrasonic(payload)
    assert reading.front_m == pytest.approx(0.420)
    assert reading.left_m == pytest.approx(1.0)
    assert reading.right_m is None
    assert FrameDecoder().feed(encode_frame(ULTRASONIC, 9, payload))[0].message_type == ULTRASONIC


def test_imu_payload_decodes_units_and_validity() -> None:
    payload = struct.pack("<hhhhhhhH", 1000, -500, 980, 1250, -2500, 0, 3653, 1)
    reading = decode_imu(payload)
    assert reading.accel_x_g == pytest.approx(1.0)
    assert reading.accel_y_g == pytest.approx(-0.5)
    assert reading.gyro_x_dps == pytest.approx(1.25)
    assert reading.temperature_c == pytest.approx(36.53)
    assert reading.valid
    assert FrameDecoder().feed(encode_frame(IMU, 13, payload))[0].message_type == IMU


def test_hall_payload_decodes_for_either_wheel() -> None:
    payload = struct.pack("<BBHIIIII", 5, 0b11, 0, 1234, 2, 3, 12750, 42)
    reading = decode_hall(payload)
    assert reading.state == 5
    assert reading.valid
    assert reading.moving
    assert reading.transitions == 1234
    assert reading.invalid_states == 2
    assert reading.skipped_transitions == 3
    assert reading.transitions_per_second == pytest.approx(12.75)
    assert reading.last_transition_age_s == pytest.approx(0.042)
    assert FrameDecoder().feed(encode_frame(HALL, 10, payload))[0].message_type == HALL
    assert FrameDecoder().feed(encode_frame(LEFT_HALL, 11, payload))[0].message_type == LEFT_HALL


def test_hall_never_moved_uses_none_age() -> None:
    payload = struct.pack("<BBHIIIII", 1, 0b01, 0, 0, 0, 0, 0, 0xFFFFFFFF)
    reading = decode_hall(payload)
    assert reading.valid
    assert not reading.moving
    assert reading.last_transition_age_s is None


def test_protocol_decoders_reject_wrong_payload_lengths() -> None:
    for decoder in (decode_capabilities, decode_ack, decode_error, decode_ultrasonic, decode_imu, decode_hall):
        with pytest.raises(ValueError):
            decoder(b"x")

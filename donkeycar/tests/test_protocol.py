import struct

import pytest

from trashcan_robot.protocol import (
    ACK,
    CAPABILITIES,
    DRIVE_MODE,
    ERROR,
    MAX_PAYLOAD,
    SET_VELOCITY_YAW,
    ULTRASONIC,
    VERSION,
    FrameDecoder,
    crc16_ccitt_false,
    decode_ack,
    decode_capabilities,
    decode_error,
    decode_ultrasonic,
    encode_frame,
    encode_motion,
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


def test_decoder_resynchronizes_after_noise() -> None:
    encoded = encode_frame(SET_VELOCITY_YAW, 7, b"abc")
    decoder = FrameDecoder()
    assert decoder.feed(b"debug noise\n" + encoded[:5]) == []
    frames = decoder.feed(encoded[5:])
    assert frames[0].sequence == 7


def test_capabilities_decode_matches_drive_contract() -> None:
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
    capabilities = decode_capabilities(payload)
    assert capabilities.protocol_version == VERSION
    assert capabilities.supports_mode(DRIVE_MODE)
    assert capabilities.maximum_payload == MAX_PAYLOAD

    encoded = encode_frame(CAPABILITIES, 3, payload)
    assert FrameDecoder().feed(encoded)[0].message_type == CAPABILITIES


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

    encoded = encode_frame(ULTRASONIC, 9, payload)
    frame = FrameDecoder().feed(encoded)[0]
    assert frame.message_type == ULTRASONIC


def test_protocol_decoders_reject_wrong_payload_lengths() -> None:
    with pytest.raises(ValueError):
        decode_capabilities(b"short")
    with pytest.raises(ValueError):
        decode_ack(b"x")
    with pytest.raises(ValueError):
        decode_error(b"bad")
    with pytest.raises(ValueError):
        decode_ultrasonic(b"short")

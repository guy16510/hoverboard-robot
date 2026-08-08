import struct

import pytest

from trashcan_robot.protocol import (
    SET_VELOCITY_YAW,
    ULTRASONIC,
    FrameDecoder,
    crc16_ccitt_false,
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


def test_ultrasonic_payload_decodes_to_meters() -> None:
    payload = struct.pack("<HHHBB", 420, 1000, 0xFFFF, 0b011, 0)
    reading = decode_ultrasonic(payload)
    assert reading.front_m == pytest.approx(0.420)
    assert reading.left_m == pytest.approx(1.0)
    assert reading.right_m is None

    encoded = encode_frame(ULTRASONIC, 9, payload)
    frame = FrameDecoder().feed(encoded)[0]
    assert frame.message_type == ULTRASONIC


def test_ultrasonic_rejects_wrong_payload_length() -> None:
    with pytest.raises(ValueError):
        decode_ultrasonic(b"short")

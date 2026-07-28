import struct

from trashcan_robot.protocol import (
    CONTROLLER_TELEMETRY,
    FAULTS,
    SET_VELOCITY_YAW,
    Frame,
    FrameDecoder,
    crc16_ccitt_false,
    decode_message,
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


def test_fault_feedback_decodes_bridge_and_link_health_bits() -> None:
    payload = struct.pack("<IIII", 0, 0, 0, 0x00000405)

    decoded = decode_message(Frame(FAULTS, 0, 1, payload))

    assert decoded["controller_status_flags"] == 0x05
    assert decoded["motor_status_flags"] == 0x04
    assert decoded["peer_healthy"] is True
    assert decoded["left_bridge_enabled"] is False
    assert decoded["right_bridge_enabled"] is False
    assert decoded["slave_pa4_high"] is True


def test_controller_telemetry_decodes_raw_halls_and_link_ages() -> None:
    payload = struct.pack(
        "<BBBBHHHBBHHHHHH",
        2,
        2,
        0x05,
        0x04,
        7,
        8,
        9,
        3,
        6,
        111,
        222,
        1000,
        50,
        2,
        1,
    )

    decoded = decode_message(Frame(CONTROLLER_TELEMETRY, 0, 2, payload))

    assert decoded["name"] == "controller"
    assert decoded["states"] == [2, 2]
    assert decoded["command_ages_ms"] == {
        "master": 7,
        "slave_feedback": 8,
        "slave_command": 9,
    }
    assert decoded["halls"] == [3, 6]
    assert decoded["bridges_enabled"] == [False, False]
    assert decoded["compare_offsets"] == [111, 222]
    assert decoded["remote_framing_errors"] == 1

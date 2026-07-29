from trashcan_robot.protocol import (
    ARM,
    CLEAR_FAULT,
    ERROR,
    HELLO,
    RESILIENCE_TELEMETRY,
    SET_OPERATING_MODE,
    SET_VELOCITY_YAW,
    FrameDecoder,
    decode_message,
    encode_frame,
    encode_motion,
)
from trashcan_robot.simulation import (
    FAULT_FEEDBACK_CRC,
    SimulatedEsp32Serial,
)


def drain(endpoint: SimulatedEsp32Serial):
    return FrameDecoder().feed(endpoint.read(endpoint.in_waiting))


def arm_drive(endpoint: SimulatedEsp32Serial) -> None:
    endpoint.write(encode_frame(HELLO, 0))
    endpoint.write(encode_frame(SET_OPERATING_MODE, 1, b"\x02"))
    endpoint.write(
        encode_frame(
            SET_VELOCITY_YAW, 2, encode_motion(0.0, 0.0, 7, 500)
        )
    )
    endpoint.write(encode_frame(CLEAR_FAULT, 3))
    endpoint.write(encode_frame(ARM, 4))
    assert endpoint.armed


def test_one_or_two_corrupt_frames_warn_and_valid_feedback_resets_streak() -> None:
    endpoint = SimulatedEsp32Serial()
    arm_drive(endpoint)

    endpoint.inject_feedback_crc(2)
    endpoint.advance(0.01)
    assert endpoint.armed
    assert not endpoint.faults & FAULT_FEEDBACK_CRC

    endpoint.advance(0.01)
    endpoint.write(encode_frame(RESILIENCE_TELEMETRY, 5))
    telemetry = decode_message(drain(endpoint)[-1])
    assert telemetry["feedback_crc"]["streak"] == 0
    assert telemetry["feedback_crc"]["total"] == 2


def test_third_corrupt_frame_faults_and_recovery_cannot_replay_motion() -> None:
    endpoint = SimulatedEsp32Serial()
    arm_drive(endpoint)
    endpoint.write(
        encode_frame(
            SET_VELOCITY_YAW, 5, encode_motion(0.2, 0.0, 7, 500)
        )
    )
    endpoint.advance(0.10)
    assert endpoint.commanded != (0, 0)

    endpoint.inject_feedback_crc(3)
    endpoint.advance(0.02)
    assert endpoint.faults & FAULT_FEEDBACK_CRC
    assert not endpoint.armed
    assert endpoint.commanded == (0, 0)

    endpoint.write(
        encode_frame(
            SET_VELOCITY_YAW, 6, encode_motion(0.2, 0.0, 7, 500)
        )
    )
    assert drain(endpoint)[-1].message_type == ERROR

    endpoint.write(
        encode_frame(
            SET_VELOCITY_YAW, 7, encode_motion(0.0, 0.0, 8, 500)
        )
    )
    endpoint.write(encode_frame(CLEAR_FAULT, 8))
    endpoint.write(encode_frame(RESILIENCE_TELEMETRY, 9))
    retained = decode_message(drain(endpoint)[-1])
    assert retained["first_fault"]["drive"] & FAULT_FEEDBACK_CRC

    endpoint.write(encode_frame(ARM, 10))
    endpoint.advance(0.10)
    assert endpoint.armed
    assert endpoint.commanded == (0, 0)

    endpoint.write(
        encode_frame(
            SET_VELOCITY_YAW, 11, encode_motion(0.2, 0.0, 8, 500)
        )
    )
    endpoint.advance(0.10)
    assert endpoint.commanded != (0, 0)

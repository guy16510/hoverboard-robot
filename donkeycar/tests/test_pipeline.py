from pathlib import Path
from typing import Any

import pytest

from trashcan_robot.pipeline import (
    DRIVE_OUTPUTS,
    HALL_OUTPUTS,
    LEFT_HALL_OUTPUTS,
    RIGHT_HALL_OUTPUTS,
    STATE_UPDATE_INPUTS,
    ULTRASONIC_OUTPUTS,
    DriveMode,
    FrequencyMeter,
    HallPart,
    PilotCondition,
    StateUpdater,
    UltrasonicPart,
    create_tub_writer,
)
from trashcan_robot.protocol import HallReading, UltrasonicReading
from trashcan_robot.state import RobotState
from trashcan_robot.transport import MockMotorTransport


def test_tub_writer_uses_donkeycar_53_base_path_keyword() -> None:
    captured: dict[str, Any] = {}

    class FakeTubWriter:
        def __init__(
            self,
            *,
            base_path: str,
            inputs: list[str],
            types: list[str],
        ) -> None:
            captured.update(
                base_path=base_path,
                inputs=inputs,
                types=types,
            )

    inputs = ["cam/image_array", "user/angle"]
    types = ["image_array", "float"]

    writer = create_tub_writer(FakeTubWriter, Path("/tmp/tub"), inputs, types)

    assert isinstance(writer, FakeTubWriter)
    assert captured == {
        "base_path": "/tmp/tub",
        "inputs": inputs,
        "types": types,
    }


def test_frequency_meter_reports_completed_one_second_window() -> None:
    now = [10.0]
    meter = FrequencyMeter(clock=lambda: now[0])

    for _ in range(20):
        meter.run()
        now[0] += 0.05

    assert meter.run() == pytest.approx(20.0)


def test_state_updater_publishes_runtime_rates_ultrasonic_and_both_halls() -> None:
    state = RobotState()
    updater = StateUpdater(state, "driveway")

    updater.run(
        "Manual",
        False,
        0.0,
        0.0,
        False,
        None,
        None,
        0.42,
        0.75,
        None,
        5,
        True,
        True,
        123,
        18.5,
        2,
        1,
        0.03,
        3,
        True,
        False,
        88,
        0.0,
        0,
        0,
        0.25,
        19.8,
        0.0,
    )

    snapshot = state.snapshot()
    assert snapshot["fps"] == 19.8
    assert snapshot["inference_rate"] == 0.0
    assert snapshot["ultrasonic"] == {
        "front_m": 0.42,
        "left_m": 0.75,
        "right_m": None,
    }
    assert snapshot["right_hall"] == {
        "state": 5,
        "valid": True,
        "moving": True,
        "transitions": 123,
        "transitions_per_second": 18.5,
        "invalid_states": 2,
        "skipped_transitions": 1,
        "last_transition_age_s": 0.03,
    }
    assert snapshot["left_hall"] == {
        "state": 3,
        "valid": True,
        "moving": False,
        "transitions": 88,
        "transitions_per_second": 0.0,
        "invalid_states": 0,
        "skipped_transitions": 0,
        "last_transition_age_s": 0.25,
    }


def test_ultrasonic_part_exposes_transport_reading() -> None:
    transport = MockMotorTransport(
        ultrasonic=UltrasonicReading(front_m=0.25, left_m=0.5, right_m=1.25)
    )
    assert UltrasonicPart(transport).run() == (0.25, 0.5, 1.25)


def sample_hall(state: int, transitions: int) -> HallReading:
    return HallReading(
        state=state,
        valid=True,
        moving=True,
        transitions=transitions,
        invalid_states=1,
        skipped_transitions=2,
        transitions_per_second=24.5,
        last_transition_age_s=0.015,
    )


def test_hall_part_depends_on_reader_not_transport_concrete_type() -> None:
    transport = MockMotorTransport(hall=sample_hall(6, 99))
    assert HallPart(transport.latest_hall).run() == (
        6,
        True,
        True,
        99,
        24.5,
        1,
        2,
        0.015,
    )


def test_pipeline_keeps_drive_sensor_outputs_explicit() -> None:
    assert DRIVE_OUTPUTS == [
        "esp32/connected",
        "drive/linear",
        "drive/angular",
        "serial/latency_ms",
        "drive/fault",
    ]
    assert ULTRASONIC_OUTPUTS == [
        "ultrasonic/front_m",
        "ultrasonic/left_m",
        "ultrasonic/right_m",
    ]
    assert RIGHT_HALL_OUTPUTS == [
        "hall/right_state",
        "hall/right_valid",
        "hall/right_moving",
        "hall/right_transitions",
        "hall/right_tps",
        "hall/right_invalid_states",
        "hall/right_skipped_transitions",
        "hall/right_age_s",
    ]
    assert LEFT_HALL_OUTPUTS == [
        "hall/left_state",
        "hall/left_valid",
        "hall/left_moving",
        "hall/left_transitions",
        "hall/left_tps",
        "hall/left_invalid_states",
        "hall/left_skipped_transitions",
        "hall/left_age_s",
    ]
    assert HALL_OUTPUTS == [*RIGHT_HALL_OUTPUTS, *LEFT_HALL_OUTPUTS]
    assert STATE_UPDATE_INPUTS[-18:] == [
        *RIGHT_HALL_OUTPUTS,
        *LEFT_HALL_OUTPUTS,
        "camera/fps",
        "inference/rate",
    ]


def test_autonomous_is_blocked_when_esp32_disconnects() -> None:
    angle, throttle, mode = DriveMode().run("local_angle", 0.1, 0.2, 0.7, 0.8, False)
    assert (angle, throttle, mode) == (0.0, 0.0, "Stopped")


def test_autonomous_uses_pilot_when_connected() -> None:
    angle, throttle, mode = DriveMode().run("local_angle", 0.1, 0.2, 0.7, 0.8, True)
    assert (angle, throttle, mode) == (0.7, 0.8, "Autonomous")


def test_manual_uses_user_controls() -> None:
    angle, throttle, mode = DriveMode().run("user", -0.2, 0.4, 0.7, 0.8, True)
    assert (angle, throttle, mode) == (-0.2, 0.4, "Manual")


def test_pilot_condition_precedes_model_and_requires_connection() -> None:
    condition = PilotCondition()
    assert condition.run("local", True)
    assert not condition.run("local", False)
    assert not condition.run("user", True)

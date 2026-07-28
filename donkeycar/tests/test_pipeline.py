from pathlib import Path
from types import ModuleType
from typing import Any
import sys

import pytest

from trashcan_robot.pipeline import (
    DRIVE_OUTPUTS,
    STATE_UPDATE_INPUTS,
    DriveMode,
    FrequencyMeter,
    PilotCondition,
    StateUpdater,
    create_tub_writer,
    create_web_controller,
)
from trashcan_robot.state import RobotState


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


def test_web_controller_binds_explicitly_to_all_interfaces(monkeypatch) -> None:
    calls: list[tuple[int, str]] = []
    loop_started = []

    class FakeLoop:
        @staticmethod
        def instance():
            return FakeLoop()

        def start(self) -> None:
            loop_started.append(True)

    tornado = ModuleType("tornado")
    ioloop = ModuleType("tornado.ioloop")
    ioloop.IOLoop = FakeLoop
    monkeypatch.setitem(sys.modules, "tornado", tornado)
    monkeypatch.setitem(sys.modules, "tornado.ioloop", ioloop)

    class FakeController:
        def __init__(self, port: int) -> None:
            self.port = port
            self.loop = None

        def listen(self, port: int, *, address: str) -> None:
            calls.append((port, address))

    controller = create_web_controller(FakeController, "0.0.0.0", 8887)
    controller.update()

    assert calls == [(8887, "0.0.0.0")]
    assert loop_started == [True]
    assert controller.bind_host == "0.0.0.0"
    assert controller.bind_port == 8887


def test_web_controller_rejects_loopback_only_binding() -> None:
    with pytest.raises(ValueError):
        create_web_controller(lambda port: object(), "127.0.0.1", 8887)


def test_frequency_meter_reports_completed_one_second_window() -> None:
    now = [10.0]
    meter = FrequencyMeter(clock=lambda: now[0])

    for _ in range(20):
        meter.run()
        now[0] += 0.05

    assert meter.run() == pytest.approx(20.0)


def test_state_updater_publishes_runtime_rates() -> None:
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
        19.8,
        0.0,
    )

    snapshot = state.snapshot()
    assert snapshot["fps"] == 19.8
    assert snapshot["inference_rate"] == 0.0


def test_pipeline_keeps_drive_outputs_separate_from_state_telemetry() -> None:
    assert DRIVE_OUTPUTS == [
        "esp32/connected",
        "drive/linear",
        "drive/angular",
        "serial/latency_ms",
        "drive/fault",
    ]
    assert STATE_UPDATE_INPUTS[-2:] == ["camera/fps", "inference/rate"]


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

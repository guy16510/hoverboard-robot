import importlib.util
import sys
import types
from pathlib import Path

if "serial" not in sys.modules:
    sys.modules["serial"] = types.SimpleNamespace(Serial=object)

SCRIPT = Path(__file__).parents[1] / "scripts" / "capture_hall_movement.py"
SPEC = importlib.util.spec_from_file_location("capture_hall_movement", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
capture = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(capture)


def controller(halls: list[int], bridges: list[bool] | None = None) -> dict:
    return {
        "halls": halls,
        "bridges_enabled": bridges or [False, False],
        "command_ages_ms": {
            "master": 1,
            "slave_feedback": 2,
            "slave_command": 3,
        },
    }


def test_records_each_wheels_manual_hall_changes_with_bridges_off() -> None:
    recorder = capture.HallObservationRecorder()

    recorder.observe(0.0, controller([1, 2]))
    recorder.observe(0.1, controller([3, 2]))
    recorder.observe(0.2, controller([3, 4]))

    assert recorder.summary()["hall_changes"] == [1, 1]
    assert len(recorder.summary()["observations"]) == 3


def test_rejects_manual_capture_if_either_bridge_is_enabled() -> None:
    recorder = capture.HallObservationRecorder()

    try:
        recorder.observe(0.0, controller([1, 2], [False, True]))
    except RuntimeError as exc:
        assert "bridge enabled" in str(exc)
    else:
        raise AssertionError("manual Hall capture must require disabled bridges")

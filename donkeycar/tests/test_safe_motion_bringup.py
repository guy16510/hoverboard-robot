import importlib.util
import sys
import types
from pathlib import Path

if "serial" not in sys.modules:
    sys.modules["serial"] = types.SimpleNamespace(Serial=object)

SCRIPT = Path(__file__).parents[1] / "scripts" / "safe_motion_bringup.py"
SPEC = importlib.util.spec_from_file_location("safe_motion_bringup", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
bringup = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bringup)


def test_wheel_targets_invert_drive_mixer() -> None:
    assert bringup.wheel_targets_to_motion(25, 25) == (25 / 650, 0.0)
    assert bringup.wheel_targets_to_motion(25, 0) == (25 / 1300, 25 / 700)
    assert bringup.wheel_targets_to_motion(0, 25) == (25 / 1300, -25 / 700)
    assert bringup.wheel_targets_to_motion(25, -25) == (0.0, 25 / 350)


def test_escalates_only_when_both_odometers_are_unchanged() -> None:
    assert bringup.needs_escalation(0, 0)
    assert not bringup.needs_escalation(1, 0)
    assert not bringup.needs_escalation(0, -1)


def test_final_level_without_odometry_is_rejected() -> None:
    try:
        bringup.accept_motion_or_escalate(
            "right-forward",
            250,
            250,
            [0, 0],
        )
    except RuntimeError as exc:
        assert "no Hall odometry" in str(exc)
    else:
        raise AssertionError("zero-odometry final level must fail")


def test_lower_level_without_odometry_escalates() -> None:
    assert not bringup.accept_motion_or_escalate(
        "right-forward",
        100,
        250,
        [0, 0],
    )
    assert bringup.accept_motion_or_escalate(
        "right-forward",
        250,
        250,
        [0, 4],
    )


def test_command_levels_supports_explicit_full_startup_torque() -> None:
    assert bringup.command_levels("250") == (250,)
    assert bringup.command_levels("25,50,100") == (25, 50, 100)


def test_maneuvers_can_select_right_reverse_diagnostic() -> None:
    assert bringup.maneuver_names("right-reverse") == ("right-reverse",)

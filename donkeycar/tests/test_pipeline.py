from trashcan_robot.pipeline import DriveMode, PilotCondition


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

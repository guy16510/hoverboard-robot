from trashcan_robot.pipeline import DriveMode


def test_autonomous_is_blocked_when_esp32_disconnects() -> None:
    angle, throttle, run_pilot, mode = DriveMode().run("local_angle", 0.1, 0.2, 0.7, 0.8, False)
    assert (angle, throttle, run_pilot, mode) == (0.0, 0.0, False, "Stopped")


def test_autonomous_uses_pilot_when_connected() -> None:
    angle, throttle, run_pilot, mode = DriveMode().run("local_angle", 0.1, 0.2, 0.7, 0.8, True)
    assert (angle, throttle, run_pilot, mode) == (0.7, 0.8, True, "Autonomous")


def test_manual_uses_user_controls() -> None:
    angle, throttle, run_pilot, mode = DriveMode().run("user", -0.2, 0.4, 0.7, 0.8, True)
    assert (angle, throttle, run_pilot, mode) == (-0.2, 0.4, False, "Manual")

import pytest

from trashcan_robot.config import WheelKinematicsConfig
from trashcan_robot.kinematics import HallKinematics
from trashcan_robot.protocol import HallReading


def test_observed_t882_hall_run_converts_to_speed_and_travel() -> None:
    kinematics = HallKinematics(
        WheelKinematicsConfig(
            wheel_diameter_m=0.1651,
            hall_transitions_per_revolution=90,
        )
    )
    reading = HallReading(
        state=5,
        valid=True,
        moving=True,
        transitions=1200,
        invalid_states=0,
        skipped_transitions=0,
        transitions_per_second=240.0,
        last_transition_age_s=0.0,
    )

    motion = kinematics.calculate(reading)

    assert motion.rpm == pytest.approx(160.0)
    assert motion.speed_mps == pytest.approx(1.38314, rel=1e-5)
    assert motion.speed_mph == pytest.approx(3.09399, rel=1e-5)
    assert motion.travel_m == pytest.approx(6.91569, rel=1e-5)


def test_kinematics_rejects_invalid_calibration() -> None:
    with pytest.raises(ValueError):
        HallKinematics(
            WheelKinematicsConfig(
                wheel_diameter_m=0.0,
                hall_transitions_per_revolution=90,
            )
        )

    with pytest.raises(ValueError):
        HallKinematics(
            WheelKinematicsConfig(
                wheel_diameter_m=0.1651,
                hall_transitions_per_revolution=0,
            )
        )

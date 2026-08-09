import pytest

from trashcan_robot.config import LimitsConfig
from trashcan_robot.right_wheel_hall_test import (
    RightWheelHallTestPlan,
    counter_delta,
    right_wheel_only_command,
)


def limits() -> LimitsConfig:
    return LimitsConfig(
        max_linear_velocity=0.35,
        max_angular_velocity=0.8,
        throttle_deadband=0.03,
        steering_deadband=0.03,
    )


def test_right_wheel_command_algebra_leaves_left_wheel_at_zero() -> None:
    demand = 0.25
    linear_velocity, angular_velocity = right_wheel_only_command(limits(), demand)

    normalized_linear = linear_velocity / limits().max_linear_velocity
    normalized_yaw = angular_velocity / limits().max_angular_velocity
    left = normalized_linear + normalized_yaw
    right = normalized_linear - normalized_yaw

    assert left == pytest.approx(0.0)
    assert right == pytest.approx(demand)


def test_right_wheel_plan_rejects_aggressive_bench_demand() -> None:
    with pytest.raises(ValueError):
        RightWheelHallTestPlan(demand=0.75).validate()


def test_hall_counter_delta_handles_u32_wrap() -> None:
    assert counter_delta(0xFFFFFFFE, 3) == 5

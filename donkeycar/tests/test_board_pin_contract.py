import re
from pathlib import Path


BOARD_CONFIG = Path(__file__).parents[2] / "firmware" / "esp32" / "board_config.h"


def cpp_int(name: str) -> int:
    text = BOARD_CONFIG.read_text(encoding="utf-8")
    match = re.search(rf"\b{name}\s*=\s*(\d+)", text)
    assert match is not None, f"missing {name} in ESP32 board config"
    return int(match.group(1))


def test_full_esp32_pin_map_is_locked_and_unique() -> None:
    expected = {
        "kLeftThrottlePin": 25,
        "kRightThrottlePin": 26,
        "kLeftReversePin": 27,
        "kRightReversePin": 14,
        "kLeftBrakePin": 33,
        "kRightBrakePin": 32,
        "kRightHallAPin": 19,
        "kRightHallBPin": 21,
        "kRightHallCPin": 22,
        "kLeftHallAPin": 16,
        "kLeftHallBPin": 17,
        "kLeftHallCPin": 36,
        "kFrontTrigPin": 5,
        "kFrontEchoPin": 34,
        "kLeftTrigPin": 15,
        "kLeftEchoPin": 35,
        "kRightTrigPin": 18,
        "kRightEchoPin": 39,
        "kMpu6050SdaPin": 4,
        "kMpu6050SclPin": 23,
        "kArmServoPin": 13,
    }
    actual = {name: cpp_int(name) for name in expected}
    assert actual == expected
    assert len(set(actual.values())) == len(actual)
    assert not set(actual.values()) & set(range(6, 12))


def test_input_only_gpio_are_not_used_as_outputs() -> None:
    output_names = [
        "kLeftThrottlePin",
        "kRightThrottlePin",
        "kLeftReversePin",
        "kRightReversePin",
        "kLeftBrakePin",
        "kRightBrakePin",
        "kFrontTrigPin",
        "kLeftTrigPin",
        "kRightTrigPin",
        "kMpu6050SdaPin",
        "kMpu6050SclPin",
        "kArmServoPin",
    ]
    assert all(cpp_int(name) < 34 for name in output_names)

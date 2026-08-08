from pathlib import Path

from trashcan_robot.config import load_config


def test_robot_config_uses_automatic_esp32_serial_discovery() -> None:
    config_path = Path(__file__).parents[1] / "config" / "robot.yaml"

    config = load_config(config_path)

    assert config.serial.port == "auto"
    assert config.serial.baud == 115200

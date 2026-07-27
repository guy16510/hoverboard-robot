from pathlib import Path

from trashcan_robot.config import load_config


def test_robot_config_uses_stable_esp32_serial_path() -> None:
    config_path = Path(__file__).parents[1] / "config" / "robot.yaml"

    config = load_config(config_path)

    assert config.serial.port == (
        "/dev/serial/by-id/"
        "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
    )

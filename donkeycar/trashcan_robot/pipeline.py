from __future__ import annotations

from typing import Any

from .config import AppConfig
from .dashboard import DashboardServer
from .esp32_drive import ESP32Drive
from .logging_part import JsonRunLogger
from .state import RobotState
from .transport import MockMotorTransport, SerialMotorTransport


def build_vehicle(config: AppConfig, donkey_config: Any, use_mock: bool = False) -> Any:
    import donkeycar as dk
    from donkeycar.parts.camera import PiCamera
    from donkeycar.parts.controller import LocalWebController
    from donkeycar.parts.keras import KerasLinear

    vehicle = dk.Vehicle()
    state = RobotState()
    transport = MockMotorTransport() if use_mock else SerialMotorTransport(config.serial)
    drive = ESP32Drive(transport, config.limits, config.serial.reconnect_seconds)

    camera_cfg = config.raw["camera"]
    camera = PiCamera(
        image_w=camera_cfg["width"],
        image_h=camera_cfg["height"],
        image_d=3,
        framerate=camera_cfg["fps"],
    )
    vehicle.add(camera, outputs=["cam/image_array"], threaded=True)

    controller = LocalWebController(port=donkey_config.WEB_CONTROL_PORT)
    vehicle.add(
        controller,
        inputs=["cam/image_array", "tub/num_records", "user/mode", "recording"],
        outputs=["user/angle", "user/throttle", "user/mode", "recording"],
        threaded=True,
    )

    model_cfg = config.raw["model"]
    pilot = KerasLinear()
    pilot.load(model_cfg["path"])
    vehicle.add(
        pilot,
        inputs=["cam/image_array"],
        outputs=["pilot/angle", "pilot/throttle"],
        run_condition="run_pilot",
    )

    vehicle.add(
        DriveMode(),
        inputs=["user/mode", "user/angle", "user/throttle", "pilot/angle", "pilot/throttle", "esp32/connected"],
        outputs=["angle", "throttle", "run_pilot", "robot/mode"],
    )
    vehicle.add(
        drive,
        inputs=["throttle", "angle"],
        outputs=["esp32/connected", "drive/linear", "drive/angular", "serial/latency_ms", "drive/fault"],
    )

    logger = JsonRunLogger(state, transport, config.raw["logging"]["directory"], model_cfg["name"])
    vehicle.add(StateUpdater(state, model_cfg["name"]), inputs=["robot/mode", "recording", "angle", "throttle", "esp32/connected", "serial/latency_ms", "drive/fault"])
    vehicle.add(logger, inputs=["camera/fps", "inference/rate"], outputs=["log/path"])

    dashboard_cfg = config.raw["dashboard"]
    dashboard = DashboardServer(state, dashboard_cfg["host"], dashboard_cfg["port"])
    dashboard.start()
    vehicle.add(CameraPublisher(dashboard), inputs=["cam/image_array"])
    return vehicle


class DriveMode:
    def run(self, mode: str | None, user_angle: float | None, user_throttle: float | None, pilot_angle: float | None, pilot_throttle: float | None, connected: bool | None) -> tuple[float, float, bool, str]:
        mode = mode or "user"
        autonomous = mode != "user"
        if autonomous and not connected:
            return 0.0, 0.0, False, "Stopped"
        if autonomous:
            return float(pilot_angle or 0.0), float(pilot_throttle or 0.0), True, "Autonomous"
        return float(user_angle or 0.0), float(user_throttle or 0.0), False, "Manual"


class StateUpdater:
    def __init__(self, state: RobotState, model_name: str) -> None:
        self._state = state
        self._model_name = model_name

    def run(self, mode: str | None, recording: bool | None, angle: float | None, throttle: float | None, connected: bool | None, latency: float | None, fault: str | None) -> None:
        self._state.update(
            mode=mode or "Stopped",
            recording=bool(recording),
            steering=float(angle or 0.0),
            throttle=float(throttle or 0.0),
            esp32_connected=bool(connected),
            serial_latency_ms=latency,
            faults=[fault] if fault else [],
            model_name=self._model_name,
        )


class CameraPublisher:
    def __init__(self, dashboard: DashboardServer) -> None:
        self._dashboard = dashboard

    def run(self, image: Any) -> None:
        if image is None:
            return
        try:
            from PIL import Image
            from io import BytesIO
            buffer = BytesIO()
            Image.fromarray(image).save(buffer, format="JPEG", quality=70)
            self._dashboard.update_camera(buffer.getvalue())
        except Exception:
            return

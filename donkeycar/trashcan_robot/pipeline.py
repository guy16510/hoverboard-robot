from __future__ import annotations

import time
from collections.abc import Callable
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
from types import MethodType
from typing import Any

from .apriltag_camera import AprilTagCamera
from .config import AppConfig
from .dashboard import DashboardServer
from .esp32_drive import ESP32Drive
from .logging_part import JsonRunLogger
from .state import RobotState
from .transport import MockMotorTransport, SerialMotorTransport

DRIVE_OUTPUTS = [
    "esp32/connected",
    "drive/linear",
    "drive/angular",
    "serial/latency_ms",
    "drive/fault",
]
STATE_UPDATE_INPUTS = [
    "robot/mode",
    "recording",
    "angle",
    "throttle",
    "esp32/connected",
    "serial/latency_ms",
    "drive/fault",
    "camera/fps",
    "inference/rate",
]


def build_vehicle(config: AppConfig, use_mock: bool = False) -> Any:
    import donkeycar as dk
    from donkeycar.parts.camera import PiCamera
    from donkeycar.parts.controller import LocalWebController
    from donkeycar.parts.keras import KerasLinear
    from donkeycar.parts.tub_v2 import TubWriter
    from donkeycar.parts.web_controller.web import WebSocketDriveAPI

    install_safe_websocket_close_handler(WebSocketDriveAPI)

    vehicle = dk.Vehicle()
    state = RobotState()
    transport = (
        MockMotorTransport()
        if use_mock
        else SerialMotorTransport(config.serial, renew_commands=True)
    )
    drive = ESP32Drive(transport, config.limits, config.serial.reconnect_seconds)

    camera_cfg = config.raw["camera"]
    camera = PiCamera(
        image_w=camera_cfg["width"],
        image_h=camera_cfg["height"],
        image_d=3,
    )
    vehicle.add(camera, outputs=["cam/image_array"], threaded=True)
    vehicle.add(FrequencyMeter(), outputs=["camera/fps"])

    backup_cfg = config.raw.get("backup_camera", {})
    backup_enabled = bool(backup_cfg.get("enabled", False))
    if backup_enabled:
        backup_camera = AprilTagCamera(backup_cfg)
        vehicle.add(
            backup_camera,
            outputs=["backup/image_array", "backup/status"],
            threaded=True,
        )

    controller_cfg = config.raw["controller"]
    controller = create_web_controller(
        LocalWebController,
        controller_cfg["web_host"],
        controller_cfg["web_port"],
    )
    vehicle.add(
        controller,
        inputs=["cam/image_array", "tub/num_records", "user/mode", "recording"],
        outputs=["user/angle", "user/throttle", "user/mode", "recording"],
        threaded=True,
    )

    vehicle.add(
        PilotCondition(),
        inputs=["user/mode", "esp32/connected"],
        outputs=["run_pilot"],
    )

    model_cfg = config.raw["model"]
    model_path = Path(model_cfg["path"])
    if model_path.is_file():
        pilot = KerasLinear()
        pilot.load(str(model_path))
        vehicle.add(
            pilot,
            inputs=["cam/image_array"],
            outputs=["pilot/angle", "pilot/throttle"],
            run_condition="run_pilot",
        )
    else:
        vehicle.add(ZeroPilot(), outputs=["pilot/angle", "pilot/throttle"])

    vehicle.add(
        DriveMode(),
        inputs=[
            "user/mode",
            "user/angle",
            "user/throttle",
            "pilot/angle",
            "pilot/throttle",
            "esp32/connected",
        ],
        outputs=["angle", "throttle", "robot/mode"],
    )
    vehicle.add(
        drive,
        inputs=["throttle", "angle"],
        outputs=DRIVE_OUTPUTS,
    )

    tub_root = Path(config.raw["data"]["tubs_directory"])
    tub_root.mkdir(parents=True, exist_ok=True)
    tub_path = tub_root / datetime.now(timezone.utc).strftime("tub_%Y%m%dT%H%M%SZ")
    tub_inputs = ["cam/image_array", "user/angle", "user/throttle", "user/mode"]
    tub_types = ["image_array", "float", "float", "str"]
    tub = create_tub_writer(TubWriter, tub_path, tub_inputs, tub_types)
    vehicle.add(
        tub,
        inputs=tub_inputs,
        outputs=["tub/num_records"],
        run_condition="recording",
    )

    logging_cfg = config.raw["logging"]
    logger = JsonRunLogger(
        state,
        transport,
        logging_cfg["directory"],
        model_cfg["name"],
        telemetry_hz=logging_cfg["telemetry_hz"],
    )
    vehicle.add(
        StateUpdater(state, model_cfg["name"]),
        inputs=STATE_UPDATE_INPUTS,
    )
    if backup_enabled:
        vehicle.add(
            AprilTagStateUpdater(state),
            inputs=["backup/status"],
        )
    else:
        state.update(
            backup_camera={
                "connected": False,
                "device": backup_cfg.get("device"),
                "fps": 0.0,
                "family": backup_cfg.get("family"),
                "error": "backup camera disabled",
                "last_frame_age_ms": None,
            },
            apriltags=[],
        )
    vehicle.add(logger, inputs=["camera/fps", "inference/rate"], outputs=["log/path"])

    dashboard_cfg = config.raw["dashboard"]
    dashboard = DashboardServer(
        state,
        dashboard_cfg["host"],
        dashboard_cfg["port"],
        manual_port=controller_cfg["web_port"],
    )
    dashboard.start()
    vehicle.add(CameraPublisher(dashboard), inputs=["cam/image_array"])
    if backup_enabled:
        vehicle.add(
            CameraPublisher(dashboard, backup=True),
            inputs=["backup/image_array"],
        )
    return vehicle


def create_web_controller(controller_type: Any, host: str, port: int) -> Any:
    """Create Donkeycar's controller with an explicit Tornado bind address."""
    if host != "0.0.0.0":
        raise ValueError("manual driving controller must bind to 0.0.0.0")
    controller = controller_type(port=port)

    def update(bound_controller: Any) -> None:
        import asyncio

        from tornado.ioloop import IOLoop

        asyncio.set_event_loop(asyncio.new_event_loop())
        bound_controller.listen(port, address=host)
        bound_controller.loop = IOLoop.instance()
        bound_controller.loop.start()

    controller.update = MethodType(update, controller)
    controller.bind_host = host
    controller.bind_port = port
    return controller


def install_safe_websocket_close_handler(handler_type: Any) -> None:
    """Force neutral controls before Donkeycar removes a websocket client."""
    if getattr(handler_type, "_trashcan_safe_close_installed", False):
        return
    original_close = handler_type.on_close

    def safe_close(handler: Any) -> Any:
        handler.application.angle = 0.0
        handler.application.throttle = 0.0
        handler.application.recording = False
        handler.application.mode = "user"
        return original_close(handler)

    handler_type.on_close = safe_close
    handler_type._trashcan_safe_close_installed = True


def create_tub_writer(
    writer_type: Any,
    tub_path: Path,
    inputs: list[str],
    types: list[str],
) -> Any:
    return writer_type(
        base_path=str(tub_path),
        inputs=inputs,
        types=types,
    )


class PilotCondition:
    def run(self, mode: str | None, connected: bool | None) -> bool:
        return bool(connected) and (mode or "user") != "user"


class FrequencyMeter:
    def __init__(
        self,
        window_seconds: float = 1.0,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._window_seconds = window_seconds
        self._clock = clock
        self._window_started_at = clock()
        self._count = 0
        self._frequency = 0.0

    def run(self) -> float:
        now = self._clock()
        elapsed = now - self._window_started_at
        if elapsed < self._window_seconds:
            self._count += 1
            return self._frequency
        self._frequency = self._count / elapsed
        self._window_started_at = now
        self._count = 1
        return self._frequency


class ZeroPilot:
    def run(self) -> tuple[float, float]:
        return 0.0, 0.0


class DriveMode:
    def run(
        self,
        mode: str | None,
        user_angle: float | None,
        user_throttle: float | None,
        pilot_angle: float | None,
        pilot_throttle: float | None,
        connected: bool | None,
    ) -> tuple[float, float, str]:
        mode = mode or "user"
        autonomous = mode != "user"
        if autonomous and not connected:
            return 0.0, 0.0, "Stopped"
        if autonomous:
            return (
                float(pilot_angle or 0.0),
                float(pilot_throttle or 0.0),
                "Autonomous",
            )
        return (
            float(user_angle or 0.0),
            float(user_throttle or 0.0),
            "Manual",
        )


class StateUpdater:
    def __init__(self, state: RobotState, model_name: str) -> None:
        self._state = state
        self._model_name = model_name

    def run(
        self,
        mode: str | None,
        recording: bool | None,
        angle: float | None,
        throttle: float | None,
        connected: bool | None,
        latency: float | None,
        fault: str | None,
        fps: float | None,
        inference_rate: float | None,
    ) -> None:
        self._state.update(
            mode=mode or "Stopped",
            recording=bool(recording),
            steering=float(angle or 0.0),
            throttle=float(throttle or 0.0),
            esp32_connected=bool(connected),
            serial_latency_ms=latency,
            faults=[fault] if fault else [],
            model_name=self._model_name,
            fps=float(fps or 0.0),
            inference_rate=float(inference_rate or 0.0),
        )


class AprilTagStateUpdater:
    def __init__(self, state: RobotState) -> None:
        self._state = state

    def run(self, status: dict[str, Any] | None) -> None:
        if not status:
            return
        snapshot = deepcopy(status)
        detections = list(snapshot.pop("detections", []))
        self._state.update(
            backup_camera=snapshot,
            apriltags=detections,
        )


class CameraPublisher:
    def __init__(self, dashboard: DashboardServer, backup: bool = False) -> None:
        self._dashboard = dashboard
        self._backup = backup

    def run(self, image: Any) -> None:
        if image is None:
            return
        try:
            from io import BytesIO

            from PIL import Image

            buffer = BytesIO()
            Image.fromarray(image).save(buffer, format="JPEG", quality=70)
            if self._backup:
                self._dashboard.update_backup_camera(buffer.getvalue())
            else:
                self._dashboard.update_camera(buffer.getvalue())
        except Exception:
            return

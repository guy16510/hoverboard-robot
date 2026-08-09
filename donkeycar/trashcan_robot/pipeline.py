from __future__ import annotations

import time
from collections.abc import Callable
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .config import AppConfig
from .dashboard import DashboardServer
from .esp32_drive import ESP32Drive
from .logging_part import JsonRunLogger
from .protocol import HallReading
from .state import RobotState
from .transport import MockMotorTransport, MotorTransport, SerialMotorTransport

DRIVE_OUTPUTS = [
    "esp32/connected",
    "drive/linear",
    "drive/angular",
    "serial/latency_ms",
    "drive/fault",
]
ULTRASONIC_OUTPUTS = [
    "ultrasonic/front_m",
    "ultrasonic/left_m",
    "ultrasonic/right_m",
]
RIGHT_HALL_OUTPUTS = [
    "hall/right_state",
    "hall/right_valid",
    "hall/right_moving",
    "hall/right_transitions",
    "hall/right_tps",
    "hall/right_invalid_states",
    "hall/right_skipped_transitions",
    "hall/right_age_s",
]
LEFT_HALL_OUTPUTS = [
    "hall/left_state",
    "hall/left_valid",
    "hall/left_moving",
    "hall/left_transitions",
    "hall/left_tps",
    "hall/left_invalid_states",
    "hall/left_skipped_transitions",
    "hall/left_age_s",
]
HALL_OUTPUTS = [*RIGHT_HALL_OUTPUTS, *LEFT_HALL_OUTPUTS]
STATE_UPDATE_INPUTS = [
    "robot/mode",
    "recording",
    "angle",
    "throttle",
    "esp32/connected",
    "serial/latency_ms",
    "drive/fault",
    *ULTRASONIC_OUTPUTS,
    *HALL_OUTPUTS,
    "camera/fps",
    "inference/rate",
]


def build_vehicle(config: AppConfig, use_mock: bool = False) -> Any:
    import donkeycar as dk
    from donkeycar.parts.camera import PiCamera
    from donkeycar.parts.controller import LocalWebController
    from donkeycar.parts.keras import KerasLinear
    from donkeycar.parts.tub_v2 import TubWriter

    vehicle = dk.Vehicle()
    state = RobotState()
    transport = MockMotorTransport() if use_mock else SerialMotorTransport(config.serial)
    drive = ESP32Drive(transport, config.limits, config.serial.reconnect_seconds)

    camera_cfg = config.raw["camera"]
    camera = PiCamera(
        image_w=camera_cfg["width"],
        image_h=camera_cfg["height"],
        image_d=3,
    )
    vehicle.add(camera, outputs=["cam/image_array"], threaded=True)
    vehicle.add(FrequencyMeter(), outputs=["camera/fps"])

    controller = LocalWebController(port=config.raw["controller"]["web_port"])
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
    vehicle.add(UltrasonicPart(transport), outputs=ULTRASONIC_OUTPUTS)
    vehicle.add(HallPart(transport.latest_hall), outputs=RIGHT_HALL_OUTPUTS)
    vehicle.add(HallPart(transport.latest_left_hall), outputs=LEFT_HALL_OUTPUTS)

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
    vehicle.add(logger, inputs=["camera/fps", "inference/rate"], outputs=["log/path"])

    dashboard_cfg = config.raw["dashboard"]
    dashboard = DashboardServer(state, dashboard_cfg["host"], dashboard_cfg["port"])
    dashboard.start()
    vehicle.add(CameraPublisher(dashboard), inputs=["cam/image_array"])
    return vehicle


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


class UltrasonicPart:
    def __init__(self, transport: MotorTransport) -> None:
        self._transport = transport

    def run(self) -> tuple[float | None, float | None, float | None]:
        reading = self._transport.latest_ultrasonic()
        return reading.front_m, reading.left_m, reading.right_m


class HallPart:
    def __init__(self, reader: Callable[[], HallReading]) -> None:
        self._reader = reader

    def run(
        self,
    ) -> tuple[int, bool, bool, int, float, int, int, float | None]:
        reading = self._reader()
        return (
            reading.state,
            reading.valid,
            reading.moving,
            reading.transitions,
            reading.transitions_per_second,
            reading.invalid_states,
            reading.skipped_transitions,
            reading.last_transition_age_s,
        )


def hall_state(
    state: int | None,
    valid: bool | None,
    moving: bool | None,
    transitions: int | None,
    tps: float | None,
    invalid_states: int | None,
    skipped_transitions: int | None,
    age_s: float | None,
) -> dict[str, object]:
    return {
        "state": int(state or 0),
        "valid": bool(valid),
        "moving": bool(moving),
        "transitions": int(transitions or 0),
        "transitions_per_second": float(tps or 0.0),
        "invalid_states": int(invalid_states or 0),
        "skipped_transitions": int(skipped_transitions or 0),
        "last_transition_age_s": age_s,
    }


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
        ultrasonic_front_m: float | None,
        ultrasonic_left_m: float | None,
        ultrasonic_right_m: float | None,
        hall_right_state: int | None,
        hall_right_valid: bool | None,
        hall_right_moving: bool | None,
        hall_right_transitions: int | None,
        hall_right_tps: float | None,
        hall_right_invalid_states: int | None,
        hall_right_skipped_transitions: int | None,
        hall_right_age_s: float | None,
        hall_left_state: int | None,
        hall_left_valid: bool | None,
        hall_left_moving: bool | None,
        hall_left_transitions: int | None,
        hall_left_tps: float | None,
        hall_left_invalid_states: int | None,
        hall_left_skipped_transitions: int | None,
        hall_left_age_s: float | None,
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
            ultrasonic={
                "front_m": ultrasonic_front_m,
                "left_m": ultrasonic_left_m,
                "right_m": ultrasonic_right_m,
            },
            right_hall=hall_state(
                hall_right_state,
                hall_right_valid,
                hall_right_moving,
                hall_right_transitions,
                hall_right_tps,
                hall_right_invalid_states,
                hall_right_skipped_transitions,
                hall_right_age_s,
            ),
            left_hall=hall_state(
                hall_left_state,
                hall_left_valid,
                hall_left_moving,
                hall_left_transitions,
                hall_left_tps,
                hall_left_invalid_states,
                hall_left_skipped_transitions,
                hall_left_age_s,
            ),
            faults=[fault] if fault else [],
            model_name=self._model_name,
            fps=float(fps or 0.0),
            inference_rate=float(inference_rate or 0.0),
        )


class CameraPublisher:
    def __init__(self, dashboard: DashboardServer) -> None:
        self._dashboard = dashboard

    def run(self, image: Any) -> None:
        if image is None:
            return
        try:
            from io import BytesIO

            from PIL import Image

            buffer = BytesIO()
            Image.fromarray(image).save(buffer, format="JPEG", quality=70)
            self._dashboard.update_camera(buffer.getvalue())
        except Exception:
            return

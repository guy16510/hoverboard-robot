from trashcan_robot.config import LimitsConfig, SerialConfig
from trashcan_robot.dashboard import DashboardServer
from trashcan_robot.esp32_drive import ESP32Drive
from trashcan_robot.logging_part import JsonRunLogger
from trashcan_robot.pipeline import DriveMode
from trashcan_robot.simulation import (
    FakeClock,
    SimulatedEsp32Serial,
    SimulatedGd32Boundary,
    SimulatedMpu6050,
    SimulatedSerialFactory,
)
from trashcan_robot.state import RobotState
from trashcan_robot.transport import SerialMotorTransport


def test_web_pipeline_serial_simulation_and_dashboard_telemetry(tmp_path) -> None:
    clock = FakeClock()
    gd32 = SimulatedGd32Boundary(clock)
    mpu = SimulatedMpu6050(clock)
    endpoint = SimulatedEsp32Serial(clock, gd32, mpu)
    transport = SerialMotorTransport(
        SerialConfig(
            port="/dev/serial/by-id/simulated-esp32",
            baud=115200,
            timeout_seconds=0.05,
            reconnect_seconds=0.1,
            lease_ms=500,
            command_hz=20,
        ),
        serial_factory=SimulatedSerialFactory(endpoint),
        clock=clock.monotonic,
        sleeper=clock.sleep,
        lease_id_factory=lambda: 0x10203040,
    )
    drive = ESP32Drive(
        transport,
        LimitsConfig(
            max_linear_velocity=0.35,
            max_angular_velocity=0.8,
            throttle_deadband=0.03,
            steering_deadband=0.03,
        ),
        reconnect_seconds=0,
        clock=clock.monotonic,
    )
    state = RobotState()
    logger = JsonRunLogger(
        state,
        transport,
        str(tmp_path),
        model_name="simulation",
        telemetry_hz=5,
        clock=clock.monotonic,
    )

    drive.run(0.0, 0.0)
    angle, throttle, label = DriveMode().run(
        "user", 0.25, 0.5, 0.0, 0.0, True
    )
    connected, linear, angular, latency, fault = drive.run(throttle, angle)
    state.update(
        mode=label,
        steering=angle,
        throttle=throttle,
        esp32_connected=connected,
        serial_latency_ms=latency,
        faults=[fault] if fault else [],
    )
    endpoint.advance(0.30)
    logger.run()

    dashboard = DashboardServer(state, "0.0.0.0", 8888)
    response = dashboard._app.test_client().get("/api/state")
    payload = response.get_json()

    assert response.status_code == 200
    assert payload["esp32_connected"] is True
    assert payload["mode"] == "Manual"
    assert payload["telemetry"]["drive"]["requested_linear"] == linear
    assert payload["telemetry"]["drive"]["requested_yaw"] == angular
    assert payload["telemetry"]["drive"]["armed"] is True
    assert max(abs(value) for value in payload["telemetry"]["drive"]["commanded"]) <= 250
    assert payload["telemetry"]["motor"]["applied"][0] > payload["telemetry"]["motor"]["applied"][1]
    assert "odometry" in payload["telemetry"]
    assert "faults" in payload["telemetry"]

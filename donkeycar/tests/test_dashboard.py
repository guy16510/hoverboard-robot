from trashcan_robot.dashboard import DashboardServer
from trashcan_robot.state import RobotState


def test_dashboard_exposes_manual_controller_and_both_cameras() -> None:
    state = RobotState()
    dashboard = DashboardServer(state, "0.0.0.0", 8888, manual_port=9999)
    client = dashboard._app.test_client()

    page = client.get("/")
    assert page.status_code == 200
    assert b":9999/" in page.data
    assert b"/camera.jpg" in page.data
    assert b"/backup-camera.jpg" in page.data
    assert b"AprilTags" in page.data

    assert client.get("/camera.jpg").status_code == 204
    assert client.get("/backup-camera.jpg").status_code == 204

    dashboard.update_camera(b"front")
    dashboard.update_backup_camera(b"backup")
    assert client.get("/camera.jpg").data == b"front"
    assert client.get("/backup-camera.jpg").data == b"backup"


def test_dashboard_health_reports_drive_and_backup_camera_state() -> None:
    state = RobotState()
    state.update(
        esp32_connected=True,
        backup_camera={
            "connected": True,
            "device": "/dev/video0",
            "fps": 15.0,
            "family": "36h11",
            "error": None,
            "last_frame_age_ms": 0,
        },
    )
    dashboard = DashboardServer(state, "0.0.0.0", 8888)
    payload = dashboard._app.test_client().get("/healthz").get_json()

    assert payload == {
        "dashboard": "ok",
        "esp32_connected": True,
        "backup_camera_connected": True,
    }

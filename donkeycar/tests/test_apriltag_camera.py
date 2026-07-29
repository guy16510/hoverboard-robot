import pytest

from trashcan_robot.apriltag_camera import marker_geometry
from trashcan_robot.pipeline import AprilTagStateUpdater
from trashcan_robot.state import RobotState


def square(center_x: float, center_y: float, side: float):
    half = side / 2.0
    return [
        [center_x - half, center_y - half],
        [center_x + half, center_y - half],
        [center_x + half, center_y + half],
        [center_x - half, center_y + half],
    ]


def test_marker_geometry_reports_id_bearing_position_and_range() -> None:
    detection = marker_geometry(
        17,
        square(480, 240, 100),
        frame_width=640,
        frame_height=480,
        horizontal_fov_degrees=70.0,
        tag_size_m=0.20,
    )

    assert detection["id"] == 17
    assert detection["horizontal_position"] == "right"
    assert detection["bearing_degrees"] > 0
    assert detection["distance_m"] == pytest.approx(0.914, abs=0.001)
    assert detection["area_ratio"] == pytest.approx(10000 / (640 * 480))


def test_marker_geometry_works_without_metric_tag_calibration() -> None:
    detection = marker_geometry(
        3,
        square(320, 240, 80),
        frame_width=640,
        frame_height=480,
        horizontal_fov_degrees=70.0,
        tag_size_m=0.0,
    )

    assert detection["horizontal_position"] == "center"
    assert detection["bearing_degrees"] == pytest.approx(0.0)
    assert detection["distance_m"] is None


def test_apriltag_state_updater_publishes_camera_and_detections() -> None:
    state = RobotState()
    updater = AprilTagStateUpdater(state)
    updater.run(
        {
            "connected": True,
            "device": "/dev/video0",
            "fps": 14.8,
            "family": "36h11",
            "error": None,
            "last_frame_age_ms": 12,
            "detections": [{"id": 9, "horizontal_position": "left"}],
        }
    )

    snapshot = state.snapshot()
    assert snapshot["backup_camera"]["connected"] is True
    assert snapshot["backup_camera"]["fps"] == 14.8
    assert "detections" not in snapshot["backup_camera"]
    assert snapshot["apriltags"] == [
        {"id": 9, "horizontal_position": "left"}
    ]

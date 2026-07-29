from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable, Iterable, Mapping
from copy import deepcopy
from typing import Any

from .video_devices import video_device_candidates


def marker_geometry(
    marker_id: int,
    corners: Iterable[Iterable[float]],
    *,
    frame_width: int,
    frame_height: int,
    horizontal_fov_degrees: float,
    tag_size_m: float,
) -> dict[str, Any]:
    """Return JSON-safe relative geometry for one detected AprilTag."""
    points = [(float(point[0]), float(point[1])) for point in corners]
    if len(points) != 4:
        raise ValueError("AprilTag corners must contain exactly four points")
    if frame_width <= 0 or frame_height <= 0:
        raise ValueError("frame dimensions must be positive")
    if not 1.0 <= horizontal_fov_degrees < 179.0:
        raise ValueError("horizontal_fov_degrees must be between 1 and 179")

    center_x = sum(point[0] for point in points) / 4.0
    center_y = sum(point[1] for point in points) / 4.0
    side_lengths = [
        math.hypot(
            points[(index + 1) % 4][0] - points[index][0],
            points[(index + 1) % 4][1] - points[index][1],
        )
        for index in range(4)
    ]
    mean_side_pixels = sum(side_lengths) / 4.0
    focal_pixels = frame_width / (
        2.0 * math.tan(math.radians(horizontal_fov_degrees) / 2.0)
    )
    bearing_degrees = math.degrees(
        math.atan2(center_x - (frame_width / 2.0), focal_pixels)
    )
    distance_m = (
        float(tag_size_m) * focal_pixels / mean_side_pixels
        if tag_size_m > 0.0 and mean_side_pixels > 0.0
        else None
    )
    area_pixels = abs(
        sum(
            points[index][0] * points[(index + 1) % 4][1]
            - points[(index + 1) % 4][0] * points[index][1]
            for index in range(4)
        )
        / 2.0
    )
    area_ratio = area_pixels / float(frame_width * frame_height)
    rotation_degrees = math.degrees(
        math.atan2(
            points[1][1] - points[0][1],
            points[1][0] - points[0][0],
        )
    )
    lateral_error = (center_x - (frame_width / 2.0)) / (frame_width / 2.0)
    vertical_error = (center_y - (frame_height / 2.0)) / (frame_height / 2.0)
    horizontal_position = (
        "left"
        if bearing_degrees < -7.0
        else "right"
        if bearing_degrees > 7.0
        else "center"
    )

    return {
        "id": int(marker_id),
        "center": [round(center_x, 1), round(center_y, 1)],
        "corners": [[round(x, 1), round(y, 1)] for x, y in points],
        "bearing_degrees": round(bearing_degrees, 2),
        "distance_m": round(distance_m, 3) if distance_m is not None else None,
        "horizontal_position": horizontal_position,
        "rotation_degrees": round(rotation_degrees, 2),
        "lateral_error": round(lateral_error, 4),
        "vertical_error": round(vertical_error, 4),
        "area_ratio": round(area_ratio, 6),
    }


class AprilTagCamera:
    """Threaded USB camera with AprilTag detection and reconnect handling."""

    _DICTIONARIES = {
        "16h5": "DICT_APRILTAG_16h5",
        "25h9": "DICT_APRILTAG_25h9",
        "36h10": "DICT_APRILTAG_36h10",
        "36h11": "DICT_APRILTAG_36h11",
    }

    def __init__(
        self,
        config: Mapping[str, Any],
        *,
        cv2_module: Any | None = None,
        clock: Callable[[], float] = time.monotonic,
        sleeper: Callable[[float], None] = time.sleep,
        device_resolver: Callable[[str], list[str]] = video_device_candidates,
    ) -> None:
        self._config = dict(config)
        self._requested_device = str(self._config.get("device", "auto"))
        self._active_device: str | None = None
        self._width = int(self._config.get("width", 640))
        self._height = int(self._config.get("height", 480))
        self._fps_target = int(self._config.get("fps", 15))
        self._family = str(self._config.get("family", "36h11")).lower()
        self._tag_size_m = float(self._config.get("tag_size_m", 0.0))
        self._horizontal_fov_degrees = float(
            self._config.get("horizontal_fov_degrees", 70.0)
        )
        self._reconnect_seconds = float(
            self._config.get("reconnect_seconds", 2.0)
        )
        self._tag_roles = {
            int(marker_id): str(role)
            for marker_id, role in dict(self._config.get("tag_roles", {})).items()
        }
        if self._family not in self._DICTIONARIES:
            raise ValueError(
                f"unsupported AprilTag family {self._family!r}; "
                f"choose one of {sorted(self._DICTIONARIES)}"
            )
        if self._width <= 0 or self._height <= 0 or self._fps_target <= 0:
            raise ValueError("backup camera width, height, and fps must be positive")
        if self._reconnect_seconds <= 0:
            raise ValueError("backup camera reconnect_seconds must be positive")

        self._cv2 = cv2_module
        self._clock = clock
        self._sleep = sleeper
        self._device_resolver = device_resolver
        self._lock = threading.Lock()
        self._stopped = threading.Event()
        self._capture: Any | None = None
        self._detector: Callable[[Any], tuple[Any, Any, Any]] | None = None
        self._latest_image: Any | None = None
        self._status: dict[str, Any] = {
            "connected": False,
            "requested_device": self._requested_device,
            "device": None,
            "fps": 0.0,
            "family": self._family,
            "detections": [],
            "error": "camera has not started",
            "last_frame_age_ms": None,
        }
        self._last_frame_at: float | None = None
        self._window_started_at = self._clock()
        self._window_frames = 0

    def _load_cv2(self) -> Any:
        if self._cv2 is None:
            import cv2

            self._cv2 = cv2
        if not hasattr(self._cv2, "aruco"):
            raise RuntimeError(
                "OpenCV was built without the aruco module required for AprilTags"
            )
        return self._cv2

    def _create_detector(self) -> Callable[[Any], tuple[Any, Any, Any]]:
        cv2 = self._load_cv2()
        aruco = cv2.aruco
        dictionary_id = getattr(aruco, self._DICTIONARIES[self._family])
        dictionary = aruco.getPredefinedDictionary(dictionary_id)
        if hasattr(aruco, "ArucoDetector"):
            parameters = aruco.DetectorParameters()
            detector = aruco.ArucoDetector(dictionary, parameters)
            return detector.detectMarkers

        parameters = aruco.DetectorParameters_create()

        def detect(gray: Any) -> tuple[Any, Any, Any]:
            return aruco.detectMarkers(gray, dictionary, parameters=parameters)

        return detect

    def _new_capture(self, device: str) -> Any:
        cv2 = self._load_cv2()
        source: str | int = int(device) if device.isdecimal() else device
        backend = getattr(cv2, "CAP_V4L2", None)
        capture = (
            cv2.VideoCapture(source, backend)
            if backend is not None
            else cv2.VideoCapture(source)
        )
        if not capture or not capture.isOpened():
            if capture:
                capture.release()
            raise RuntimeError("could not open device")
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        capture.set(cv2.CAP_PROP_FPS, self._fps_target)
        if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if hasattr(cv2, "CAP_PROP_FOURCC") and hasattr(cv2, "VideoWriter_fourcc"):
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        return capture

    def _open_capture(self) -> Any:
        candidates = self._device_resolver(self._requested_device)
        if not candidates:
            raise RuntimeError(
                f"no V4L2 capture devices found for {self._requested_device!r}"
            )

        errors: list[str] = []
        for device in candidates:
            capture = None
            try:
                capture = self._new_capture(device)
                ok, frame = capture.read()
                if not ok or frame is None:
                    raise RuntimeError("opened but did not return a frame")
                self._capture = capture
                self._active_device = device
                self._detector = self._create_detector()
                return frame
            except Exception as exc:
                errors.append(f"{device}: {exc}")
                if capture is not None:
                    try:
                        capture.release()
                    except Exception:
                        pass
        raise RuntimeError("unable to open backup camera; " + "; ".join(errors))

    def _close_capture(self) -> None:
        capture = self._capture
        self._capture = None
        if capture is not None:
            try:
                capture.release()
            except Exception:
                pass

    def _set_disconnected(self, error: str) -> None:
        with self._lock:
            self._status.update(
                connected=False,
                requested_device=self._requested_device,
                device=self._active_device,
                detections=[],
                error=error,
            )

    def _process_frame(self, frame: Any) -> tuple[Any, list[dict[str, Any]]]:
        cv2 = self._load_cv2()
        if self._detector is None:
            self._detector = self._create_detector()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _rejected = self._detector(gray)
        detections: list[dict[str, Any]] = []
        if ids is not None:
            for marker_corners, marker_id in zip(corners, ids.flatten()):
                points = marker_corners.reshape(4, 2)
                detection = marker_geometry(
                    int(marker_id),
                    points,
                    frame_width=int(frame.shape[1]),
                    frame_height=int(frame.shape[0]),
                    horizontal_fov_degrees=self._horizontal_fov_degrees,
                    tag_size_m=self._tag_size_m,
                )
                detection["role"] = self._tag_roles.get(int(marker_id))
                detections.append(detection)
                integer_points = points.astype("int32").reshape((-1, 1, 2))
                cv2.polylines(frame, [integer_points], True, (0, 255, 0), 2)
                center_x, center_y = detection["center"]
                cv2.circle(frame, (int(center_x), int(center_y)), 4, (0, 0, 255), -1)
                label = f"tag {detection['id']} {detection['bearing_degrees']:+.1f}deg"
                if detection["role"]:
                    label += f" {detection['role']}"
                if detection["distance_m"] is not None:
                    label += f" {detection['distance_m']:.2f}m"
                cv2.putText(
                    frame,
                    label,
                    (int(points[0][0]), max(18, int(points[0][1]) - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )
        detections.sort(key=lambda item: (-item["area_ratio"], item["id"]))
        return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), detections

    def _record_frame(self, image: Any, detections: list[dict[str, Any]]) -> None:
        now = self._clock()
        self._window_frames += 1
        elapsed = now - self._window_started_at
        fps = self._status["fps"]
        if elapsed >= 1.0:
            fps = self._window_frames / elapsed
            self._window_started_at = now
            self._window_frames = 0
        self._last_frame_at = now
        with self._lock:
            self._latest_image = image
            self._status = {
                "connected": True,
                "requested_device": self._requested_device,
                "device": self._active_device,
                "fps": round(float(fps), 2),
                "family": self._family,
                "detections": detections,
                "error": None,
                "last_frame_age_ms": 0,
            }

    def update(self) -> None:
        while not self._stopped.is_set():
            frame = None
            if self._capture is None:
                try:
                    frame = self._open_capture()
                except Exception as exc:
                    self._set_disconnected(str(exc))
                    self._sleep(self._reconnect_seconds)
                    continue

            try:
                if frame is None:
                    ok, frame = self._capture.read()
                    if not ok or frame is None:
                        raise RuntimeError(
                            f"backup camera {self._active_device} stopped returning frames"
                        )
                image, detections = self._process_frame(frame)
                self._record_frame(image, detections)
            except Exception as exc:
                self._close_capture()
                self._set_disconnected(str(exc))
                self._sleep(self._reconnect_seconds)

    def run_threaded(self) -> tuple[Any | None, dict[str, Any]]:
        with self._lock:
            status = deepcopy(self._status)
            image = self._latest_image
        if self._last_frame_at is not None:
            status["last_frame_age_ms"] = round(
                max(0.0, self._clock() - self._last_frame_at) * 1000.0
            )
        return image, status

    def shutdown(self) -> None:
        self._stopped.set()
        self._close_capture()

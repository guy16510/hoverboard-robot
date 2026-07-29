from trashcan_robot.video_devices import video_device_candidates


def test_explicit_video_device_is_not_rewritten() -> None:
    assert video_device_candidates("/dev/video7") == ["/dev/video7"]


def test_auto_prefers_stable_v4l_symlink_and_deduplicates_target() -> None:
    matches = {
        "/dev/v4l/by-id/*-video-index0": ["/dev/v4l/by-id/usb-camera-video-index0"],
        "/dev/v4l/by-path/*-video-index0": [],
        "/dev/video*": ["/dev/video0", "/dev/video2"],
    }
    targets = {
        "/dev/v4l/by-id/usb-camera-video-index0": "/dev/video2",
        "/dev/video0": "/dev/video0",
        "/dev/video2": "/dev/video2",
    }

    assert video_device_candidates(
        "auto",
        globber=lambda pattern: matches[pattern],
        realpath=lambda path: targets[path],
    ) == ["/dev/v4l/by-id/usb-camera-video-index0", "/dev/video0"]

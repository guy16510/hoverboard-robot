from __future__ import annotations

import glob
import os
from collections.abc import Callable, Iterable


def video_device_candidates(
    requested: str,
    *,
    globber: Callable[[str], Iterable[str]] = glob.glob,
    realpath: Callable[[str], str] = os.path.realpath,
) -> list[str]:
    """Return deterministic V4L2 capture candidates, preferring stable symlinks."""
    requested = requested.strip()
    if requested and requested.lower() != "auto":
        return [requested]

    paths: list[str] = []
    for pattern in (
        "/dev/v4l/by-id/*-video-index0",
        "/dev/v4l/by-path/*-video-index0",
        "/dev/video*",
    ):
        paths.extend(sorted(globber(pattern)))

    candidates: list[str] = []
    seen_targets: set[str] = set()
    for path in paths:
        target = realpath(path)
        if target in seen_targets:
            continue
        seen_targets.add(target)
        candidates.append(path)
    return candidates

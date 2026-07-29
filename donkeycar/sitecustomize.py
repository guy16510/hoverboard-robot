"""Prefer Raspberry Pi OS hardware-enabled Python modules over pip shadows.

The image uses a system-site-packages virtual environment. Donkeycar may pull a
basic pip OpenCV build that lacks cv2.aruco, so put Debian's validated hardware
build first without changing imports throughout the application.
"""

from __future__ import annotations

import sys
from pathlib import Path

SYSTEM_DIST_PACKAGES = "/usr/lib/python3/dist-packages"

if Path(SYSTEM_DIST_PACKAGES).is_dir():
    while SYSTEM_DIST_PACKAGES in sys.path:
        sys.path.remove(SYSTEM_DIST_PACKAGES)
    sys.path.insert(0, SYSTEM_DIST_PACKAGES)

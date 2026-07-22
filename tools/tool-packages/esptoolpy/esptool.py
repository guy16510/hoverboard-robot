#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
# Offline image-generation wrapper. Build scripts never pass serial, erase,
# write, upload, or chip-discovery subcommands.
import os
import sys

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path = [path for path in sys.path if os.path.abspath(path) != script_dir]
sys.modules.pop("esptool", None)

import esptool

if __name__ == "__main__":
    esptool._main()

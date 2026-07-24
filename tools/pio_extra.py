# SPDX-License-Identifier: GPL-3.0-only
Import("env")

from pathlib import Path
from shutil import copyfile

pio_environment = env.subst("$PIOENV")

if pio_environment != "native_tests":
    map_path = env.subst("$BUILD_DIR/firmware.map")
    env.Append(LINKFLAGS=["-Wl,-Map," + map_path])


def copy_esp32_boot_app0(source, target, env):
    del source, target
    framework_dir = env.PioPlatform().get_package_dir("framework-arduinoespressif32")
    source_path = Path(framework_dir) / "tools" / "partitions" / "boot_app0.bin"
    target_path = Path(env.subst("$BUILD_DIR")) / "boot_app0.bin"
    copyfile(source_path, target_path)


if pio_environment.startswith("esp32_"):
    env.AddPostAction("$BUILD_DIR/firmware.bin", copy_esp32_boot_app0)

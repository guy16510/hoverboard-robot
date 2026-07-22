# SPDX-License-Identifier: GPL-3.0-only
Import("env")

if env.subst("$PIOENV") != "native_tests":
    map_path = env.subst("$BUILD_DIR/firmware.map")
    env.Append(LINKFLAGS=["-Wl,-Map," + map_path])

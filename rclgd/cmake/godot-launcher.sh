#!/usr/bin/env bash
# Godot launcher for rclgd, installed as lib/rclgd/godot (used by
# `ros2 run rclgd godot`, the per-package shims and `ros2 rclgd editor`).
#
# The engine binary is installed by the rclgd build as lib/rclgd/godot-bin —
# right next to librclgd.so, which is where GDExtension resolves the bare
# "librclgd.so" path in rclgd.gdextension.
set -u

DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"

if [ -x "$DIR/godot-bin" ]; then
    exec "$DIR/godot-bin" "$@"
fi

VERSION="$(cat "$DIR/../../share/rclgd/godot_version" 2>/dev/null || echo '?')"
echo "rclgd: Godot $VERSION binary missing from $DIR — broken install." >&2
echo "hint: rebuild rclgd (the engine is installed by the build)." >&2
exit 2

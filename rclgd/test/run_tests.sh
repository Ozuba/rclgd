#!/usr/bin/env bash
# Runs the headless rclgd system tests against a built librclgd.so.
#
#   run_tests.sh <godot-binary> <librclgd.so> <project-dir> [runner args...]
#
# The project is copied to a scratch directory and wired to the given library,
# so tests run straight from the build tree. Runner args (--junit=<file>,
# --filter=<text>) are passed through to runner.gd.
set -euo pipefail

godot="$1"
lib="$2"
project="$3"
shift 3

work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT

cp -r "$project/." "$work"
ln -s "$(readlink -f "$lib")" "$work/librclgd.so"
cat > "$work/rclgd.gdextension" <<'EOF'
[configuration]
entry_symbol = "rclgd_init"
compatibility_minimum = "4.7"

[libraries]
linux = "librclgd.so"
EOF
# Registering the extension directly replaces the editor import pass, which a
# scripts-only project doesn't otherwise need.
mkdir -p "$work/.godot"
echo "res://rclgd.gdextension" > "$work/.godot/extension_list.cfg"

# A random domain on localhost keeps concurrent runs, and any ROS system
# already running on this machine, out of the tests.
export ROS_DOMAIN_ID="$((RANDOM % 100 + 1))"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

"$godot" --headless --path "$work" --script res://runner.gd -- "$@"

#!/bin/bash
# Get the directory of the script (where the executable is)
DIR=$(dirname "$(realpath "$0")")

# Set the working directory to where the Godot project is located (install/share/rclgd_demo)
PROJECT_DIR="../../share/rclgd/rclgd_demo"

# Get into this directory to launch godot
cd "$DIR"

# Run the Godot executable with the project path
./godot --path "$PROJECT_DIR" "--" "$@" # Replace with your actual executable if needed
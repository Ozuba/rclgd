#!/bin/bash

# This script downloads the architecture-specific Godot executable,
# places it in a target directory, and names it "godot".

set -euo pipefail

# === Arguments ===
# $1: The directory where the final 'godot' executable should be placed (e.g., CMAKE_BINARY_DIR)
# $2: The architecture suffix (e.g., x86_64 or arm64)

if [ "$#" -ne 2 ]; then
  echo "Error: Two arguments are required: <target_dir> <arch_suffix>"
  exit 1
fi

GODOT_EXE_DIR="$1"
ARCH_SUFFIX="$2"

# === Definitions ===
GODOT_VERSION="4.4.1-stable"
TARGET_GODOT_NAME="godot"
FINAL_EXE_PATH="${GODOT_EXE_DIR}/${TARGET_GODOT_NAME}"

FILE_NAME="Godot_v${GODOT_VERSION}_linux.${ARCH_SUFFIX}"
ZIP_NAME="${FILE_NAME}.zip"
URL="https://github.com/godotengine/godot-builds/releases/download/4.4.1-stable/${ZIP_NAME}"

ZIP_PATH="${GODOT_EXE_DIR}/${ZIP_NAME}"

# === Step 1: Check if the final executable already exists ===
if [ -f "$FINAL_EXE_PATH" ]; then
  echo "Godot executable already exists at ${FINAL_EXE_PATH}. Skipping download."
  exit 0
fi

echo "Downloading Godot ${ARCH_SUFFIX} from ${URL}..."

# === Step 2: Download, Extract, and Rename ===

# Download the zip file
curl -fSL "$URL" -o "$ZIP_PATH" || { echo "Download failed for $URL"; exit 1; }

# Unzip into the target directory
unzip -q "$ZIP_PATH" -d "$GODOT_EXE_DIR" || { echo "Extraction failed for $ZIP_PATH"; exit 1; }

# Rename the specific downloaded file to the generic "godot"
mv "${GODOT_EXE_DIR}/${FILE_NAME}" "$FINAL_EXE_PATH" || { echo "Failed to rename executable"; exit 1; }

# Clean up the zip file
rm -f "$ZIP_PATH" || { echo "Failed to remove $ZIP_PATH"; exit 1; }

# Ensure the executable has correct permissions
chmod +x "$FINAL_EXE_PATH"

echo "Godot successfully installed as ${FINAL_EXE_PATH}"
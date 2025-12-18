#!/bin/bash

# This script FORCES a re-download and replacement of the Godot executable.
set -euo pipefail

# === Arguments ===
if [ "$#" -ne 2 ]; then
  echo "Error: Two arguments are required: <target_dir> <arch_suffix>"
  exit 1
fi

GODOT_EXE_DIR="$1"
ARCH_SUFFIX="$2"

# === Definitions ===
GODOT_VERSION="4.5.1"
VERSION_STATUS="stable"
TARGET_GODOT_NAME="godot"

# Naming and URL construction
FILE_NAME="Godot_v${GODOT_VERSION}-${VERSION_STATUS}_linux.${ARCH_SUFFIX}"
ZIP_NAME="${FILE_NAME}.zip"
URL="https://github.com/godotengine/godot/releases/download/${GODOT_VERSION}-${VERSION_STATUS}/${ZIP_NAME}"

ZIP_PATH="${GODOT_EXE_DIR}/${ZIP_NAME}"
FINAL_EXE_PATH="${GODOT_EXE_DIR}/${TARGET_GODOT_NAME}"

# === Step 1: Force Cleanup ===
echo "Force cleaning existing Godot files..."
rm -f "$FINAL_EXE_PATH"
rm -f "$ZIP_PATH"
mkdir -p "$GODOT_EXE_DIR"

# === Step 2: Download, Extract, and Replace ===
echo "Downloading Godot ${GODOT_VERSION} (${ARCH_SUFFIX}) from ${URL}..."

# -fSL ensures it fails on 404 and follows redirects
curl -fSL "$URL" -o "$ZIP_PATH" || { echo "Download failed!"; exit 1; }

echo "Extracting..."
unzip -q -o "$ZIP_PATH" -d "$GODOT_EXE_DIR" || { echo "Unzip failed!"; exit 1; }

# Atomic move and rename
EXTRACTED_FILE="${GODOT_EXE_DIR}/${FILE_NAME}"
if [ -f "$EXTRACTED_FILE" ]; then
    mv -f "$EXTRACTED_FILE" "$FINAL_EXE_PATH"
else
    # Fallback to catch any Godot binary in the zip
    find "$GODOT_EXE_DIR" -maxdepth 1 -name "Godot_v*" -type f -not -name "*.zip" -exec mv -f {} "$FINAL_EXE_PATH" \;
fi

# Cleanup and Permissions
rm -f "$ZIP_PATH"
chmod +x "$FINAL_EXE_PATH"

echo "Godot successfully replaced at: ${FINAL_EXE_PATH}"
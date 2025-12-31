#!/bin/bash

# build_ws.sh (Raspberry Pi)

set -e

echo "[Pi Setup] Building robot_ws..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR/../../robot_ws"

if [ -d "$WORKSPACE_DIR" ]; then
    cd "$WORKSPACE_DIR"
    source /opt/ros/jazzy/setup.bash
    
    # Delete old build artifacts for clean build if desired (optional)
    # rm -rf build install log

    colcon build --symlink-install
    echo "[Pi Setup] Build Complete!"
    echo "Source with: source $WORKSPACE_DIR/install/setup.bash"
else
    echo "Error: robot_ws not found at $WORKSPACE_DIR"
    exit 1
fi

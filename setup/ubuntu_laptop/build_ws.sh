#!/bin/bash

set -e

# Build View Workspace
echo "Building view_ws..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR/../../view_ws"

if [ -d "$WORKSPACE_DIR" ]; then
    cd "$WORKSPACE_DIR"
    source /opt/ros/jazzy/setup.bash
    colcon build --symlink-install
    echo "Workspace built successfully!"
    echo "Done! Please verify by running: source $WORKSPACE_DIR/install/setup.bash"
else
    echo "Error: view_ws directory not found at $WORKSPACE_DIR"
    exit 1
fi
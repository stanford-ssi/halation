#!/bin/bash
set -e
set -o pipefail

source /opt/ros/humble/setup.bash
[[ -f /workspace/install/setup.bash ]] && source /workspace/install/setup.bash
[[ -f ./install/setup.bash ]] && source ./install/setup.bash

ros2 run rosapi rosapi_node &
ROSAPI_PID=$!

cleanup() {
    echo "Shutting down rosapi..."
    kill $ROSAPI_PID 2>/dev/null || true
}
trap cleanup EXIT

exec ros2 launch rover_bringup rover_system.launch.py
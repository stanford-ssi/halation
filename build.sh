#!/bin/bash

source /opt/ros/humble/setup.bash
colcon build --symlink-install
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi
if [ -f ./install/setup.bash ]; then
    source ./install/setup.bash
fi
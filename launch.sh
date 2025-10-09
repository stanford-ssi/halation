#!/bin/bash

source /opt/ros/humble/setup.bash
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi
if [ -f ./install/setup.bash ]; then
    source ./install/setup.bash
fi
ros2 launch rover_bringup rover_system.launch.py
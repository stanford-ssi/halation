#!/bin/bash

source /opt/ros/humble/setup.bash
rosdep update
colcon build --symlink-install
source /workspace/install/setup.bash
#!/bin/bash
set -e

# start sshd
/usr/sbin/sshd

# setup ros for non-interactive mode commands
source /opt/ros/${ROS_DISTRO}/setup.bash
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi

# execute the command passed to docker run
exec "$@"
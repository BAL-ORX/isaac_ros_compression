#!/usr/bin/env bash

# Source before strict mode: ROS setup scripts may reference unset variables,
# which would trigger set -u and silently kill the script before colcon runs.
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash 2>/dev/null || true
source /opt/ros_ws/install/setup.bash 2>/dev/null || true

set -euo pipefail

colcon build \
    --symlink-install \
    --base-paths src/ \
    --cmake-args=-DCMAKE_BUILD_TYPE=Release \
    --parallel-workers $(nproc)

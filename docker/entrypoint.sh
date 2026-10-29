#!/bin/bash
# Build the mounted workspace, then run the container command (default: the nav launch).
set -e
source /opt/ros/humble/setup.bash
cd /ros2_ws
colcon build --symlink-install --base-paths src    # every start; only our src/, not utils/
source install/setup.bash
exec "$@"

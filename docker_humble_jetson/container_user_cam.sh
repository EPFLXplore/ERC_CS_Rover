#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
cd /home/xplore/dev_ws
# Persistent volume keeps old install/ — rebuild so bind-mounted ERC_CS_Rover source is used.
# echo "colcon build --packages-select camera --symlink-install (mounted src)..."
# colcon build --packages-select camera --symlink-install
source install/setup.bash
ros2 launch camera camera_node_cs.launch.py &
exec bash -i

cd ../rover_ws
colcon build
source install/setup.bash
ros2 launch rover_pkg rover_start

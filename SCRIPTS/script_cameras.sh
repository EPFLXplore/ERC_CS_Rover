cd ../rover_ws
colcon build
source install/setup.bash
ros2 run rover_pkg cameras_publisher

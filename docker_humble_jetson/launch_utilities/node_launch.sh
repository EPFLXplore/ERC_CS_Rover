if [ "$1" == "rover" ]; then
    echo "-------- Launching Rover --------"
    colcon build
    source install/setup.bash && echo "--------- Sourced ---------"
    ros2 run rover_pkg rover

elif [ "$1" == "nav" ]; then
    echo "Launching Navigation (Only Wheels Control)"
    source install/setup.bash && echo "--------- Sourced ---------"
    ros2 launch wheels_control wheels_control.launch.xml

elif [ "$1" == "hd" ]; then
    echo "Launching Handling Device"
else
    echo "Not a valid Subsystem"
fi
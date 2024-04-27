if [ "$1" == "rover" ]; then
    echo "Attaching to Rover"
elif [ "$1" == "nav" ]; then
    echo "Attaching to Navigation"
elif [ "$1" == "hd" ]; then
    echo "Attaching to Handling Device"
else
    echo "Not a valid Docker Subsystem"
    exit
fi


docker exec -it $1_humble_desktop bash
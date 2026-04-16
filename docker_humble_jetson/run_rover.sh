#!/bin/bash

XAUTH=/tmp/.docker.xauth
USERNAME=xplore
CONTAINER_NAME=rover_humble_jetson
IMAGE_NAME=ghcr.io/epflxplore/rover:humble-jetson
# chown must run as root (no sudo in image); ros2 runs as xplore.
DOCKER_RUN_COMMAND="chown -R $USERNAME:$USERNAME /home/$USERNAME && exec runuser -u $USERNAME -- /bin/bash -c 'source src/docker_humble_jetson/attach.sh; ros2 launch rover_pkg launch.py'"
DOCKER_EXEC_COMMAND="source src/docker_humble_jetson/attach.sh; ros2 launch rover_pkg launch.py"

# Function to check if a Docker container is running
is_container_running() {
    if [ "$(docker inspect -f '{{.State.Running}}' "$1" 2>/dev/null)" == "true" ]; then
        echo "true"
    else
        echo "false"
    fi
}

# Prepare Xauthority data
echo "Preparing Xauthority data..."
xauth_list=$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/')
if [ ! -f $XAUTH ]; then
    if [ ! -z "$xauth_list" ]; then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

echo "Done."
echo ""
echo "Verifying file contents:"
file $XAUTH
echo "--> It should say \"X11 Xauthority data\"."
echo ""
echo "Permissions:"
ls -FAlh $XAUTH
echo ""

# Get the current working directory and parent directory
current_dir=$(pwd)
parent_dir=$(dirname "$current_dir")

# Check if the container is running
container_status=$(is_container_running "$CONTAINER_NAME")


if [ "$container_status" == "false" ]; then
    echo "Container $CONTAINER_NAME is not running. Starting a new container..."
    docker run -i \
        --user root \
        --name $CONTAINER_NAME \
        --rm \
        --privileged \
        --net=host \
        -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
        -e DISPLAY=unix$DISPLAY \
        -e QT_X11_NO_MITSHM=1 \
        -e XAUTHORITY=$XAUTH \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v $XAUTH:$XAUTH \
        -v /run/user/1000/at-spi:/run/user/1000/at-spi \
        -v /dev:/dev \
        -v /home/xplore/photos_competition:/home/xplore/dev_ws/photos_competition \
        -v $parent_dir:/home/$USERNAME/dev_ws/src \
        -v rover_humble_jetson_home_volume:/home/$USERNAME \
        $IMAGE_NAME \
        /bin/bash -c "$DOCKER_RUN_COMMAND"
else
    echo "Container $CONTAINER_NAME is already running. Attaching to it..."
    docker exec -it -u "$USERNAME" -w "/home/$USERNAME/dev_ws" $CONTAINER_NAME \
        /bin/bash -c "$DOCKER_EXEC_COMMAND"
fi
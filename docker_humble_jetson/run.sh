# If not working, first do: sudo rm -rf /tmp/.docker.xauth
# If still not working, try running the script as root.

XAUTH=/tmp/.docker.xauth

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
echo "Running docker..."

# Get the current working directory
current_dir=$(pwd)

# Use dirname to get the parent directory
parent_dir=$(dirname "$current_dir")

JTOP_GID=$(getent group jtop | awk -F: '{print $3}')

docker run -it \
    --name rover_humble_jetson \
    --rm \
    --privileged \
    --net=host \
    --group-add $JTOP_GID \
    -e DISPLAY=unix$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $XAUTH:$XAUTH \
    -v /run/user/1000/at-spi:/run/user/1000/at-spi \
    -v /run/jtop.sock:/run/jtop.sock \
    -v /dev:/dev \
    -v $parent_dir:/home/xplore/dev_ws/src \
    -v rover_humble_jetson_home_volume:/home/xplore \
    ghcr.io/epflxplore/rover:humble-jetson \
    /bin/bash -c "sudo chown -R $USERNAME:$USERNAME /home/$USERNAME; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; /bin/bash"

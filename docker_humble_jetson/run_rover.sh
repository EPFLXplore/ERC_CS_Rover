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

USERNAME=xplore

# Get the current working directory
current_dir=$(pwd)

# Use dirname to get the parent directory and export variables
export PARENT_DIR=$(dirname "$current_dir")
export XAUTH=$XAUTH
export JTOP_GID=$(getent group jtop | awk -F: '{print $3}')

/usr/bin/docker rm -f mongodb

/usr/bin/docker compose -f compose.yaml up -d

/usr/bin/docker exec rover_humble_jetson /bin/bash -c "ros2 run rover_pkg new_rover" 
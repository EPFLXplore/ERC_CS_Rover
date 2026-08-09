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

# Rover camera udev nodes
ROVER_CAMERAS=(
    "/dev/top_cam"
    "/dev/right_steer_cam"
    "/dev/left_steer_cam"
    "/dev/drill_cam_inside"
)

echo "Checking rover cameras..."
for cam in "${ROVER_CAMERAS[@]}"; do
    if [ -e "$cam" ]; then
        target=$(readlink -f "$cam")
        echo -e "\033[0;32m$(basename "$cam") detected: $cam -> $target\033[0m"
    else
        echo -e "\033[0;31m$(basename "$cam") NOT detected! still going in docker\033[0m"
    fi
done
echo ""

# GID owning the V4L device on the host — map into container and use as primary group for xplore
# NOTE: stat on a udev symlink returns the symlink's gid, often 0, unless -L is used.
HOST_VIDEO_GID=""

for cam in "${ROVER_CAMERAS[@]}"; do
    if [ -e "$cam" ]; then
        HOST_VIDEO_GID=$(stat -L -c '%g' "$cam")
        break
    fi
done

if [ -z "$HOST_VIDEO_GID" ] && getent group video >/dev/null 2>&1; then
    HOST_VIDEO_GID=$(getent group video | cut -d: -f3)
fi

DOCKER_DEVICE_ARGS=()
for cam in "${ROVER_CAMERAS[@]}"; do
    if [ -e "$cam" ]; then
        DOCKER_DEVICE_ARGS+=(--device="$cam")
    fi
done

echo "Running docker..."

USERNAME=xplore

# Get the current working directory
current_dir=$(pwd)

# Use dirname to get the parent directory and export variables
export PARENT_DIR=$(dirname "$current_dir")
export XAUTH=$XAUTH

# Start as root so we can chown the mounted volume without sudo.
docker run -it \
    "${DOCKER_DEVICE_ARGS[@]}" \
    --user root \
    --name rover_humble_jetson \
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
    -v $PARENT_DIR:/home/xplore/dev_ws/src \
    -v rover_humble_jetson_home_volume:/home/xplore \
    -v $current_dir/cyclonedds.xml:/cyclone.xml:ro \
    -e CYCLONEDDS_URI="file:///cyclone.xml" \
    -e HOST_VIDEO_GID="$HOST_VIDEO_GID" \
    ghcr.io/epflxplore/rover:humble-jetson \
    /bin/bash -c "$(cat <<'ROOTINIT'
if [ -n "$HOST_VIDEO_GID" ]; then
  GNAME=$(getent group "$HOST_VIDEO_GID" | cut -d: -f1)
  if [ -z "$GNAME" ]; then
    GNAME=rovervideo
    groupadd -g "$HOST_VIDEO_GID" "$GNAME" || true
  fi
  usermod -g "$GNAME" xplore
  usermod -aG "$GNAME" xplore
  getent group xplore >/dev/null 2>&1 && usermod -aG xplore xplore
fi
chown -R xplore:xplore /home/xplore
exec runuser -u xplore -- /bin/bash
ROOTINIT
)"
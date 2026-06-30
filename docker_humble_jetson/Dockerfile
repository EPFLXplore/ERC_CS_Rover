###########################################
# Xplore Minimal Raspberry Pi 5 Image with ros2 humble
# Image size: 2.61GB 
###########################################
FROM ros:humble-ros-base

ARG DEBIAN_FRONTEND=noninteractive

# Use Cyclone DDS as the default RMW (Fast DDS is the ros-base default without this)
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Official ros:humble-ros-base already ships the ROS 2 apt repo; no keyring refresh needed.

# Install Rover packages (+ common network debugging: capture, paths, DNS, link stats)
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    dnsutils \
    ethtool \
    ffmpeg \
    iproute2 \
    iputils-ping \
    iperf3 \
    mtr-tiny \
    net-tools \
    netcat-openbsd \
    libopencv-dev \
    python3-opencv \
    python3-pip \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    socat \
    tcpdump \
    traceroute \
    wget \
 && rm -rf /var/lib/apt/lists/*
RUN useradd -m -s /bin/bash xplore

# Add USB rules (minimal base images may not ship /etc/udev/rules.d)
RUN mkdir -p /etc/udev/rules.d \
    && echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' > /etc/udev/rules.d/80-movidius.rules
RUN /etc/init.d/udev restart || true

WORKDIR /home/xplore/dev_ws/src
COPY . .
RUN chown -R xplore:xplore /home/xplore/dev_ws

WORKDIR /home/xplore/dev_ws

# Install ROS 2 Nav dependencies: rosdep warns if `rosdep update` runs as root — run it only
# as xplore after making the system rosdep dir writable (so cache stays in /etc/ros/rosdep for `rosdep install`).
RUN mkdir -p /etc/ros/rosdep \
    && chown -R xplore:xplore /etc/ros/rosdep \
    && runuser -u xplore -- rosdep update \
    && rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} -y -r --skip-keys="depthai opencv4" \
 && rm -rf /var/lib/apt/lists/*

# Remove all the confidential Xplore source code from the image
RUN rm -rf /home/xplore/dev_ws/src/*

USER xplore
WORKDIR /home/xplore

RUN pip install --no-cache-dir numpy
# RUN pip install --no-cache-dir aiohttp aiortc aiohttp_cors
# RUN pip install --no-cache-dir ping3 tornado requests bson pymongo

WORKDIR /home/xplore/dev_ws

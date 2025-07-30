###########################################
# Xplore Navigation image
###########################################
FROM ghcr.io/epflxplore/docker_commons:humble-jetson

ARG DEBIAN_FRONTEND=noninteractive

RUN rm /usr/share/keyrings/ros-archive-keyring.gpg
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN apt update && apt install ros2-apt-source  
RUN rm /etc/apt/sources.list.d/ros2.list
RUN rm /usr/share/keyrings/ros-archive-keyring.gpg

# Install Rover packages
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y curl
RUN apt-get install -y python3-opencv
RUN apt-get install -y ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup

# Add USB rules
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
RUN /etc/init.d/udev restart


USER $USERNAME

WORKDIR /home/$USERNAME

RUN pip install numpy
RUN pip install aiohttp aiortc aiohttp_cors
RUN pip install ping3 tornado requests pyrealsense2 bson pymongo

# Set a diretory to store the project
WORKDIR /home/$USERNAME/dev_ws/src
COPY . .

# Set a directory to build the project
WORKDIR /home/$USERNAME/dev_ws

# Install ROS 2 Nav dependencies
RUN rosdep update \
    && rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y -r

# Clean up
RUN sudo rm -rf /var/lib/apt/lists/*

# Remove all the confidential Xplore source code from the image
RUN sudo rm -rf /home/$USERNAME/dev_ws/src/*

###########################################
# Xplore Navigation image
###########################################
FROM ghcr.io/epflxplore/docker_commons:humble-jetson

ARG DEBIAN_FRONTEND=noninteractive

# Install Rover packages
RUN apt-get update && apt-get upgrade -y
RUN apt-get install ros-${ROS_DISTRO}-nav-msgs -y
RUN apt install -y curl
RUN apt-get install -y python3-opencv
RUN pip install -U jetson-stats
RUN apt install ros-humble-rmw-cyclonedds-cpp -y

# Add USB rules
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
RUN /etc/init.d/udev restart

# Install rosidl to fix issue with compilation of custom actions (error rosidl_generator)
WORKDIR /home/$USERNAME/rosidl_ws/src
WORKDIR /home/$USERNAME/rosidl_ws
RUN git clone https://github.com/ros2/rosidl.git --branch 3.1.5 ./src/rosidl
RUN rosdep update && rosdep install -y --from-paths ./src --ignore-src
RUN colcon build --symlink-install

USER $USERNAME

WORKDIR /home/$USERNAME

RUN python3 -m pip install numpy
RUN pip install aiohttp aiortc aiohttp_cors
RUN pip install pymongo ping3

# Set a diretory to store the project
WORKDIR /home/$USERNAME/dev_ws/src
COPY . .

# Set a directory to build the project
WORKDIR /home/$USERNAME/dev_ws

# Install ROS 2 Nav dependencies
RUN rosdep update \
    && rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y --skip-keys "libg2o g2o"

# Clean up
RUN sudo rm -rf /var/lib/apt/lists/*

# Remove all the confidential Xplore source code from the image
RUN sudo rm -rf /home/$USERNAME/dev_ws/src/*

FROM --platform=linux/arm64 docker.io/arm64v8/ros:humble-ros-base

# Basic dev tools + colcon + ROS tools
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-humble-actuator-msgs \
    vim \
    can-utils \
    iproute2 \
    sudo \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp/ws
FROM docker.io/arm64v8/ros:humble-ros-base
# FROM ros:humble-ros-base

# Install dependencies for building and joystick support
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-joy \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    sudo \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp

RUN git clone https://github.com/gsl-lite/gsl-lite && cd gsl-lite && \
    mkdir build && cd build && \
    cmake .. && make -j8 && \
    sudo make install

RUN git clone https://github.com/catchorg/Catch2 && cd Catch2 && \
    mkdir build && cd build && \
    cmake .. && make -j8 && \
    sudo make install

RUN git clone https://github.com/fmtlib/fmt && cd fmt && \
    mkdir build && cd build && \
    cmake -DFMT_TEST=OFF .. && \
    make -j8 && \
    sudo make install

RUN mkdir -p /tmp/colcon_ws/src
WORKDIR /tmp/colcon_ws/src
RUN git clone -b flix --recursive https://github.com/Kevin-Gu-2022/viper

WORKDIR /tmp/colcon_ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build

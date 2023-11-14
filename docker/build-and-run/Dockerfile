FROM docker.io/arm64v8/ros:humble-ros-base

COPY start.sh /
RUN chmod ugo+x /start.sh

RUN mkdir -p /tmp/colcon_ws/src
WORKDIR /tmp/colcon_ws/src
RUN git clone --recursive https://github.com/107-systems/viper

WORKDIR /tmp/colcon_ws
RUN . /opt/ros/humble/setup.sh && \
    colcon_ws build --symlink-install

ENTRYPOINT ["/start.sh"]

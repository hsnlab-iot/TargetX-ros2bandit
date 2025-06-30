FROM osrf/ros:jazzy-desktop

RUN \
 apt update && \
 apt install -y --no-install-recommends dialog apt-utils curl software-properties-common apt-transport-https wget git iputils-ping net-tools iproute2 vim build-essential ssh tmux tcpdump gdb busybox;

RUN \
 busybox --install

RUN \
 # Install dependencies
 apt update && \
 apt install -y --no-install-recommends \
   ros-jazzy-rmw-cyclonedds-cpp \
   python3-colcon-common-extensions \
   ros-jazzy-std-msgs ros-jazzy-geometry-msgs ros-jazzy-tf2-msgs ros-jazzy-nav2-msgs ros-jazzy-tf2-geometry-msgs

SHELL ["bash", "-c"]

COPY src /opt/ros2_ws/
COPY docker-entrypoint.sh /usr/bin

RUN \
 . /opt/ros/jazzy/setup.sh && \
 cd /opt/ros2_ws && \
 colcon build

ENTRYPOINT [ "bash", "/usr/bin/docker-entrypoint.sh" ]

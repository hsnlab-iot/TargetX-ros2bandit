FROM ros2base:humble

RUN \
 # Install dependencies
 apt update && \
 apt install -y --no-install-recommends \
   python3-colcon-common-extensions libczmq-dev nlohmann-json3-dev python3-shapely
   
SHELL ["bash", "-c"]

RUN \
 git clone https://github.com/zeromq/cppzmq.git && \
 cd cppzmq && \
 mkdir build && \
 cd build && \
 cmake -DCPPZMQ_BUILD_TESTS=OFF .. && \
 make -j4 install

RUN apt install -y --no-install-recommends python3-zmq

#COPY src /opt/ros2_ws/
COPY docker-entrypoint.sh /usr/bin

#RUN \
# . /opt/ros/jazzy/setup.sh && \
# cd /opt/ros2_ws && \
# colcon build

ENTRYPOINT [ "bash", "/usr/bin/docker-entrypoint.sh" ]

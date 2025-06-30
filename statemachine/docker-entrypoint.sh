#!/bin/bash

#cp /opt/ros2_ws/build/rmw_bandit/librmw_bandit.so /host/build

source /opt/ros/${ROS_DISTRO}/setup.bash

tmux new-session -s "SM" -d "cd /opt/ros2_ws/state_machine_node; python3 state_machine_node.py; sleep inf"

tail -f /dev/null

#!/bin/bash

export  LD_LIBRARY_PATH=/rmw_bandit
ldconfig
source /opt/ros2_ws/install/setup.bash

# Start node using bandit
export RMW_IMPLEMENTATION=rmw_bandit
export RMW_IMPLEMENTATION_WRAP=rmw_fastrtps_cpp
tmux new-session -s 1 -d "ros2 launch mock_control_pkg control_launch.py; sleep inf"

tail -f /dev/null
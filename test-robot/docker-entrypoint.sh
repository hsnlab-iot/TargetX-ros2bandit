#!/bin/sh

# Environment variables are coming from the docker-compose file
export  LD_LIBRARY_PATH=/rmw_bandit
ldconfig
source /opt/ros2_ws/install/setup.bash

tmux new-session -s 1 -d "ros2 run examples_rclpy_minimal_action_server server; sleep inf"
tmux new-session -s 2 -d "ros2 launch mock_robot_pkg robot_launch.py; sleep inf"

tail -f /dev/null
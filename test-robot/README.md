# ROS2-BANDIT: Test Robot Container

This container simulates a mobile robot (rover) as part of the `ros2bandit` test environment. It publishes mock video data, simulates motion via `cmd_vel`, and continuously broadcasts a transform (`TF`) representing its movement pattern. This enables realistic testing of ROS2-BANDIT's topic control, monitoring, and state machine features.

## Features

- Simulated robot node (`simulated_rover_node`)
- Periodic publication of synthetic video frames (`video_topic`)
- Circular motion pattern with dynamic `cmd_vel` output
- TF broadcaster emitting robot pose relative to map frame
- Action server for test interaction via `/fibonacci` goal handling
- Full integration with `rmw_bandit` middleware via environment

## Usage

This container is typically started via `docker-compose` alongside the `rmw_bandit`, `bandit_broker`, and `test-control` containers. See the top-level README for full orchestration.

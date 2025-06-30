# mock_control_pkg (test-control)

This directory contains a minimal ROS 2-based simulated control node (`mock_control_pkg`) used for testing the `rmw_bandit` wrapper and topic-based communication control in ROS2-BANDIT.

## Overview

The node emulates a controller that:
- Subscribes to a mock video stream (`video_topic`).
- Publishes control signals (`control` topic) at 100Hz.
- Sends periodic Fibonacci action goals to a remote action server.


## Usage

This container is typically started via `docker-compose` alongside the `rmw_bandit`, `bandit_broker`, and `test-robot` containers. See the top-level README for full orchestration.
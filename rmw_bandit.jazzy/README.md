# rmw_bandit (jazzy)

`rmw_bandit` is a custom RMW (ROS 2 Middleware) implementation that integrates ZeroMQ-based dynamic control and monitoring of ROS 2 topics, allowing external brokers and controllers to observe, block/unblock, and watch traffic at runtime. It enables runtime introspection and reactive behavior based on system context.

## Features

- Acts as a wrapper around existing RMW implementations (e.g., `rmw_fastrtps_cpp`)
- Monitors publisher traffic (bandwidth and frequency)
- Allows topics to be dynamically blocked/unblocked
- Sends metadata to a ZMQ broker for live control
- Supports state-machine-based controllers that can react to topic triggers and robot location
- Provides internal monitoring via threads for updating statistics and publishing state
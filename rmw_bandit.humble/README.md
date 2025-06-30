# rmw_bandit

`rmw_bandit` is a custom RMW (ROS 2 Middleware) implementation that integrates ZeroMQ-based dynamic control and monitoring of ROS 2 topics, allowing external brokers and controllers to observe, block/unblock, and watch traffic at runtime. It enables runtime introspection and reactive behavior based on system context.

## Features

- Acts as a wrapper around existing RMW implementations (e.g., `rmw_fastrtps_cpp`)
- Monitors publisher traffic (bandwidth and frequency)
- Allows topics to be dynamically blocked/unblocked
- Sends metadata to a ZMQ broker for live control
- Supports state-machine-based controllers that can react to topic triggers and robot location
- Provides internal monitoring via threads for updating statistics and publishing state

## Components

### `bandit_broker/`
- Contains the ZeroMQ-based broker (`broker.py`) and helper scripts (`block.py`, `unblock.py`, `watch.py`) that control topic publishing remotely.

### `rmw_bandit/`
- Contains the actual RMW implementation (`rmw_bandit.cpp`) that wraps another RMW and adds traffic interception logic.
- Tracks published topics, bandwidth, and frequencies.
- Connects to a ZMQ dealer socket and listens for control commands.

### `state_machine_node/`
- Implements a ROS 2 node that loads a YAML configuration of states, triggers, and areas.
- Reacts to robot location and topic triggers using TF2, Shapely, and ZMQ.

## Usage

1. **Build the RMW implementation in Docker**:
    ```bash
    docker build -t rmw_bandit_image .
    ```

2. **Run the broker**:
    ```bash
    python3 bandit_broker/broker.py
    ```

3. **Launch a ROS 2 system using `RMW_IMPLEMENTATION=rmw_bandit`**.

## Environment Variables

- `RMW_IMPLEMENTATION_WRAP`: Underlying RMW to wrap (default: `rmw_fastrtps_cpp`)
- `ROS2BANDIT_BROKER`: IP address of the broker (default: `127.0.0.1`)
- `ROS2BANDIT_BROKER_PORT`: ZMQ port of the broker (default: `1884`)

## License

MIT or TODO.

## Author

Attila Vidács <vidacs@tmit.bme.hu>
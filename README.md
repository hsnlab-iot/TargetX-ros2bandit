# TargetX-ROS2BANDIT Repository

This repository provides a standalone deployment and demonstration setup for the **ROS2-BANDIT** framework, developed under the TARGET-X Open Call project: ROS2 Bandwidth Aware Networking Dynamics and Integration Toolkit (ROS2-BANDIT). It includes the ROS 2 RMW wrapper, centralized broker backend, frontend dashboard, and example robotic test containers.

## Repository Structure

- **`libs/`** – The dynamic middleware library files (.so) for various ROS2 distributions.
- **`statemachine/`** - State Machine node implementation in Python.
- **`bandit_broker/`** – The backend (broker + API server) and the frontend web application.
- **`test-control/`** – A simulated control node publishing mock control signals.
- **`test-robot/`** – A simulated robot that publishes video and TF data.

## Usage Overview

This system is designed to demonstrate the runtime blocking/unblocking of ROS 2 topics using dynamic policies and ZeroMQ-based coordination. The RMW wrapper `rmw_bandit` intercepts publishing and enables per-topic filtering.

### Quick Start

Build and launch the full setup with Docker Compose:

```bash
docker-compose up --build
```

## Notes

- Environment variables like `RMW_IMPLEMENTATION`, `ROS2BANDIT_BROKER`, and `ROS2BANDIT_DOMAIN` are configured in `docker-compose.yml`.
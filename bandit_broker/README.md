# bandit_broker

This folder contains both the **backend control service** and the **frontend web dashboard** for the ROS2-BANDIT system. Together, these components enable real-time monitoring, control, and visualization of bandwidth-aware robotic communication policies.

## Overview

- **Backend (`backend/`)**: Implements the core ZeroMQ broker and the Flask-based telemetry API.
- **Frontend (`frontend/`)**: A React + Tailwind-based web dashboard that communicates with the backend via REST and SSE.

---

## Backend

The backend consists of:
- A ZeroMQ-based **message router** for communicating with `rmw_bandit` and controller nodes.
- A Flask-based **HTTP API** to serve telemetry, topic traffic, state machine visualizations, and live events.
- A `tmux`-based runtime logging system and named pipes for diagnostics.

### Key Files

- `main.py`: Orchestrates both broker and web server
- `broker.py`: ZMQ router and policy manager
- `webapi.py`: Flask REST + SSE API
- `shared_memory.py`: Shared memory access using `multiprocessing.Manager`

---

## Frontend

The frontend is a React dashboard compiled with Vite, styled with Tailwind CSS, and deployed via Nginx. 

---

## Developer Notes

- Backend logs are captured via named pipes into `tmux` sessions: `webapi`, `broker`
- State machines are visualized dynamically using Graphviz + Flask + SSE
- Frontend YAML-based visual editor uses `react-flow` with Dagre layout and can export `.yaml` files


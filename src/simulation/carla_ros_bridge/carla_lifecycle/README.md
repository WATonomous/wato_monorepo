# carla_lifecycle

Lifecycle management for CARLA ROS 2 bridge nodes.

## Overview

Provides a lifecycle manager that coordinates the startup and restart of all CARLA bridge nodes in response to scenario changes.

## Nodes

### lifecycle_manager

Manages lifecycle transitions for all CARLA nodes. Waits for the scenario server to connect to CARLA before bringing up other nodes.

**Subscriptions:**
- `/<scenario_server>/scenario_status` (carla_msgs/ScenarioStatus)

**Parameters:** See `carla_bringup/config/carla_bridge.yaml`

## Behavior

1. Retries bringing up `scenario_server` until CARLA is available
2. When scenario loads, configures and activates all managed nodes
3. When scenario changes, restarts all managed nodes (deactivate → cleanup → configure → activate)

## Usage

Launched automatically via:

```bash
ros2 launch carla_bringup carla_bridge.launch.yaml
```

# carla_control

Vehicle control nodes for CARLA simulation.

## Overview

Provides nodes for controlling CARLA vehicles using standard ROS 2 message types.

## Nodes

### ackermann_control

Subscribes to `ackermann_msgs/AckermannDriveStamped` messages and applies control to the ego vehicle using CARLA's native Ackermann control interface.

**Subscriptions:**
- `~/command` (ackermann_msgs/AckermannDriveStamped)

**Parameters:** See `carla_bringup/config/carla_bridge.yaml`

## Usage

Launched automatically via:

```bash
ros2 launch carla_bringup carla_bridge.launch.yaml
```

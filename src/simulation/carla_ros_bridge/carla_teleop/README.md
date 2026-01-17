# carla_teleop

Teleoperation control for CARLA vehicles.

## Overview

Provides teleoperation via `geometry_msgs/Twist` messages, compatible with Foxglove teleop panel and other standard ROS teleop interfaces.

## Nodes

### teleop

Subscribes to Twist messages and applies control to the ego vehicle.

**Subscriptions:**
- `~/cmd_vel` (geometry_msgs/Twist)

**Services:**
- `~/set_autonomy` (std_srvs/SetBool) - Enable/disable CARLA autopilot

**Parameters:** See `carla_bringup/config/carla_bridge.yaml`

## Control Mapping

- `linear.x` → throttle/brake (scaled by `max_speed`)
- `angular.z` → steering (scaled by `max_steering`)

## Usage

Launched automatically via:
```bash
ros2 launch carla_bringup carla_bridge.launch.yaml
```

### Enable Autonomy (CARLA Autopilot)

```bash
ros2 service call /carla_teleop/set_autonomy std_srvs/srv/SetBool "{data: true}"
```

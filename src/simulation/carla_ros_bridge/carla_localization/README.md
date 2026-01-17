# carla_localization

Ground truth localization from CARLA simulation.

## Overview

Publishes TF transforms from CARLA ground truth pose data, providing the ego vehicle's position in the world.

## Nodes

### localization

Publishes the TF tree: `map` → `odom` → `base_link`

**Publications:**
- `/tf` (tf2_msgs/TFMessage)

**Parameters:** See `carla_bringup/config/carla_bridge.yaml`

## Coordinate System

CARLA uses a left-handed coordinate system. This node converts to ROS right-handed coordinates:
- X: forward (unchanged)
- Y: flipped (CARLA right → ROS left)
- Yaw/Pitch: flipped signs

## Usage

Launched automatically via:
```bash
ros2 launch carla_bringup carla_bridge.launch.yaml
```

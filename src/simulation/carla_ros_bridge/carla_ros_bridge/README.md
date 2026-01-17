# carla_ros_bridge

Metapackage for the CARLA ROS 2 bridge.

## Overview

This is a metapackage that depends on all CARLA ROS 2 bridge packages. Installing this package installs the complete bridge.

## Packages

- **carla_bringup** - Launch files and configuration
- **carla_control** - Vehicle control nodes
- **carla_lifecycle** - Lifecycle management
- **carla_localization** - Ground truth TF publishing
- **carla_msgs** - Custom messages and services
- **carla_perception** - Sensor publishers (camera, lidar, bbox)
- **carla_pygame** - Web-based map visualization
- **carla_sample_description** - Sample robot URDF
- **carla_scenarios** - Scenario management
- **carla_teleop** - Teleoperation control

## Quick Start

```bash
ros2 launch carla_bringup carla_bridge.launch.yaml
```

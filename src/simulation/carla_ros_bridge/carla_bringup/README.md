# carla_bringup

Launch files and configuration for the CARLA ROS 2 bridge.

This package is the main entry point for launching the complete CARLA bridge stack. The launch file starts all nodes in the correct order: first the `scenario_server` and `lifecycle_manager`, then the robot description, and finally all sensor and control nodes as managed lifecycle nodes.

All node parameters are centralized in `config/carla_bridge.yaml`, making it easy to configure the entire system from one file. Individual packages can still be run standalone for development using `ros2 run`.

## Usage

```bash
ros2 launch carla_bringup carla_bridge.launch.yaml
```

Launch arguments: `ros2 launch carla_bringup carla_bridge.launch.yaml --show-args`

## Files

- `config/carla_bridge.yaml` - Centralized configuration for all nodes
- `config/qos_overrides.yaml` - Example QoS settings for publishers/subscribers
- `launch/carla_bridge.launch.yaml` - Main launch file
- `launch/carla_bridge_qos.launch.yaml` - Example launch with QoS overrides

## QoS Configuration

ROS 2 supports QoS overrides via YAML files. To customize QoS settings for sensor topics:

```bash
# Use the example launch file with QoS overrides
ros2 launch carla_bringup carla_bridge_qos.launch.yaml
```

Edit `config/qos_overrides.yaml` to adjust reliability, durability, and history settings. This is useful for tuning performance (best_effort for high-frequency sensors) or ensuring delivery (reliable for control commands).

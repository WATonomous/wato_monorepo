# CARLA ROS Bridge

ROS2 Client-side Bridge to CARLA 0.10.0.

This bridge connects CARLA to ROS 2, providing sensor data, vehicle control, and simulation management through standard ROS interfaces. All nodes are lifecycle-managed for clean startup/shutdown during scenario changes.

## Motivation

This bridge is compatible with [CARLA 0.10.0 (Unreal Engine 5)](https://carla-ue5.readthedocs.io/en/latest/) and extends CARLA's ROS capabilities with features that ROS developers commonly expect:

- **Middleware agnostic** - Works with any ROS 2 middleware (CycloneDDS, Zenoh, etc.) using standard ROS interfaces
- **Lifecycle management** - All nodes implement ROS 2 lifecycle for coordinated startup, shutdown, and scenario transitions
- **Native TF integration** - Sensor transforms defined via standard URDF/robot_state_publisher, not hardcoded
- **Modular architecture** - Each function (perception, control, localization) is a separate package that can be used independently
- **GPU flexibility** - Given that newer version os CARLA required >15GB VRAM, this bridge supports both GPU and no-GPU rendering modes
- **Configurable synchronicity** - Run CARLA in synchronous or asynchronous mode
- **Runtime scenario switching** - Swap maps, weather, and traffic scenarios without restarting the bridge

## Getting Started

### Starting the CARLA Server

Start the CARLA server before launching the bridge (we suggest docker, but check our the latest [CARLA UE5 documentation](https://carla-ue5.readthedocs.io/en/latest/) for this):

```bash
# With GPU rendering
docker run \
    --runtime=nvidia \
    --net=host \
    --env=NVIDIA_VISIBLE_DEVICES=all \
    --env=NVIDIA_DRIVER_CAPABILITIES=all \
    carlasim/carla:0.10.0 bash CarlaUnreal.sh -RenderOffScreen -nosound


# Without GPU rendering
docker run \
    --net=host \
    carlasim/carla:0.10.0 bash CarlaUnreal.sh -nullrhi -nosound
```

The `-nullrhi` flag disables GPU rendering, useful for running on headless servers or when GPU resources are limited to non-existent. See the [CARLA UE5 documentation](https://carla-ue5.readthedocs.io/en/latest/) for more server options.

### Launching the Bridge

```bash
ros2 launch carla_bringup carla_bridge.launch.yaml
```

See [carla_bringup](carla_bringup/README.md) for configuration options.

## Packages

| Package | Description |
|---------|-------------|
| [carla_bringup](carla_bringup/README.md) | Launch files and centralized configuration |
| [carla_scenarios](carla_scenarios/README.md) | Scenario server and built-in scenarios (traffic, weather, ego spawn) |
| [carla_lifecycle](carla_lifecycle/README.md) | Lifecycle manager for coordinated node startup/shutdown |
| [carla_perception](carla_perception/README.md) | Sensor publishers (camera, LiDAR, bounding boxes) |
| [carla_localization](carla_localization/README.md) | Ground truth TF from CARLA (map → odom → base_link) |
| [carla_control](carla_control/README.md) | Ackermann vehicle control |
| [carla_teleop](carla_teleop/README.md) | Twist-based teleoperation with autopilot toggle |
| [carla_pygame](carla_pygame/README.md) | Web-based bird's-eye map visualization |
| [carla_sample_description](carla_sample_description/README.md) | Sample URDF for sensor frame definitions |
| [carla_msgs](carla_msgs/README.md) | Custom message and service definitions |
| [carla_common](carla_common/README.md) | Shared Python utilities |

## Architecture

```
┌─────────────────┐     subscribes to      ┌──────────────────┐
│ scenario_server │◄─────────────────────── │ lifecycle_manager │
│  (loads maps,   │     scenario_status    │  (coordinates     │
│   spawns ego,   │                        │   node lifecycle) │
│   ticks world)  │                        └────────┬─────────┘
└────────┬────────┘                                 │ manages
         │                                          ▼
         │ CARLA                          ┌─────────────────────┐
         │ connection                     │   Managed Nodes     │
         ▼                                │  ┌───────────────┐  │
    ┌─────────┐                           │  │ localization  │  │
    │  CARLA  │◄──────────────────────────│  │ perception    │  │
    │ Server  │    sensors, control       │  │ control       │  │
    └─────────┘                           │  │ teleop        │  │
                                          │  └───────────────┘  │
                                          └─────────────────────┘
```

The `scenario_server` owns the CARLA connection and coordinates with `lifecycle_manager` during scenario switches. All other nodes connect to CARLA independently but are lifecycle-managed to ensure clean resource handling.

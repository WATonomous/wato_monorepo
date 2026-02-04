# Simulation Module

The simulation module provides a CARLA-based simulation environment integrated with the WATonomous monorepo. It enables testing of autonomous driving algorithms in a realistic virtual environment with sensors, traffic, and various weather/lighting conditions.

## Table of Contents

- [Quick Start](#quick-start)
- [Configuration](#configuration)
- [Workflow](#workflow)
- [Visualization](#visualization)
- [Architecture](#architecture)
- [Troubleshooting](#troubleshooting)

## Quick Start

### 1. Enable the Simulation Module

Add `simulation` to your active modules in `watod-config.local.sh`:

```bash
export ACTIVE_MODULES="simulation"
```

### 2. Configure GPU Mode (Optional)

By default, CARLA runs without GPU rendering (`no_gpu` mode), which automatically enables the pygame HUD for web-based visualization. For GPU-accelerated rendering:

```bash
export CARLA_RENDER_MODE="gpu"  # Requires NVIDIA GPU with >8GB VRAM
```

### 3. Start the Simulation

```bash
./watod up
```

This starts:
- **carla_sim** - The CARLA server (UE5-based simulator)
- **simulation_bringup** - All ROS2 bridge nodes (sensors, control, localization)

All bridge nodes start automatically and connect to CARLA. The simulation is ready when you see sensor data flowing in Foxglove.

## Configuration

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `CARLA_RENDER_MODE` | `no_gpu` | `gpu` for GPU rendering, `no_gpu` for headless |
| `PYGAME_HUD_ENABLED` | `true` | Auto-set to `true` when `CARLA_RENDER_MODE=no_gpu` |
| `PYGAME_HUD_PORT` | Auto | Port for pygame web HUD (auto-assigned based on user ID) |

### Simulation Parameters

Simulation parameters are configured in `src/simulation/simulation_bringup/config/carla_bridge.yaml`. Key settings include:

- **Timing**: FPS, synchronous mode, physics substepping
- **Sensors**: Camera resolution, LiDAR configuration, detection range
- **Control**: Throttle/steering scaling, command timeout
- **Scenarios**: Initial scenario, traffic density

## Workflow

### Basic Usage

Once `watod up` completes, the simulation is running with the default scenario (light traffic). Use ROS2 services to interact with the simulation:

#### Switch Scenarios

```bash
# List available scenarios
ros2 service call /carla/scenario_server/get_available_scenarios carla_msgs/srv/GetAvailableScenarios

# Switch to a different scenario
ros2 service call /carla/scenario_server/switch_scenario carla_msgs/srv/SwitchScenario \
  "{scenario_name: 'carla_scenarios.scenarios.heavy_traffic_scenario'}"
```

Built-in scenarios:
- `empty_scenario` - Ego vehicle only, no NPCs
- `light_traffic_scenario` - Ego + 5 vehicles + 10 pedestrians (default)
- `default_scenario` - Ego + 20 vehicles + 40 pedestrians
- `heavy_traffic_scenario` - Ego + 50 vehicles + 100 pedestrians

#### Enable Autonomy Mode

Toggle CARLA's built-in autopilot to let the ego vehicle drive itself:

```bash
# Enable autopilot
ros2 service call /carla/carla_teleop/set_autonomy std_srvs/srv/SetBool "{data: true}"

# Disable autopilot (return to manual/teleop control)
ros2 service call /carla/carla_teleop/set_autonomy std_srvs/srv/SetBool "{data: false}"
```

#### Manual Control

When autonomy is disabled, control the vehicle via the `/carla/cmd_vel` topic:

```bash
# Using teleop_twist_keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/carla/cmd_vel
```

Or use the Foxglove teleop panel connected to `/carla/cmd_vel`.

### Integration with Other Modules

To test perception, planning, or control algorithms:

```bash
export ACTIVE_MODULES="simulation perception world_modeling action"
./watod up
```

The simulation provides:
- `/carla/top_lidar/points` - PointCloud2 from simulated LiDAR
- `/carla/detections_3d` - Ground truth 3D bounding boxes
- TF tree - Ground truth pose (map → odom → base_link)

Your modules can subscribe to these topics and publish control commands to test the full autonomy stack.

## Visualization

### Foxglove Studio (Recommended)

1. Ensure `infrastructure` is in your active modules
2. Connect Foxglove to `ws://localhost:<FOXGLOVE_BRIDGE_PORT>`
3. Add panels for:
   - 3D view with `/carla/top_lidar/points` and `/carla/bbox_markers`
   - Image view with camera topics
   - Teleop panel publishing to `/carla/cmd_vel`

### Pygame HUD (No-GPU Mode)

When `CARLA_RENDER_MODE=no_gpu` (the default), a web-based bird's-eye view is automatically enabled:

1. Forward the `PYGAME_HUD_PORT` to your local machine
2. Open `http://localhost:<PYGAME_HUD_PORT>` in a browser

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        watod up                                  │
└─────────────────────────────────────────────────────────────────┘
                              │
              ┌───────────────┴───────────────┐
              ▼                               ▼
     ┌─────────────────┐            ┌─────────────────────┐
     │   carla_sim     │◄──────────►│ simulation_bringup  │
     │  (CARLA Server) │   TCP/IP   │   (ROS2 Bridge)     │
     │                 │            │                     │
     │  - Physics      │            │  - scenario_server  │
     │  - Rendering    │            │  - lifecycle_mgr    │
     │  - Traffic AI   │            │  - localization     │
     └─────────────────┘            │  - perception       │
                                    │  - control/teleop   │
                                    └─────────────────────┘
                                              │
                                              ▼
                                    ┌─────────────────────┐
                                    │    ROS2 Topics      │
                                    │  /clock             │
                                    │  /tf                │
                                    │  /carla/*/points    │
                                    │  /carla/detections  │
                                    │  /carla/cmd_vel     │
                                    └─────────────────────┘
```

For detailed bridge architecture, see [carla_ros_bridge/README.md](carla_ros_bridge/README.md).

## Troubleshooting

### CARLA server fails to start

**GPU mode**: Ensure NVIDIA drivers are installed and GPU has sufficient VRAM (>16GB recommended).

```bash
# Check GPU availability
nvidia-smi
```

**No GPU mode**: Should work on any machine. If issues persist, check Docker logs:

```bash
docker logs watod_<user>-carla_sim-1
```

### Bridge nodes not connecting

The bridge waits for CARLA to be ready. Check scenario_server logs:

```bash
docker logs watod_<user>-simulation_bringup-1
```

Common issues:
- CARLA server still starting (wait 30-60 seconds)
- Port conflict on 2000 (CARLA's default port)
- Network mode issues (both containers should use `host` network)

### Low FPS / Slow simulation

This is expected behavior. The ROS bridge runs CARLA in synchronous mode, which prioritizes determinism over real-time performance. The simulation maintains accurate physics regardless of render FPS.

To improve performance:
- Use `no_gpu` mode (headless rendering)
- Reduce sensor resolution in config
- Disable unused sensors
- Use `no_rendering_mode: true` in scenario_server config

### Sensors not publishing

Check that the lifecycle manager has activated all nodes:

```bash
ros2 lifecycle list /carla/lidar_publisher
```

All nodes should be in the `active` state. If stuck in `inactive`, check the scenario_server connection to CARLA.

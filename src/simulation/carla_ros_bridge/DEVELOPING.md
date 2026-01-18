# Developing CARLA ROS Bridge

## Requirements

The bridge was designed to meet these requirements:

1. **Scenario switching without restart** - Users should be able to switch maps, traffic density, and ego spawn without restarting the ROS stack
2. **Standard ROS interfaces** - Use standard message types (Twist, TF, PointCloud2) for compatibility with existing ROS tools
3. **Configurable sensors** - Sensor types, positions, and parameters should be configurable without code changes
4. **Clean resource management** - No dangling CARLA sensors or connections when nodes stop
5. **Simulation time** - Support `use_sim_time` for deterministic replay and testing

## Key Design Decisions

### Lifecycle Nodes Everywhere

All CARLA-connected nodes are ROS 2 lifecycle nodes. This enables:
- Coordinated startup (wait for CARLA before activating sensors)
- Clean shutdown (destroy sensors before scenario unloads)
- Recovery from scenario switches without full restart

The `lifecycle_manager` orchestrates transitions based on `scenario_server` status.

### Centralized World Ticking

Only `scenario_server` calls `world.tick()` in synchronous mode. Other nodes read from CARLA on their own timers but don't control simulation advancement. This prevents race conditions and ensures consistent world state.

### Sensor Positions from TF

Sensor nodes look up their spawn positions from TF (provided by URDF via `robot_state_publisher`). This keeps sensor configuration in one place (the URDF) rather than duplicating positions in ROS parameters and CARLA spawn calls.

### Per-Node CARLA Connections

Each node connects to CARLA independently rather than sharing a connection. This:
- Simplifies node lifecycle (each manages its own connection)
- Allows nodes to run on different machines
- Avoids single point of failure

The tradeoff is multiple TCP connections to CARLA, but this hasn't been a bottleneck.

### Scenarios as Loadable Modules

Scenarios are Python classes loaded by module path, not hardcoded in the server. Users can define custom scenarios in their own packages and load them by path (e.g., `my_package.scenarios.my_scenario`).

## Package Structure

```
carla_ros_bridge/
├── carla_bringup/        # Launch + config (entry point for users)
├── carla_scenarios/      # World state management
├── carla_lifecycle/      # Node coordination
├── carla_perception/     # Sensors → ROS messages
├── carla_localization/   # Ground truth TF
├── carla_control/        # ROS commands → CARLA vehicle
├── carla_teleop/         # Twist interface for manual control
├── carla_pygame/         # Debug visualization
├── carla_sample_description/  # Example URDF
├── carla_msgs/           # Custom messages/services
└── carla_common/         # Shared utilities (no ROS deps)
```

## Common Patterns

### Node Lifecycle

```
on_configure:  Connect to CARLA, find ego vehicle, create publishers
on_activate:   Start timers, spawn sensors, register callbacks
on_deactivate: Stop timers, destroy sensors, brake vehicle
on_cleanup:    Destroy publishers, release CARLA connection
```

### Ego Vehicle Discovery

Nodes find the ego vehicle by `role_name` attribute. The scenario is responsible for spawning the ego with the correct role_name (default: `ego_vehicle`).

### Coordinate Conversion

CARLA uses left-handed coordinates (Y-right), ROS uses right-handed (Y-left). Negate Y for positions and pitch/yaw for rotations. The `carla_common` package provides conversion utilities.

### Command Timeout

Control nodes implement a safety timeout. If no command arrives within `command_timeout`, the vehicle brakes. This prevents runaway vehicles when the control source disconnects.

## Adding a New Node

1. Create a lifecycle node in the appropriate package
2. Follow the configure/activate/deactivate/cleanup pattern
3. Add entry point in `setup.cfg`
4. Add to `carla_bringup/config/carla_bridge.yaml`
5. If it needs CARLA resources, add to `lifecycle_manager.node_names`

## Testing Changes

```bash
# Build
colcon build --packages-select <package_name>

# Run single node
ros2 run <package_name> <node_name>

# Run full stack
ros2 launch carla_bringup carla_bridge.launch.yaml
```

Nodes can be tested individually with `ros2 run` as long as CARLA is running and a scenario is loaded.

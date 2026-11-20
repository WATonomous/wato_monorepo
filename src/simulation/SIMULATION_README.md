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
| `WATCLOUD_MODE` | `false` | Set to `true` in `watod-config.local.sh` if running on WatCloud |

### Simulation Parameters

Simulation parameters are configured in `src/simulation/simulation_bringup/config/carla_bridge.yaml`. Key settings include:

- **Timing**: FPS, synchronous mode, physics substepping
- **Sensors**: Camera resolution, LiDAR configuration, detection range
- **Control**: Throttle/steering scaling, command timeout
- **Scenarios**: Initial scenario, traffic density

## Workflow

### Basic Usage

Once `watod up` completes, the simulation is running with the default scenario (light traffic).

You can control the simulation using the commands configured in the Foxglove layout (see [Visualization](#visualization)).

Alternatively, to run commands manually in the container:

```bash
./watod -t simulation_bringup
source /opt/watonomous/setup.bash
```

Then you can issue ROS2 service calls:

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
3. Import the layout from `config/foxglove_config/carla_sim.json` by clicking the Layout dropdown → Import from file → Select `carla_sim.json`
4. Use the layout buttons to run commands and visualize the simulation.

### Pygame HUD (No-GPU Mode)

When `CARLA_RENDER_MODE=no_gpu` (the default), a web-based bird's-eye view is automatically enabled:

1. Forward the `PYGAME_HUD_PORT` to your local machine
2. Open `http://localhost:<PYGAME_HUD_PORT>` in a browser

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        watod up                                 │
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

## WATOD simulation v1 integration contract

The session reuses the existing simulation, action and world modeling images; it
has one experiment at a time. `watod sim` owns only containers it starts. The
existing scenario server is the sole CARLA tick and `/clock` authority. Manual
launch paths retain their defaults. Live acceptance is performed by the operator.

- States: IDLE -> PREPARING -> READY -> RUNNING <-> PAUSED -> FINISHED.
  `close` returns to IDLE; `reset` creates a new attempt and returns to READY.
  Open/test require setup and IDLE. Invalid transitions name the state and recovery.
- `/sim/open` uses `carla_msgs/srv/SwitchScenario`, with `scenario/NAME` or
  `playground/NAME` in `scenario_name`. World preparation precedes stack startup;
  the host then calls `/sim/ready` (`std_srvs/srv/Trigger`).
- `/sim/{play,pause,resume,stop,reset,close,status,ready}` use
  `std_srvs/srv/Trigger`. `success=false` means the operation failed;
  status returns JSON in `message`. `/sim/status` topic is a transient-local
  `std_msgs/msg/String` snapshot with a wall-clock heartbeat, including while paused.
- World services: `/carla/scenario_server/prepare_experiment` uses
  `SwitchScenario` with an installed playground name; `/carla/pause` uses
  `SetBool`; `/carla/scenario_server/{hold,release,close_experiment}` use Trigger.
  Preparation selects the reviewed YAML world definition, holds the ego during
  readiness ticks and resets seeds. Pause settles the in-flight tick before reply.
- Scenario definitions live in `carla_scenarios/config/scenarios`, playgrounds in
  `config/playgrounds`. Version 1 YAML has `id`, `playground`, `stack`, `events`
  (an ordered sequence of `when` / `do` mappings), `deadlines`, `completion` and
  `thresholds`. Events use `play` or `{elapsed_s: N}` triggers and reviewed action
  names. Simulation time drives events/scoring; monotonic time bounds readiness
  and stalls. Events are consumed once per attempt.
- `controller_mpc` owns one fake planner and one MPC; `autonomy` owns real
  behavior, lattice, trajectory planning and MPC. Only the CARLA Ackermann node
  actuates during runner experiments; teleop is excluded. Fake planner services
  are `/action/fake_planner/{start_trajectory,stop_trajectory,reset}` and its
  transient-local String state is `/action/fake_planner/status` (`ready`,
  `running`, `finished`, `stopped`). Reference Path is `/sim/reference_path`.
- Common inputs: `/ego/odom` (map frame), `/carla/detections_3d`,
  `/world_modeling/costmap`, `/world_modeling/lanelet/route_ahead`;
  `/goal_point` is geometry_msgs/PointStamped. Actual Path is `/sim/actual_path`.
- Attempt IDs combine UTC time and a UUID. Results under the session result root
  contain `configuration.yaml`, `events.json`, and `result.json` with run ID,
  UTC timestamps, simulated duration, outcome, reasons and metrics with units.
  Outcomes/CLI test exit codes: pass/0, assertion_failure/1, data_error/2,
  interrupted/3; usage or session errors use 2. Missing evidence cannot pass.
- Reset discards old samples/events, resets world actors and lifecycle state,
  resets the fake planner anchor and preserves prior results. Evaluation reloads
  the world within the same CARLA process. Thresholds are provisional until the
  operator records a measured baseline and completes live acceptance.

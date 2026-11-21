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
- `controller_pure_pursuit` owns one fake planner and one adaptive pure pursuit
  controller; `autonomy` owns real behavior, lattice, trajectory planning and
  adaptive pure pursuit. Only the CARLA Ackermann node actuates during runner
  experiments; teleop is excluded. Fake planner services
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

## Runner v1 operator guide

Use prepared module images containing this branch. `watod sim` never builds or
pulls images. The action image discovers `fake_planner` through its existing
`COPY src/action` and colcon build. The simulation image additionally installs
message definitions needed to inspect the real stack. No runner Dockerfile is
needed. Existing `watod up` and manual ROS launch commands remain available.

From the repository root, the installed `watod` symlink and `./watod` support the
same commands:

```bash
./watod sim list
./watod sim setup
./watod sim open playground oval
./watod sim close
./watod sim open scenario controller_snake
./watod sim play
./watod sim pause
./watod sim resume
./watod sim status
./watod sim reset
./watod sim play
./watod sim close
./watod sim test parked_car_straight
./watod sim stop
```

`setup` starts an idle session and prints the configured Foxglove WebSocket
address. Repeating it preserves a healthy session, including an open experiment.
`open` requires setup and an idle session. `open scenario` waits for active stack
nodes, fresh odometry and the reference preview or accepted route and obstacle in
the planner costmap. Setup ticks hold the ego stationary and are excluded from
scoring. `play` releases the ego. `close` releases the experiment; `stop` removes
only recorded session containers and preserves maps, caches and results. The
playground path lets developers launch their own ROS modules before `play`.

Import [the runner layout](../../config/foxglove_config/sim_runner.json) into
Foxglove and connect to the address printed by setup. The Play, Pause, Resume,
Stop run, Reset and Status service controls use the same `/sim/*` handlers as the
CLI. **Stop run** saves an interrupted attempt and leaves CARLA available;
`watod sim stop` tears down the session. The layout displays the reference,
driven path, detections, odometry and state. Set the 3D panel's fixed frame to
`map`. Paused time freezes physics, event dispatch and scoring; the status
heartbeat continues on wall time.

Maps must be readable inside both consuming images. These presets use
`maps/xodr/oval.xodr`; autonomy also uses `maps/osm/oval.osm` and the current oval
map alignment overlay. Convert the source with the existing
`watod_scripts/tools/osm_to_xodr/osm_to_xodr.sh` workflow before setup/open. The
converter records source and output SHA256 digests, its pinned revision and
settings in `oval.xodr.metadata.json`. Unchanged inputs/settings reuse the
conversion; changed or unrecorded output requires an explicit `--force`.
World preflight checks available provenance before replacing the world.
The parked-car preset requires this provenance, including the matching OSM source.

The snake selects the shipped `steady_state_slalom` JSON with one fake planner
and one adaptive pure pursuit controller. The world chooses a deterministic
straight lane with at least 100 m available and records its position, heading
and geometry. The parked car starts 40 m ahead, with a goal 80 m ahead on that
lane. Its preset uses the real behavior, lattice planner, trajectory planner and
adaptive pure pursuit, CARLA ground-truth localization and tracked detections
feeding prediction, enriched world objects and the costmap. Overtaking is
disabled; the runner never issues a stop command
to produce a passing score. The autonomy preset caps speed at 3 m/s and uses a
2 m planner stop margin.

All metric thresholds are provisional until measured on the team's host. The
parked test requires at least 15 m forward progress, bumper clearance from 2 to
8 m, speed at most 0.15 m/s and 3 continuous simulated seconds stopped in that
window. A stationary ego at spawn cannot pass. Missing detections, absent
obstacle occupancy in the planner input, collision, passing or timeout prevent
success. The snake requires explicit planner completion, sufficient driven
progress and samples, lateral RMSE at most 0.6 m and maximum error at most 1.5 m.
The manifests record deadlines, thresholds and seeds; result configuration
records applied world settings and vehicle footprints.

Results live in `.cache/watod-sim/results/<UTC>_<unique-id>/`:

- `configuration.yaml`: resolved definition, world geometry/settings and revision.
- `events.json`: ordered delivery records and timestamps.
- `result.json`: outcome, failure reasons and metrics with units.

The adjacent `session.json` records owned container IDs and runtime configuration;
`compose.json` captures effective Compose settings. Never edit ownership records
while a session is running. Fix a named missing dependency, then `close` and
reopen, or `reset` a prepared experiment. A failed base setup cleans up its
partial containers. If session configuration changes, stop using the original
configuration before setting up a replacement. `WATOD_SIM_FOXGLOVE_HOST` changes
the printed host for remote/forwarded connections. Setup and readiness wall
limits can be adjusted with `WATOD_SIM_SETUP_TIMEOUT_S` and
`WATOD_SIM_READY_TIMEOUT_S`.

`watod sim test NAME` uses the interactive runner, saves its terminal result,
closes the experiment and keeps the session warm. Exit codes are 0 for pass,
1 for assertion failure, 2 for setup/data/usage error and 3 for interruption.
Required completion and observation evidence must be present to pass.

### Delivery evidence and pending live acceptance

Implementation baseline: main `6f921aadb506a993c767a6ce0665264d58b06693`.
Selected PR 487 head: `f19fbad28cba3eb9bb5c6053046831c3a33f84fa`
(Brian Zheng); only its fake planner package is integrated. Current main's oval
world and world-model alignment are retained. Configured CARLA server/client are
0.10.0, with a local CPython 3.12 wheel; the applied versions are checked on
connection. Physics is 60 Hz with ten substeps of up to 2 ms, covering the
16.67 ms world step. Runtime settings are saved per attempt.

Per the implementation request, container builds and runtime tests are left to
the operator. No simulated pass, timings, or baseline calibration is claimed.
The target map files are absent from this checkout, and no prepared ROS/CARLA
session was launched. Formatting, parsing and Compose rendering can establish
source consistency; compilation and live acceptance remain pending.

Completed static checks: Ruff 0.11.5 formatting/lint, clang-format 19.1.7,
cpplint, CMake lint, ShellCheck and Bash syntax, Python AST, YAML/JSON/XML
parsing, whitespace checks, and Compose rendering for Linux host and WatCloud
networking. YAML lint retains existing document-start/comment-indent warnings.

On the team's prepared Linux host, record the revision, effective image IDs,
map provenance, thresholds and seed. Measure cold/cached setup, both opens and
reset separately from image preparation. Exercise missing setup, invalid play,
a missing map, delayed odometry and an unavailable service, then recover. Run
the snake twice through Foxglove controls, check that pause freezes actors and
commands, and verify old samples do not cross reset. Close and open the parked
car test while checking that the CARLA container/process ID stays the same.
Capture its stopped result and a no-progress or missing-obstacle failure.
The final review must include those results, timings and `git log` for this
branch before calling the Linux live acceptance complete.

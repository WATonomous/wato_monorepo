# Simulation Manager

Use `watod sim` to explore CARLA playgrounds or run scored driving scenarios.
A session keeps CARLA and the ROS bridge running while you open, reset, and close scenarios.

## Quick Start

Run these commands on the host, using your configured image tag and GPU mode:

```bash
# Download CARLA (the same image supports GPU and no-GPU modes).
docker pull carlasim/carla:0.10.0

# Build the ROS images; infrastructure is included automatically.
watod -m simulation -m action -m world_modeling build

# Start a session and prepare the controller scenario.
watod sim setup
watod sim open scenario controller_snake
watod sim play
```

Alternatively, set the default modules in `watod-config.local.sh`:

```bash
export ACTIVE_MODULES="simulation action world_modeling"
```

Then use `watod build` instead of the explicit build command above. Infrastructure
is included automatically; CARLA still needs its separate pull. This setting also
controls ordinary `watod up`; use `watod sim setup` to start a managed session.
Explicit `-m` options override the configured module list for that command.

`controller_snake` needs the simulation and action images. Autonomy scenarios
also need world-modeling. `setup` requires local images; it neither builds nor pulls them.
`brake_in_curve` and `sine_with_dwell` also use world-modeling for map visualization.
For a playground, the simulation and infrastructure images are enough.
On WatCloud (`WATCLOUD_MODE=true`), also run `docker pull busybox:latest` before setup.

Build and recreate the session once when installing executable changes, including
this data-loading support. Subsequent scenario and maneuver edits use the mounted files:

```bash
watod sim stop
watod sim setup
watod sim open scenario controller_snake
```

## Adding Scenarios

Add a scenario YAML under `carla_scenarios/config/scenarios/` and, if needed,
a JSON under `src/action/fake_planner/maneuvers/`. Managed sessions read these
files from the checkout through read-only mounts; data edits do not need image
rebuilds. Reopen the scenario to apply changes.

The YAML selects trajectory source, world modeling, actors, goal, completion,
and metric limits. The required modules are inferred from those choices. Omitting a trajectory
starts only the requested world capabilities. See the [authoring guide](../carla_ros_bridge/carla_scenarios/AUTHORING.md)
for a complete example and map-coordinate vehicle placement.

## Controller Scenarios

| Scenario | Maneuver | Map visualization |
|----------|----------|-------------------|
| `controller_snake` | Steady slalom | No world-modeling group |
| `brake_in_curve` | Slow through a left curve, then stop | Oval map |
| `sine_with_dwell` | Swerve, hold an offset, then return and stop | Oval map |

```bash
watod sim open scenario brake_in_curve
watod sim play
# Close before switching scenarios.
watod sim close
watod sim open scenario sine_with_dwell
watod sim play
```

The two new scenarios start the world model and map markers before the controller.
Reimport `sim_runner.json` to see `/world_modeling/lanelet/map_viz_markers` alongside
the reference trajectory. These are authored controller maneuvers, not road-following routes.

## Commands

`NAME` is a playground or scenario name from `watod sim list`.

| Command | Behavior |
|---------|----------|
| `watod sim list` | List playgrounds and scored scenarios from the checkout; Docker is not required |
| `watod sim setup` | Start/reuse the base session using prepared images and print Foxglove connection details |
| `watod sim open playground NAME` | Load map, ego, traffic, weather and bridge; ready for manual use without application nodes or scoring |
| `watod sim open scenario NAME` | Prepare the world and reviewed application stack; leave the attempt ready for play |
| `watod sim play` | Start a prepared scenario |
| `watod sim pause` | Pause the current world/execution |
| `watod sim resume` | Resume after a pause |
| `watod sim status` | Show session readiness, state and active attempt information |
| `watod sim reset` | Reload the current world; for scenarios, reset the stack and start a fresh attempt |
| `watod sim close` | Close the playground/scenario and its application stack; keep the base session available |
| `watod sim test NAME` | Open, run, evaluate, save and close a scenario; keep the base session available |
| `watod sim stop` | Stop the managed session and its application containers; keep saved results |

Close the current playground or scenario before opening another.
In a playground, start your application nodes yourself; scenarios start them for you.

## How the Stack Works

| Container | Purpose |
|-----------|---------|
| `carla_sim` / `carla_sim_no_gpu` | Runs CARLA physics, vehicles, and traffic |
| `sim_session` | Runs the ROS bridge and simulation manager |
| `sim_action` | Runs the controller and, for autonomy, the planners |
| `sim_world` | Runs world modeling for autonomy, or map visualization for controller tests |
| `infrastructure_bringup` | Provides Foxglove and shared ROS infrastructure |

Opening a scenario loads its world and starts the application nodes. `play` begins
the run; `reset` reloads the world and starts a fresh attempt. CARLA's scenario
server controls simulation time and `/clock`.

`sim_session` uses the image built by `simulation_bringup`. It is a separate
service from the standard `watod up` workflow; use one workflow at a time.
See the [simulation guide](../SIMULATION_README.md) for configuration and manual use.

## Results and Diagnostics

- Results: `.cache/watod-sim/results/<run-id>/` contains configuration, events, and scores.
- Logs: `docker logs --follow <container>`; use `sim_session` for the bridge and `sim_action` or `sim_world` for application logs.
- Foxglove: connect to the address printed by `setup` and import [`sim_runner.json`](../../../config/foxglove_config/sim_runner.json).
- Scenario definitions, maps, and scoring: see the [authoring guide](../carla_ros_bridge/carla_scenarios/AUTHORING.md).
- Runtime code and topic naming: see [`simulation_runtime`](../simulation_runtime/README.md).

## Shutdown

```bash
watod sim stop
```

This stops the managed session and keeps results. If you also started regular
modules with `watod up`, stop those with `watod down all`.

## Session Recovery

If a session stops responding or a SLURM job expires, run `watod sim stop`,
then `watod sim setup` in a valid allocation. Reopen the scenario afterward.
Stop can recover verified leftover containers when the session record is missing.
Do not delete `.cache/watod-sim` to restart: it also contains saved results.

# Fake planner

Loads a maneuver JSON and replays its trajectory for controller testing.
It uses Brian Zheng's fake planner from #487.

The lifecycle node configures the path, activates its publishers, and waits for
release. Start, stop, and reset work without reloading the file.

## Usage

Start the simulation or vehicle environment first. Choose a spawn with enough
room for the maneuver. The existing simulation maps are documented in the
[manual simulation guide](../../simulation/SIMULATION_README.md).

```bash
# CARLA
ros2 launch fake_planner fake_planner_sim.launch.yaml

# Vehicle
ros2 launch fake_planner fake_planner_vehicle.launch.yaml
```

Both wrappers wait for the start service. The vehicle wrapper keeps its deployed
controller tuning and disables automatic reset after an odometry jump.

```bash
ros2 service call /action/fake_planner/fake_planner_node/start_trajectory std_srvs/srv/Trigger
ros2 service call /action/fake_planner/fake_planner_node/stop_trajectory std_srvs/srv/Trigger
ros2 service call /action/fake_planner/fake_planner_node/reset std_srvs/srv/Trigger
```

Reset clears completion state. Relative maneuvers anchor at the next odometry
pose. Absolute maneuvers keep their authored geometry and restart path matching.
The vehicle must return to the start to repeat an absolute open maneuver.

## Launch arguments

| Argument | Purpose |
|----------|---------|
| `maneuver` | Select `steady_state_slalom`, `sine_with_dwell`, or `brake_in_curve` |
| `maneuver_file` | Load another JSON file |
| `controller` | Manual shared/vehicle rig: `pure_pursuit` or `mpc` |
| `pure_pursuit_config` | Controller tuning file |
| `anchoring` | `auto`, `relative`, or `absolute` |
| `start_on_activate` | Release immediately when true; wrappers default to false |
| `odom_topic` | Vehicle pose and velocity input |
| `trajectory_topic` | Controller trajectory output |
| `behaviour_topic` | Controller engagement heartbeat |
| `marker_topic` | Trajectory preview markers |

Topic names belong in launch arguments and ROS remappings. `config/params.yaml`
contains behavior and geometry settings.

## Maneuver JSON

JSON contains an ordered `segments` list. Supported segments are `straight`,
`dwell`, `shift`, `slalom`, and `arc`. The helper samples these into a path once
when the node configures.

An optional `start` pose gives absolute coordinates. Otherwise `anchoring:=auto`
uses the first odometry pose. Open paths brake into a stop pad. Closed paths
wrap around for repeated laps.

The controller receives a rolling window around the vehicle. A release ramp
limits initial speed. A completed open maneuver stops publishing commands;
preview markers remain visible.

See [DEVELOPING.md](DEVELOPING.md) for the code structure and test commands.

## Managed simulation

Automated scenarios use adaptive pure pursuit. The simulation wrapper selects
`action_sim.yaml` and publishes to `/action/trajectory_planning/trajectory`.

The original private services above remain available. The manager uses the
aliases `/action/fake_planner/{start_trajectory,stop_trajectory,reset}`.

`/action/fake_planner/status` publishes `ready`, `running`, `stopped`, or `finished`.
Ready requires fresh odometry. Finished requires driven completion and a stop.
`/sim/reference_path` publishes the full anchored path for preview and scoring.
Both snapshots are reliable and transient local.

The controller snake fixture requires 82 metres of straight runout. Its spatial
launch ramp is disabled; the ROS-time release ramp handles startup.

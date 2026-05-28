# sil_testing

Software-In-the-Loop (SIL) testing for autonomy components. It drives the existing CARLA
simulation in a closed loop to test **one component at a time** (currently motion control
/ pure pursuit), records a fixed-duration MCAP for playback, and checks the run against
scenario-defined limits.

This package (the SIL framework) lives in the simulation module and runs in the
**simulation image**. The component under test is its own module's binary and runs from
**its own image** (e.g. `pure_pursuit` from the action image), launched by `watod sil run`.
The two sides communicate over ROS.

## How it works

A single scenario YAML drives the whole run. The `yaml_scenario` plugin (in
`carla_scenarios`) reads `map` / `ego` / `actors` / `weather` / `seed` to build the CARLA
world. This package reads `route` / `limits` / `duration_s` to feed the controller-under-
test and decide pass/fail.

```
scenario.yaml ─► yaml_scenario (CARLA)  ─ spawns ego + actors, seeds TM, sync /clock
                       │
            carla_localization ─ TF (map→odom→base_footprint) + /ego/odom
                       │
scenario route ─► trajectory_feeder ─ wato_trajectory_msgs/Trajectory ─►┐   [simulation image]
                                                                          │
                          CONTROLLER UNDER TEST (e.g. pure_pursuit)           [action image]
                                                                          │
                              AckermannDriveStamped → /carla/ackermann_control/command
                                                                          │
                          carla_control.ackermann_control ─ apply to CARLA ego (loop closes)

  sil_monitor      ─ live limit diagnostics (/sil/diagnostics)          [simulation image]
  ros2 bag (mcap)  ─ records the run for Foxglove playback
  scenario_runner  ─ ends the run after duration_s of sim time
  sil_analyzer     ─ post-hoc: metrics vs limits → report.json + exit code (authoritative)
```

The component under test is the **real node**; only its upstream input (the reference
trajectory) is synthesized, replacing the planner.

## Scenario file

Poses are in the **ROS map frame** (x forward, y left, z up, yaw degrees CCW). Limit keys
must start with `max_` (upper bound) or `min_` (lower bound).

```yaml
name: straight_follow
component: motion_control
map: Town10HD
duration_s: 30.0
seed: 42
weather: ClearNoon
ego:
  transform: {x: 100.0, y: 8.0, z: 0.3, yaw: 0.0}   # or: spawn_point_index: 5
  blueprint: vehicle.mini.cooper
route:
  target_speed_mps: 8.0
  waypoints: [{x: 100, y: 8}, {x: 140, y: 8}, {x: 180, y: 8}]
actors:
  - type: vehicle
    transform: {x: 120.0, y: 11.5, z: 0.3, yaw: 0.0}
    behavior: {kind: constant_velocity, speed_mps: 6.0}   # static | autopilot | constant_velocity
limits:
  max_lateral_accel_mps2: 4.0
  max_steering_rate_radps: 1.0
  max_cross_track_error_m: 1.5
  max_speed_mps: 10.0
  min_distance_to_actor_m: 2.0
```

Supported limits today: `max_lateral_accel_mps2`, `max_steering_rate_radps`,
`max_cross_track_error_m`, `max_speed_mps`, `min_distance_to_actor_m`. Extend
`sil_testing/sil_metrics.py` + `sil_analyzer._build_series` to add more.

## Running

A CARLA server must be running (e.g. `watod up` with `simulation` active). Then:

```bash
watod sil run src/simulation/sil_testing/config/scenarios/straight_follow.yaml
```

The driver starts the sim side in the simulation image (CARLA bridge via
`motion_control_sil.launch.py`, which includes `simulation_bringup/sil_simulation.launch.yaml`,
plus the reference feeder, monitor, and recorder), starts the controller under test from
the action image, records to `sil_runs/<name>_<ts>/bag`, then prints a pass/fail table and
writes `report.json`. It exits non-zero if any limit was breached (CI-friendly).

Play the bag back in Foxglove using the layout at `config/foxglove/sil.json`.

## Tests

```bash
watod test simulation sil_testing
```

Unit tests cover the scenario schema (`test_sil_schema.py`) and the metric/limit logic
(`test_sil_metrics.py`); both are pure Python.

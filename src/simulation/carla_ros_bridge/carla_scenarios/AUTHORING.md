# Authoring playgrounds and scenarios

Add YAML and maneuver JSON files to create new scenarios using the existing
trajectory, world, completion, and metric capabilities. No Python registration is needed.
See the [manager commands](../../simulation_manager/README.md#commands) to run them.

## Choose a starting point

| Definition | Purpose |
|------------|---------|
| `controller_snake` | Track the shipped slalom with the controller |
| `brake_in_curve`, `sine_with_dwell` | Track other maneuvers with map visualization |
| `parked_car_straight` | Run autonomy toward a goal beyond a parked vehicle |
| `oval`, `ring_road` | Playgrounds using converted OpenDRIVE maps |
| `empty`, `custom`, `town10hd_default`, `town10hd_light`, `town10hd_heavy` | Town10HD playgrounds |

Use catalog IDs such as `oval`. Python module paths and old scenario aliases
are not accepted. The filename stem must match `id`; names use
`[a-z][a-z0-9_]*`.

## Data directories

Managed sessions mount these directories read-only into the containers that need them:

| Data | Default host directory |
|------|------------------------|
| Playground and scenario YAML | `src/simulation/carla_ros_bridge/carla_scenarios/config/` |
| Maneuver JSON | `src/action/fake_planner/maneuvers/` |
| ROS map configuration | `src/world_modeling/world_modeling_bringup/config/map_configs/` |
| OSM and OpenDRIVE maps | `maps/` |

After the initial build and session recreation for this feature, adding or editing
a definition, maneuver, or map asset does not require an image rebuild. Open the
scenario again to apply edits. Reset also reloads the data when the required
container roles stay the same; changing the module set requires close/open.

Optional `watod-config.local.sh` exports can point to different data roots:
`WATOD_SIM_CATALOG_DIRECTORY`, `WATOD_SIM_MANEUVER_DIRECTORY`, and
`WATOD_SIM_MAP_CONFIG_DIRECTORY`. The catalog root contains `playgrounds/` and
`scenarios/`. Recreate the session after changing a root directory.

Changes to executable code still require rebuilding the affected images.
Standalone ROS launches use installed data when these directory settings are absent.

## Create a playground

Create `config/playgrounds/my_world.yaml`:

```yaml
version: 1
id: my_world
map:
  town: Town10HD_Opt
spawn:
  index: 10
weather: ClearNoon
traffic:
  vehicles: 0
  pedestrians: 0
```

### World options

| Field | Meaning |
|-------|---------|
| `map` | Exactly one of `{town: Town10HD_Opt}` or `{xodr: oval.xodr}` |
| `ros_map` | Matching ROS map-config name; Town10HD playgrounds use `town10hd` |
| `ego_blueprint` | Vehicle blueprint; default `vehicle.mini.cooper` |
| `spawn.index` | CARLA spawn-point index |
| `spawn.straight_start` | Search for a continuous straight lane |
| `required_straight_m` | Required straight runout for that search |
| `spawn.s_margin_m` | Distance into a candidate lane before spawning |
| `spawn.road_id`, `spawn.lane_id`, `spawn.s` | Select a specific OpenDRIVE lane position |
| `spawn.pose` | Explicit map-frame ego pose, described below |
| `spawn.z_offset_m` | Height above a lane waypoint; default 0.4 m |
| `spawn.fallback_waypoint`, `spawn.fallback_z_offset_m` | Use a driving waypoint when a map has no spawn points |
| `weather` | CARLA weather preset; default `ClearNoon` |
| `seed` | Nonnegative random seed; default 42 |
| `traffic.vehicles`, `traffic.pedestrians` | Background actor counts |
| `traffic.pedestrian_crossing_factor` | Crossing fraction from 0 to 1 |
| `traffic.props.count`, `traffic.props.blueprints` | Static background props |
| `require_map_provenance` | Verify converted-map metadata and source hashes |

Keep matching OSM, OpenDRIVE, and conversion metadata together. The ROS map
configuration selects the OSM path, projection, origin, and alignment; see
[existing map configs](../../../world_modeling/world_modeling_bringup/config/map_configs).
For example, `oval` uses `maps/osm/oval.osm`, `maps/xodr/oval.xodr`, and its mapping record.
Map availability and lane geometry are checked before replacing the active world.

## Create a scenario

Scenarios use `version: 2`; the previous scenario format is not accepted. These sections select independent capabilities:

| Section | Configures |
|---------|------------|
| `trajectory` | The single trajectory source and optional maneuver |
| `world` | Base map/playground, ego, actors, weather, and goal |
| `completion` | When evaluation should finish |
| `metrics` | Values to record and optional pass/fail limits |
| `planning` | Normal planner speed and stop-distance parameters |
| `deadlines` | Optional overrides for setup, execution, and freshness timeouts |
| `events` | Optional timed `start_maneuver` or `begin_evaluation` actions |

### Controller tracking template

Create `config/scenarios/my_controller_test.yaml`:

```yaml
version: 2
id: my_controller_test
trajectory:
  source: fake_planner
  maneuver: brake_in_curve
world:
  map: oval
  modeling: map
  ego:
    blueprint: vehicle.mini.cooper
    spawn:
      straight_start: true
      s_margin_m: 10.0
  required_straight_m: 55.0
completion:
  type: maneuver_finished
metrics:
  lateral_tracking_rmse_m: {max: 0.6}
  max_lateral_error_m: {max: 1.5}
  distance_progressed_m: {min: 35.0}
  completed: {equals: true}
deadlines:
  execution_s: 65.0
```

`world.map` references a playground. World settings override that playground;
nested mappings merge and lists replace the base list. `world.ego` sets its
blueprint and spawn. Use `world.ros_map` if the ROS map-config name differs from
the playground's `ros_map` value (or its name when no override is set).

### Inferred modules

There is no separate stack or mode field. Modules follow the requested capabilities:

| Scenario data | What starts |
|---------------|-------------|
| No trajectory | World model and map markers, with duration completion |
| Fake-planner trajectory, no world modeling | Action container: fake planner and controller |
| Fake-planner trajectory plus `world.modeling: map` | Action plus world model and map markers |
| Fake-planner trajectory plus `world.modeling: full` | Action plus full world modeling |
| `trajectory.source: planner` | Action planners/controller plus full world modeling |

A named maneuver implies `trajectory.source: fake_planner`; normal planning uses
`trajectory.source: planner`. Omitting the trajectory requests a world-only run
with duration completion. `world.modeling` defaults to `off` for a fake-planner
trajectory, `full` for normal planning, and `map` for a world-only run. It accepts `off`, `map`, and `full`; a normal planner
requires `full` and a configured goal. Full world modeling supplies prediction,
costmap, and behavior in addition to the map. Planning nodes live in the action
image, not the world-modeling image.

A scenario always has one trajectory source and one command owner. All current
controller configurations use adaptive pure pursuit.

To add a maneuver, put a JSON file in the maneuver directory and use its filename
stem in `trajectory.maneuver`. There is no maneuver-name allowlist. See the
[fake-planner JSON format](../../../action/fake_planner/README.md#maneuver-json).
Managed maneuvers set `ramp_up_accel: 0`; the node's timed release ramp handles
startup. A closed maneuver needs duration or goal completion instead of
`maneuver_finished`. Preparation validates the file before launching it.

### Actors and poses

Scenario `world.actors` is a list of named vehicles. For example:

```yaml
actors:
  - id: obstacle
    blueprint: vehicle.mini.cooper
    static: true
    pose:
      frame: map
      x: 25.0
      y: -12.0
      z: 0.4
      yaw_deg: 90.0
```

Map coordinates and yaw use the ROS `map` frame. The world builder converts y
and yaw to CARLA coordinates; do not flip them in YAML. An explicit ego spawn
uses the same pose format under `world.ego.spawn.pose`.

A lane-relative pose is `{frame: lane, distance_m: 40.0}`. It follows the
continuous driving lane ahead of the ego's selected lane waypoint, rather than
adding 40 to its x coordinate. Lane-relative poses require a lane-based ego spawn.
Static actors have physics disabled; other vehicles have physics enabled but
no scripted motion or autopilot is implied. Stopping-target completion currently
requires a static actor.

### Goals and the parked-vehicle template

`world.goal` accepts a map pose or a lane-relative pose. The existing
[parked-car scenario](config/scenarios/parked_car_straight.yaml) declares:

```yaml
world:
  map: oval
  actors:
    - id: parked
      blueprint: vehicle.mini.cooper
      static: true
      pose: {frame: lane, distance_m: 40.0}
  goal: {frame: lane, distance_m: 80.0}
completion:
  type: stopped_behind_actor
  actor: parked
  clearance_m: [2.0, 8.0]
  speed_mps: 0.15
  hold_s: 3.0
```

[ConfiguredWorld](carla_scenarios/worlds/configured_world.py) resolves these poses.
The runner's [`_do_ready()` and `_publish_goal()`](../../simulation_manager/simulation_manager/simulation_runner.py)
send the resolved goal on `/goal_point` during preparation. The behavior tree
uses that goal to establish a route before play. The goal distance is scenario
data; it is not hardcoded in the runner. Planner speed and stop distance are
configured under `planning.max_speed_mps` and `planning.stop_distance_m`.

### Completion choices

| Type | Required data / condition |
|------|---------------------------|
| `maneuver_finished` | Fake planner reports driven completion and a stop |
| `stopped_behind_actor` | Named actor, clearance range, speed limit, and continuous hold time |
| `goal_reached` | Goal within `tolerance_m` at or below `speed_mps`, for `hold_s` (default 1 s) |
| `duration` | Run for `duration_s` |

A new combination of existing choices needs only data. A new metric algorithm,
actor motion primitive, completion type, or executable component needs code.

## Scoring and outputs

A metric set to `true` is recorded. A mapping adds `min`/`max` limits for numeric
values or `equals` for booleans. Set a metric to `false` or omit it to exclude it.
Only selected limits affect the score; completion conditions still apply.
Unknown metric names are rejected rather than silently ignored.

| Metrics | Availability |
|---------|--------------|
| `lateral_tracking_rmse_m`, `max_lateral_error_m` | Fake-planner reference tracking |
| `distance_progressed_m`, `sample_count`, `speed_mps`, `elapsed_s` | All current driving modes |
| `closest_clearance_m`, `final_clearance_m`, `stopped_duration_s` | Named stopping target |
| `goal_distance_m` | Configured goal |
| `completed`, `collision` | Completion and collision observations |

Controller progress follows the reference path, including curves. Normal planner
progress is projected along the initial lane. Collision, invalid data, missing
required observations, and stalled clocks always terminate a run independently
of optional metric limits. An execution timeout fails an unfinished run.

Attempts save the resolved definition, world geometry, and selected maneuver JSON
with its hash. See [results and diagnostics](../../simulation_manager/README.md#results-and-diagnostics).

## Validate, install and run

Validate source data without starting ROS or CARLA:

```bash
PYTHONPATH=src/simulation/carla_ros_bridge/carla_scenarios:src/simulation/simulation_runtime python3 -B - <<'PY'
from pathlib import Path
from carla_scenarios.definitions import load_definition
base = Path("src/simulation/carla_ros_bridge/carla_scenarios/config")
for category in ("playgrounds", "scenarios"):
    for path in sorted((base / category).glob("*.yaml")):
        load_definition(base, category, path.stem)
        print(f"OK {category}/{path.stem}")
PY
```

Then use `watod sim list`, `watod sim open scenario NAME`, and `watod sim play`.
Map geometry, installed executable packages, and ROS readiness are checked during
preparation; a valid file alone does not establish driving performance.

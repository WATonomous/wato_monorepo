# Authoring playgrounds and scenarios

This guide describes the implemented YAML options and where to extend behavior.
See the [catalog overview](README.md) for responsibilities and layout, and the
[manager command reference](../../simulation_manager/README.md#commands) for
running definitions. Examples below belong in this package's `config/` tree.

## Choose a starting point

| Existing definition | Useful starting point for |
|---------------------|---------------------------|
| `town10hd_default`, `town10hd_light`, `town10hd_heavy` | Town10HD traffic and weather development |
| `empty`, `custom` | Minimal worlds or traffic with static props |
| `oval`, `ring_road` | Existing converted OpenDRIVE maps |
| `controller_snake` | Slalom tracking with the fake planner and adaptive pure pursuit |
| `parked_car_straight` | Detection, costmap, planning and stopping behind a vehicle, using adaptive pure pursuit |

New YAML files can vary the supported settings below. The current automated
test families are slalom tracking and stopping behind a parked vehicle. A new
lap test, pedestrian crossing, moving obstacle sequence or completion rule
needs implementation work as described under [Extending behavior](#extending-behavior).

## Create a playground

Create `config/playgrounds/town10hd_practice.yaml` relative to this package:

```yaml
---
version: 1
id: town10hd_practice
description: Town10HD traffic and prop practice
map:
  town: Town10HD_Opt
seed: 42
ego_blueprint: vehicle.mini.cooper
spawn:
  index: 5
weather: CloudyNoon
traffic:
  vehicles: 8
  pedestrians: 10
  pedestrian_crossing_factor: 0.1
  props:
    count: 2
    blueprints:
      - static.prop.trafficcone*
```

The filename stem and `id` must match. Names must match `[a-z][a-z0-9_]*`, and
`version` must be `1`. No Python registration is needed for a new definition.
After preparing an updated simulation image, use `town10hd_practice` as the
playground name in the manager command reference.

For an existing converted map, a minimal definition is
`config/playgrounds/oval_practice.yaml`:

```yaml
---
version: 1
id: oval_practice
map:
  xodr: oval.xodr
seed: 42
spawn:
  index: 0
  fallback_waypoint: true
  fallback_z_offset_m: 0.5
traffic:
  vehicles: 0
  pedestrians: 0
```

`map.xodr` is a filename under the server's `xodr_map_directory`, normally
`/opt/watonomous/maps/xodr/`. The repository's `maps/` directory is bind-mounted
at `/opt/watonomous/maps`. Opening a world does not convert an OSM file or
generate a missing XODR. Town worlds must already exist in the CARLA server.

### World options

These settings are read by `ConfiguredWorld` and the scenario server. Defaults
below apply when a field is omitted.

| Field | Default | Meaning |
|-------|---------|---------|
| `version`, `id` | Required | Catalog version `1` and filename stem |
| `description` | `id` | Human-readable world description |
| `map.town` or `map.xodr` | Required | Select exactly one CARLA town or XODR filename |
| `seed` | `42` | Nonnegative integer for Python-side actor/blueprint selections |
| `ego_blueprint` | `vehicle.mini.cooper` | CARLA blueprint filter; the first match is used, so prefer an exact ID |
| `weather` | `ClearNoon` | A named CARLA weather preset, such as `CloudyNoon` or `ClearSunset` |
| `traffic.vehicles` | `0` | Requested number of NPC vehicles, with autopilot enabled |
| `traffic.pedestrians` | `0` | Requested number of walking pedestrians |
| `traffic.pedestrian_crossing_factor` | `0.0` | Crossing fraction in `[0, 1]` |
| `traffic.props.count` | `0` | Requested static prop count at available map spawn points |
| `traffic.props.blueprints` | Empty | List of blueprint filters, required when prop count is positive |
| `require_map_provenance` | `false` | Require a verified conversion record for an XODR map |

Counts must be nonnegative integers. Counts are requests: spawn-point limits,
occupied positions, unavailable blueprints or missing pedestrian navigation can
produce fewer actors. Props reserve the first available spawn locations before
NPC vehicles use the remaining locations. The seed controls Python randomness;
it does not by itself seed CARLA navigation or Traffic Manager, or guarantee
identical traffic on every run.

CARLA ports, FPS, sensor configuration and render mode are configured through
the existing launch/module settings, including
`simulation_bringup/config/carla_bridge.yaml`; they are not playground fields.

### Spawn options and precedence

| Field under `spawn` | Default | Meaning |
|---------------------|---------|---------|
| `straight_start` | `false` | Search for a continuous, approximately straight driving lane |
| `s_margin_m` | `10.0` | Advance a straight-search candidate this far before testing runout |
| `road_id`, `lane_id`, `s` | Unset | All three select a CARLA/OpenDRIVE lane waypoint; `s` is the road reference distance in metres |
| `z_offset_m` | `0.4` | Height added to a straight-search or explicit lane-waypoint spawn |
| `index` | `0` | Index in the map's spawn-point list, clamped to its last point |
| `fallback_waypoint` | `false` | Permit the first generated driving waypoint when there are no map spawn points |
| `fallback_z_offset_m` | `0.5` | Height added when using that fallback waypoint |

Selection order is: straight search; otherwise a complete `road_id/lane_id/s`
triple; otherwise a map spawn point; otherwise the enabled waypoint fallback.
Use the OpenDRIVE map's actual road/lane IDs. An index spawn does not apply
`z_offset_m`. Raw `x/y/yaw` spawning is not a catalog feature.

`required_straight_m` is a top-level world setting, defaulting to `0.0`. It is
used only with `spawn.straight_start`. The search samples the existing lane and
rejects branching, junctions, heading changes over one degree or lateral drift
over 0.25 m. A large oval can still fail if no candidate has the required
straight runout. The search selects a valid position; it does not modify the map
or guarantee clearance for a slalom's full lateral envelope.

### Scenario fixtures

Put test-only settings in a scenario's `setup`, rather than publishing a
playground just for a test fixture.

| World setting | Default | Meaning |
|---------------|---------|---------|
| `parked.distance_m` | No parked actor | Spawn one stationary, physics-disabled vehicle this far ahead along the selected lane |
| `parked.blueprint` | `vehicle.mini.cooper` | Blueprint filter for the parked vehicle |
| `goal_distance_m` | `0.0` | Place a goal this far ahead along the selected lane; zero uses the ego spawn location |

Parked and positive-distance goal fixtures require a lane waypoint spawn, not
just a map spawn-point index. Use straight search for the current straight-line
tests, with enough runout for the actor, goal and authored trajectory. Use zero
NPC traffic when establishing a repeatable baseline.

## Create a scenario

A scenario references a public playground by its canonical YAML `id`. Its
`setup` is recursively merged into that playground without changing the
original. Nested mappings merge; lists and scalar values replace the base
value. A scenario cannot override `map` through `setup`; reference a different
playground to change maps.

| Scenario field | Meaning |
|----------------|---------|
| `version`, `id` | Version `1` and filename stem, as for playgrounds |
| `playground` | Existing playground YAML ID |
| `setup` | Optional overrides for `spawn`, `traffic`, `weather`, `seed`, `ego_blueprint`, `parked`, `required_straight_m`, `goal_distance_m`, `require_map_provenance` |
| `stack`, `inputs`, `completion` | One of the supported combinations below |
| `events` | Nonempty, ordered sequence of reviewed actions |
| `deadlines` | All five positive timeout/freshness settings below |
| `thresholds` | Required scoring limits for the selected test family |
| `metrics` | Declared metric names; see [Scoring and outputs](#scoring-and-outputs) |

### Stack selection and component choices

`stack` selects a reviewed preset in the host helper; it is not a ROS package,
node name or arbitrary launch description.

| `stack` | Application components | Accepted `inputs` | `completion` |
|---------|------------------------|-------------------|--------------|
| `controller_pure_pursuit` | Fake planner and adaptive pure pursuit | `maneuver: steady_state_slalom` only | `maneuver_finished` |
| `autonomy` | World modeling, behavior, lattice/trajectory planning and adaptive pure pursuit | Named `map`, positive `stop_distance_m` and `max_speed_mps` | `stopped_behind_actor` |

There are no implemented `controller`, `trajectory_planner` or `nodes` selectors
in scenario YAML. Extra names do not discover or launch packages automatically.
The presets, launch argument wiring and image/package checks are defined in the
host helper in the following CLI layer; required lifecycle nodes
and observations are defined in the
[manager](../../simulation_manager/simulation_manager/simulation_runner.py).
All scored scenarios currently use adaptive pure pursuit. A playground allows
manual application launches without choosing a scored preset.

Selectable planners or component combinations are a future extension, not an
existing feature. Such support would need reviewed component choices, allowed
parameters, installed-package checks, compatible topic/remap wiring and
lifecycle/readiness rules. Each allowed combination must retain exactly one
trajectory publisher and one vehicle-command owner, and preserve the current
adaptive pure pursuit requirement. Selecting individual nodes also requires
specifying their dependencies; a node name alone is insufficient.

### Controller tracking template

Create `config/scenarios/controller_snake_practice.yaml`. This varies weather
while retaining the shipped controller test's scoring requirements:

```yaml
---
version: 1
id: controller_snake_practice
playground: oval
stack: controller_pure_pursuit
inputs:
  maneuver: steady_state_slalom
setup:
  weather: CloudyNoon
  seed: 42
  ego_blueprint: vehicle.mini.cooper
  spawn:
    straight_start: true
    s_margin_m: 10.0
    z_offset_m: 0.4
  required_straight_m: 82.0
  goal_distance_m: 80.0
events:
  - when: play
    do: start_maneuver
completion: maneuver_finished
deadlines:
  readiness_s: 90.0
  execution_s: 65.0
  stalled_clock_s: 15.0
  first_output_s: 2.0
  data_freshness_s: 2.0
thresholds:
  lateral_tracking_rmse_m: 0.6
  max_lateral_error_m: 1.5
  min_progress_m: 65.0
  min_samples: 30
metrics:
  - lateral_tracking_rmse_m
  - max_lateral_error_m
  - completed
```

The fake planner's other manual maneuvers need additional validation/preflight
support before they can be used as scored scenarios.

### Parked-vehicle template

Create `config/scenarios/parked_car_practice.yaml`. This retains the shipped
parked-car scoring requirements and moves the parked actor to 45 m:

```yaml
---
version: 1
id: parked_car_practice
playground: oval
stack: autonomy
inputs:
  map: oval
  stop_distance_m: 2.0
  max_speed_mps: 3.0
setup:
  require_map_provenance: true
  seed: 42
  ego_blueprint: vehicle.mini.cooper
  spawn:
    straight_start: true
    s_margin_m: 10.0
    z_offset_m: 0.4
  required_straight_m: 80.0
  parked:
    distance_m: 45.0
    blueprint: vehicle.mini.cooper
  goal_distance_m: 80.0
events:
  - when: play
    do: begin_evaluation
completion: stopped_behind_actor
deadlines:
  readiness_s: 90.0
  execution_s: 90.0
  stalled_clock_s: 15.0
  first_output_s: 3.0
  data_freshness_s: 2.0
thresholds:
  min_progress_m: 15.0
  min_clearance_m: 2.0
  max_clearance_m: 8.0
  stopped_speed_mps: 0.15
  stopped_duration_s: 3.0
  min_samples: 30
  max_lateral_offset_m: 1.5
metrics:
  - closest_clearance_m
  - final_clearance_m
  - distance_progressed_m
  - stopped_duration_s
```

The autonomy preset disables overtaking and Eidos. Its stop-distance and speed
inputs configure the action launch; they do not replace the evaluator's
clearance and stopped-speed limits.

`inputs.map` selects a world-modeling map preset, not an XODR filename. The
shipped end-to-end baseline is `oval`; current host preflight explicitly checks
for `maps/osm/oval.osm`. Another map needs a matching Lanelet2 map, installed
world-modeling preset, coordinate alignment and a review of the straight-line
evaluator. Changing this string alone does not establish those requirements.

Autonomy readiness requires converted-map provenance. Keep matching XODR and
OSM files with the same stem under `maps/xodr/` and `maps/osm/`. A
`NAME.xodr.metadata.json` record takes precedence over `NAME.mapping.json`;
source and output hashes must match the actual files. An invalid preferred
record is rejected rather than falling back to another record. A Town-only
playground does not provide this converted-map provenance.

### Events and completion

Completion labels in the stack table select implemented test families; they
are not arbitrary expressions or ROS topic conditions.

Each event has exactly `when` and `do`:

| Option | Supported behavior |
|--------|--------------------|
| `when: play` | Deliver at the start of play |
| `when: {elapsed_s: 5.0}` | Deliver when simulated execution time reaches five seconds |
| `do: start_maneuver` | Call the fake planner's start service; controller preset only |
| `do: begin_evaluation` | Record delivery of the evaluation marker; no separate service or scoring gate |

Timestamps must be finite, nonnegative, in nondecreasing order, and no later
than `execution_s`. Equal-time events execute in YAML order. Deliberate pauses
stop the simulation timeline. Events are consumed once; delivery failure ends
the attempt. A delayed `begin_evaluation` does not delay scoring, which is
active during `RUNNING`. Delaying `start_maneuver` must also fit the trajectory
output deadline measured from play.

Spawning actors mid-run, changing weather mid-run, waiting for arbitrary topic
conditions and executing shell commands are not supported event actions.

### Deadlines

All five fields are required, finite and positive:

| Field under `deadlines` | Units | Meaning |
|-------------------------|-------|---------|
| `readiness_s` | Wall seconds | Wait for the required bridge/application nodes and observations before play |
| `execution_s` | Simulation seconds | Maximum running duration before an unfinished test fails |
| `stalled_clock_s` | Wall seconds | Maximum clock silence during execution before a data error |
| `first_output_s` | Simulation seconds | Time after play allowed for the first trajectory output |
| `data_freshness_s` | Simulation seconds | Maximum accepted age of required observations |

The host CLI also has bounded service waits. Increasing a YAML deadline does
not remove those host limits. Execution has an additional wall-time bound of
`execution_s * 5 + stalled_clock_s`, excluding deliberate pauses.

### Scoring and outputs

All threshold values must be finite and positive. Keep the shipped baseline
requirements when creating variants; do not loosen them to conceal a failure.
Current baseline thresholds remain provisional pending target-host measurements.

| Field under `thresholds` | Applies to | Meaning |
|--------------------------|------------|---------|
| `min_progress_m` | Both | Minimum forward displacement projected onto the initial lane direction; not route arc length |
| `min_samples` | Both | Minimum evaluated observations; use an integer |
| `lateral_tracking_rmse_m` | Controller | Maximum root-mean-square distance to the reference path |
| `max_lateral_error_m` | Controller | Maximum observed distance to the reference path |
| `min_clearance_m`, `max_clearance_m` | Autonomy | Inclusive bumper-clearance interval for a valid stop; minimum must be below maximum |
| `stopped_speed_mps` | Autonomy | Maximum speed while establishing the stopped interval |
| `stopped_duration_s` | Autonomy | Required continuous stopped interval in simulation time |
| `max_lateral_offset_m` | Autonomy | Allowed offset from the initial lane line while stopped; optional, default `1.5` |

The controller test passes after the fake planner reports completion, with
enough samples/progress and both tracking-error limits satisfied. The parked
test passes after holding its clearance, speed, lateral-position and progress
requirements for the required duration, with enough samples. Crossing the
parked actor's rear bumper fails the test. An odometry gap over 0.5 seconds
breaks the continuous stopped interval.

Collision checks apply to both test families. Missing/stale observations,
invalid transforms, inactive required routes or missing parked-actor evidence
produce data errors. An unfinished test at its execution deadline fails.

`metrics` documents intended outputs; it currently does not filter the result
file or control evaluation. The runner writes its full built-in metric set:
`lateral_tracking_rmse_m`, `max_lateral_error_m`, `completed`,
`closest_clearance_m`, `final_clearance_m`, `distance_progressed_m`,
`stopped_duration_s` and `sample_count`. Inapplicable values may be null.
Removing a metric from YAML does not disable a check; adding a name does not
implement a new metric.

Storage paths, result files and exit codes are documented under
[results and diagnostics](../../simulation_manager/README.md#results-and-diagnostics).

## Validate, install and run

From the repository root, use a Python environment with PyYAML for a source-only
schema check. This creates no actors, containers or result files:

```bash
PYTHONPATH=src/simulation/carla_ros_bridge/carla_scenarios python3 -B - <<'PY'
from pathlib import Path
from carla_scenarios.definitions import load_definition

base = Path("src/simulation/carla_ros_bridge/carla_scenarios/config")
for category in ("playgrounds", "scenarios"):
    for path in sorted((base / category).glob("*.yaml")):
        load_definition(base, category, path.stem)
        print(f"OK {category}/{path.stem}")
PY
```

This checks definition structure, references and numeric constraints. Actual
map availability, blueprint/weather availability, lane geometry and ROS
readiness are checked during preparation and need operator runtime validation.
Unrecognized fields are not uniformly rejected; only documented settings have
implemented behavior.

The catalog-listing command reads the host checkout. Runtime loading reads the
installed catalogs under `share/carla_scenarios/config/` inside the image. Seeing a new name
in `list` does not prove it is installed. `setup.py` installs both YAML catalogs
using globs, so no file-by-file registration is needed. Prepare an updated
simulation image for catalog/world changes, an action image for changed action
assets, and a world-modeling image for changed map presets. Pulling Git alone
does not update an existing image or container.

After preparing the necessary images/assets, follow the
[manager command reference](../../simulation_manager/README.md#commands), using
`controller_snake_practice` or `parked_car_practice` as the scenario name.

## Extending behavior

| Needed change | Implementation location |
|---------------|-------------------------|
| More traffic, different supported spawn, weather or actor distance | YAML world settings or scenario `setup` |
| Reusable world/actor operation | [ConfiguredWorld](carla_scenarios/worlds/configured_world.py) and [ScenarioBase](carla_scenarios/scenario_base.py) |
| New allowed fields, event actions, test family or completion pair | [Definition validation](carla_scenarios/definitions.py) |
| Readiness, event delivery, new metrics or completion checks | [Simulation manager](../../simulation_manager/simulation_manager/simulation_runner.py) |
| New reviewed application stack or input-to-launch wiring | Host helper in the following CLI layer, including preflight |
| Another fake-planner maneuver in scored tests | Definition validation, host preflight and installed [fake-planner assets](../../../action/fake_planner) |
| Another autonomy map preset | Matching converted maps and installed [world-modeling configuration](../../../world_modeling/world_modeling_bringup/config/map_configs) |

Keep one CARLA tick and `/clock` authority in `scenario_server`. World primitives
request setup ticks through that server and register created actors for scoped
cleanup. Keep application launches out of playgrounds, and keep all automated
scenario controllers on adaptive pure pursuit. Add behavior checks for new
events/evaluators and validate them manually after rebuilding the affected
images. YAML parsing alone does not establish repeatability or passing behavior.

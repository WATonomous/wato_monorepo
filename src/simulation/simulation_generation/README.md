# Controller trajectory generation prototype

`watod sim generate scenario` turns a prompt into an open fake-planner maneuver
and a version-two controller scenario with optional named vehicle actors.
It runs on the host without Docker or a ROS installation.
Generated data uses the existing mounted scenario catalog and
maneuver directory, so no runtime changes or image rebuilds are needed.

This first version focuses on custom controller paths. It composes straights,
smooth lateral shifts, distance-based dwells, slaloms, and constant-radius arcs.
Actors may use lane-relative or explicit ROS map poses. Static parked vehicles
can be combined with authored paths that swerve around them.
It does not generate lane-following routes, arbitrary waypoint trajectories,
scripted actor motion, autopilot, pedestrians, props, reverse driving, or closed
laps. Requests for those capabilities should receive an explanation from the
model instead of a proposal.
The validator enforces the supported data shape; prompt interpretation remains
the model's responsibility. Inspect the result for semantic fidelity.

## Generate and run

Install the host dependency if PyYAML is unavailable:

```bash
python3 -m pip install -r src/simulation/simulation_generation/requirements.txt
```

Set `OPENAI_API_KEY` in your host environment, then run:

```bash
watod sim generate scenario --name custom_swerve --map oval \
  --prompt "At 2 m/s, drive straight 8 m, shift left 1 m over 12 m, hold that offset for 4 m, return over 12 m, then drive 10 m and stop."
```

`-prompt` is also accepted. Omit `--name` for a unique generated ID.
The model defaults to `gpt-4.1`; override it with `--model` or `OPENAI_MODEL`.
Use a model available to your API account that supports Responses function calls.
`--map` selects an existing playground and defaults to `oval`. The model cannot
change this selection. If a prompt requests another map, use its playground ID
in `--map` too.

To generate the parked car and the swerve together:

```bash
watod sim generate scenario --name parked_car_swerve --map oval \
  --prompt "Place a static Mini 22 m ahead along the ego lane. At 2 m/s, drive 8 m, shift left 3.5 m over 12 m, hold that offset for 10 m, shift right 3.5 m over 12 m, then drive straight 10 m and stop."
```

No manual actor edit is needed. Inspect `world.actors` in the generated scenario.
The controller follows the generated path; it does not replan around the car.

The command prints a directory under `.cache/watod-sim/generated/` containing:

| File | Purpose |
|------|---------|
| `proposal.json` | Accepted model proposal; can be validated again with `--candidate` |
| `scenario.yaml` | Complete generated scenario |
| `maneuver.json` | Complete fake-planner maneuver |
| `preview.svg` | Ego-relative path geometry in metres; green start, red end; x forward, y left |
| `report.json` | Prompt, model, prompt hash, catalog context, validation attempts, geometry, actor definitions, installed paths |

The preview shows authored geometry only. It omits the runtime's uniform
resampling, braking profile, stop pad, map anchoring, and actor positions. Actor
poses are resolved when the scenario opens; inspect them in the generated YAML
and the running CARLA world. The SVG is not a map overlay.
At runtime, inspect `/sim/reference_path` alongside map markers in Foxglove.

On success, the command also creates `scenarios/NAME.yaml` in the configured
catalog and `NAME.json` in the maneuver directory. It refuses to overwrite either
file. Existing scenarios, including tuned ego poses, are never edited. Use
`--dry-run` to save the preview and artifacts without installing either file.

With a prepared managed session:

```bash
watod sim open scenario custom_swerve
watod sim play
```

Close an existing playground or scenario before opening another. See the
[manager setup instructions](../simulation_manager/README.md#quick-start) if no
session exists. Generation never starts, resets, or modifies a running session.
Opening the generated scenario starts the fake planner, pure pursuit controller,
and map-only world modeling. Tracking errors and progress are recorded; completion
and collision have pass/fail limits. No model-authored tracking thresholds are
treated as controller acceptance criteria.

## Offline exploration

This exercises the same validation and artifact path without an API key or call:

```bash
watod sim generate scenario --name preview_swerve --map oval --dry-run \
  --candidate src/simulation/simulation_generation/examples/gentle_swerve.json
```

Remove `--dry-run` to install the candidate. A candidate contains `description`,
`default_speed`, `segments`, and optional `actors`; it is a proposal, not a
complete scenario or maneuver JSON. See the example and the
[fake-planner format](../../action/fake_planner/README.md#maneuver-json).

The parked-car example exercises both actor and path authoring offline:

```bash
watod sim generate scenario --name preview_parked_swerve --map oval --dry-run \
  --candidate src/simulation/simulation_generation/examples/parked_car_swerve.json
```

## Airfield circle

The `airfield` playground uses the tracked `maps/osm/airfield.osm` and
`maps/xodr/airfield.xodr`, with the converter's metadata and lane mapping beside
the XODR. These four assets are explicitly force-added despite the general
`maps/` ignore rule; a checkout needs no private map download. Other generated
conversion diagnostics remain ignored. See the
[conversion instructions](../../../watod_scripts/tools/osm_to_xodr/README.md#airfield)
to reproduce the map.

The source describes 20 adjacent 3.5 m lanes, each 400 m long, divided into eight
50 m sections (160 lanelets). Its paved rectangle is ROS map x = [-200, 200] m,
y = [-35, 35] m. The map config uses UTM, fixed alignment, and geographic origin
(45.5017, -73.5673, 0). A circle crosses lane boundaries on this continuous
surface; this is circular path following. Tire-slip drifting requires additional
controller and vehicle-physics work.

Generate the exact example offline, without OpenAI or CARLA:

```bash
watod sim generate scenario --name airfield_circle --map airfield \
  --candidate src/simulation/simulation_generation/examples/airfield_circle.json
```

For a preview without installing the scenario and maneuver, add `--dry-run`.
Alternatively, with `OPENAI_API_KEY` set on the host, generate from a prompt
(choose this command instead of the offline command for the same name):

```bash
watod sim generate scenario --name airfield_circle --map airfield \
  --prompt "At 2 m/s, drive straight 5 m, then two consecutive left 180-degree arcs of radius 20 m to make one circle, then straight 10 m and stop. No actors."
```

The model receives playground settings and catalog examples, not the OSM
geometry. It cannot determine circle clearance. Check that the generated
maneuver has the example's four segments, `default_speed: 2.0`, and
`closed: false`. Each arc is limited to 180 degrees, so use two arcs for one
circle. Keeping the maneuver open allows `maneuver_finished` completion after
the exit straight and braking.

**Before opening the scenario, replace its automatic lane spawn.** Generation
sets `world.ego.spawn.straight_start: true`. A straight-lane search establishes
forward runout only and may select an
edge lane with insufficient lateral space. Run this from the repository root
after either generation command (it uses the default catalog directory):

```bash
python3 - <<'PY'
from pathlib import Path
import yaml

path = Path("src/simulation/carla_ros_bridge/carla_scenarios/config/scenarios/airfield_circle.yaml")
scenario = yaml.safe_load(path.read_text())
scenario["world"]["ego"]["spawn"] = {
    "pose": {"frame": "map", "x": -5.0, "y": -20.0, "z": 0.5, "yaw_deg": 0.0}
}
scenario["world"].pop("required_straight_m", None)
path.write_text(yaml.safe_dump(scenario, sort_keys=False))
PY
```

Replace the whole spawn mapping, removing `straight_start` and any lane
selectors. If using `WATOD_SIM_CATALOG_DIRECTORY`, edit that catalog's scenario
instead. These are ROS map coordinates: x east, y north, yaw 0 degrees east.
The world builder converts them to CARLA x = -5, y = +20, yaw = 0; do not
negate y in the YAML. The 5 m entry reaches (0, -20), the left circle is centered
at (0, 0), and the exit ends at (10, -20). The reference including its 10 m
zero-speed stop pad stays within x = [-20, 20], y = [-20, 20], leaving 15 m to
the lateral surface edges for the reference centerline. Vehicle footprint,
tracking error, mesh seams, and actual dynamics still require a simulator run.

Validate the edited scenario through the shared loaders, without starting ROS
or CARLA:

```bash
PYTHONPATH=src/simulation/carla_ros_bridge/carla_scenarios:src/simulation/simulation_runtime python3 -B - <<'PY'
from pathlib import Path
from carla_scenarios.definitions import load_definition, world_definition
from simulation_runtime.assets import map_config_file

catalog = Path("src/simulation/carla_ros_bridge/carla_scenarios/config")
scenario = load_definition(catalog, "scenarios", "airfield_circle")
world = world_definition(catalog, "scenario", "airfield_circle")
assert world["spawn"]["pose"]["frame"] == "map"
assert "straight_start" not in world["spawn"]
assert scenario["inputs"]["map"] == "airfield"
print("Validated:", map_config_file("airfield"), world["spawn"])
PY
```

With the managed-session images prepared, start and run when ready:

```bash
watod sim setup
# If another playground or scenario is open, run: watod sim close
watod sim open scenario airfield_circle
watod sim play
watod sim status
```

Inspect `/sim/reference_path` with the Airfield map markers in Foxglove. Use
`watod sim close` to close the attempt and `watod sim stop` to stop the session.
The data files need no registration code or image rebuild; see
[session setup](../simulation_manager/README.md#quick-start) for initial images.

## Vehicle actors

Proposals accept up to 16 vehicle actors in the existing `world.actors` format:

```json
{
  "actors": [
    {
      "id": "parked",
      "blueprint": "vehicle.mini.cooper",
      "static": true,
      "pose": {"frame": "lane", "distance_m": 22.0, "z_offset_m": 0.4}
    }
  ]
}
```

Actor IDs must be unique lowercase names; `ego_vehicle` is reserved. Blueprints
must name CARLA vehicles, with availability checked when CARLA prepares the
scenario. `static` defaults to true; false enables physics without any scripted
motion or autopilot. Actor fields for speed, routes, and behavior are rejected.

`frame: lane` places an actor 0–300 m ahead along the selected ego spawn lane,
independently of the fake-planner trajectory. Lane poses do not support lateral
offset or yaw. For a position explicitly supplied in the ROS map frame, use:

```json
{"frame": "map", "x": 25.0, "y": -12.0, "z": 0.4, "yaw_deg": 90.0}
```

Map poses are absolute, not offsets from the ego. The generator chooses a lane
spawn at preparation time, so it cannot infer an absolute map pose for an actor
beside the ego without map/spawn geometry. Requests for arbitrary ego-relative
lateral actor placement remain unsupported. Existing proposals without `actors`
still generate an empty actor list.

## How generation works

1. Read the selected playground, matching ROS map config, and map assets from
   the checkout. Read current `controller_snake`, `brake_in_curve`, and
   `sine_with_dwell` examples and their maneuver JSON when available, plus
   `parked_car_straight` as an actor-authoring reference.
2. Send the user prompt and this bounded catalog context with the curated
   [`system_prompt.txt`](simulation_generation/system_prompt.txt) to OpenAI.
   The host API key is used only for authentication; it is not included in
   prompts, installed files, or generation records. Prompts and catalog examples
   are sent to the API and saved in the local report.
3. Let the model call only `validate_trajectory`. The host checks the proposal's
   fields, finite numbers, segment limits, speed bounds, geometry budget, and
   actor IDs, vehicle blueprints, physics flags, and pose formats.
   The tool cannot execute commands or choose paths. Invalid proposals return
   diagnostics to the model for repair. Generation stops on the first valid
   proposal, an unsupported-request explanation, an API failure, or the call
   limit (`--max-rounds`, default 4, maximum 8).
4. Compile the proposal into a fixed controller-test template: ego-relative open
   path, `ramp_up_accel: 0`, 10 m stop pad, maneuver-finished completion, selected
   metrics, and a bounded execution deadline. Validate the generated files in an
   isolated temporary catalog through the existing `load_maneuver` and
   `load_definition` functions. Copy proposed actors into `world.actors`.
5. Save review artifacts and install the maneuver before the scenario with
   exclusive file creation. A failed publish rolls back files created by that
   invocation, preserving pre-existing files. Files are local changes for review;
   generation does not commit or push them.

The client uses the [OpenAI Responses function-calling API](https://developers.openai.com/api/docs/guides/function-calling)
with `store: false` and no SDK dependency. API calls have a 90-second timeout and
are not automatically retried after transport errors. Generation may make up to
the configured number of billable calls while repairing invalid data.

The host honors `WATOD_SIM_CATALOG_DIRECTORY`, `WATOD_SIM_MANEUVER_DIRECTORY`, and
`WATOD_SIM_MAP_CONFIG_DIRECTORY`, including checkout-relative paths. Recreate a
managed session after changing data roots so its mounts match the generator.

## What validation establishes

The proposal must have 1–32 bounded segments, speeds from 0.5–8 m/s while moving,
no intermediate zero-speed point, a final straight of at least 5 m, and no more
than 300 m of sampled authored path. The host fixes maneuver sampling at 0.5 m
and uses the existing timed release ramp and braking behavior. The execution
deadline allows travel at the minimum accepted speed plus 60 seconds.

These are authoring and resource bounds. Schema and file validation do not prove
collision avoidance, road containment, steering feasibility, or scoring success.
The scenario requests a continuous straight lane using a forward-envelope
runout heuristic, extended to cover lane-relative actors plus 5 m;
CARLA may reject that spawn if the chosen map lacks enough space. Curved custom
paths can leave the road. Run and inspect CARLA before
treating a generated path as an accepted controller test.

Actor placement and a nominal pass-around path are authored together, but the
host does not compute vehicle bounding-box clearance, road fit, or a guaranteed
avoidance route. CARLA resolves actual poses and spawn overlap at preparation;
driving still needs validation in the simulator.

## Development checks

```bash
python3 -B -m unittest discover -s src/simulation/simulation_generation/tests -v
python3 -B -m unittest discover -s watod_scripts/tests -v
```

The generator tests cover the tool repair loop using mocked Responses output,
unbounded and malformed proposals, actor validation and repair, coordinate
composition, shared-loader round trips, actor-aware spawn runout, offline CLI
use, and no-overwrite behavior. They do not freeze shipped scenario poses or
claim live API/CARLA acceptance.

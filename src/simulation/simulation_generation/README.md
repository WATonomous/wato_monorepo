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

Install the host dependencies (PyYAML, pyproj, and Shapely):

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
| `map_context.json` | Projected driving polygons, holes, source hash, projection, exact ego spawn, and boundary margin sent to the model |
| `report.json` | Prompt, model, prompt hash, catalog context, validation attempts, geometry, actor definitions, installed paths |

The preview shows authored geometry only. It omits the runtime's uniform
resampling, braking profile, stop pad, map anchoring, and actor positions. Actor
poses are resolved to absolute map coordinates during generation; inspect them
in the generated YAML and the running CARLA world. The SVG is not a map overlay.
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

The model receives projected driving-surface polygons, x/y bounds, the ego spawn,
playground settings, and catalog examples. The host rejects paths and actors
outside the surface or its configured boundary margin. Check that the generated
maneuver has the example's four segments, `default_speed: 2.0`, and
`closed: false`. Each arc is limited to 180 degrees, so use two arcs for one
circle. Keeping the maneuver open allows `maneuver_finished` completion after
the exit straight and braking.

Generation automatically preserves Airfield's configured ego spawn:
ROS map x = -5 m, y = -20 m, z = 0.5 m, yaw = 0 degrees (east). No manual YAML
edit is needed. A generic straight-lane search establishes forward runout only
and can select an edge lane without circle clearance, so it is not used when a
playground supplies an explicit map pose.

The world builder converts the pose to CARLA x = -5, y = +20, yaw = 0. The 5 m
entry reaches (0, -20), the left circle is centered
at (0, 0), and the exit ends at (10, -20). The reference including its 10 m
zero-speed stop pad stays within x = [-20, 20], y = [-20, 20], leaving 15 m to
the lateral surface edges for the reference centerline. Vehicle footprint,
tracking error, mesh seams, and actual dynamics still require a simulator run.

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

Map poses are absolute, not offsets from the ego. When the playground supplies
an explicit `spawn.pose`, generation preserves it and actors must use map poses;
lane-relative actors require a lane waypoint and are rejected for such maps.
For other playgrounds, generation selects a deterministic OSM lane-center spawn
with the greatest boundary clearance among lane midpoints. It freezes that pose
before asking the model, resolves legacy lane-relative actors along unambiguous
OSM centerline continuations, and writes all actors as map poses. The model can
also calculate an actor's map x/y from a requested forward/left offset and the
known ego pose. Existing proposals without `actors` still generate an empty list.

## Map boundaries and coordinates

Every generation projects the Lanelet2 source into the configured ROS map frame
and unions its vehicle-driving lanelet polygons. Internal lane lines do not
constrain a custom maneuver, but holes, gaps, and outer edges do. Validation uses
the full polygons, rather than a rectangular bound or convex hull that could
fill non-drivable areas.

The model receives GeoJSON polygons in map metres, including holes, plus x/y
bounds, projection, source hash, exact ego spawn, and clearance. This is derived
geometry, not raw OSM XML. Model-context boundaries are simplified at 0.05 m and
cropped to 320 m around the spawn; host validation uses the complete unsimplified
surface. The existing 64 KB context limit still applies; overly detailed contexts
fail before any API request instead of silently dropping boundaries.

Export this information without OpenAI, Docker, or CARLA:

```bash
watod sim generate map-context --map airfield
```

It writes `.cache/watod-sim/map-context/airfield.json`. Normal generation includes
the same geometry automatically and saves `map_context.json` with its artifacts.

The host anchors the reference to the known ego pose, reproduces the planner's
fine sampling and uniform 0.5 m resampling, and includes its 10 m zero-speed stop
pad. The entire resulting polyline must fit within the driving surface with
`--boundary-clearance-m` (default 3 m), plus 0.1 m numerical/sampling allowance.
Actor centers must have the same clearance. Boundary failures include map x/y
diagnostics for the existing model repair loop. Invalid offline candidates fail
without publishing files. The margin is a center clearance, not a measurement of
the selected vehicle; choose it to accommodate vehicle size and tracking error.
Narrow roads may require a smaller margin appropriate to the car.

For a circle with a static car at a specific map coordinate:

```bash
watod sim generate scenario --name airfield_circle_actor --map airfield \
  --prompt "At 2 m/s drive straight 5 m, two left 180-degree arcs of radius 20 m, then straight 10 m and stop. Place one static Mini at ROS map x=45 m, y=10 m, yaw=90 degrees. Stay within the supplied driving polygons."
```

Or exercise the same path and actor placement entirely offline:

```bash
watod sim generate scenario --name airfield_circle_actor --map airfield \
  --candidate src/simulation/simulation_generation/examples/airfield_circle_actor.json
```

UTM and LocalCartesian projections with `alignment_mode: fixed` are supported.
Live TF alignment, missing/invalid lane polygons, an off-surface spawn, and
mismatched available OSM/XODR provenance fail before generation. If a playground
requires conversion metadata, missing provenance also fails. Without a verified
map pair, containment describes the OSM surface and cannot establish that a
separately supplied XODR or native CARLA town matches it. These checks are 2D;
overpasses, road elevation, actual mesh seams, blueprint dimensions, collisions,
controller tracking, and tire physics still require simulator inspection.

## How generation works

1. Read the selected playground, matching ROS map config, and map assets from
   the checkout. Read current `controller_snake`, `brake_in_curve`, and
   `sine_with_dwell` examples and their maneuver JSON when available, plus
   `parked_car_straight` as an actor-authoring reference.
2. Project and union the driving polygons, freeze the ego spawn, and send the
   user prompt and this bounded map/catalog context with the curated
   [`system_prompt.txt`](simulation_generation/system_prompt.txt) to OpenAI.
   The host API key is used only for authentication; it is not included in
   prompts, installed files, or generation records. Prompts and catalog examples
   are sent to the API and saved in the local report.
3. Let the model call only `validate_trajectory`. The host checks the proposal's
   fields, finite numbers, segment limits, speed bounds, geometry budget, and
   actor IDs, vehicle blueprints, physics flags, pose formats, and map containment
   for the anchored reference, stop pad, and actor centers.
   The tool cannot execute commands or choose paths. Invalid proposals return
   diagnostics to the model for repair. Generation stops on the first valid
   proposal, an unsupported-request explanation, an API failure, or the call
   limit (`--max-rounds`, default 4, maximum 8).
4. Compile the proposal into a fixed controller-test template: ego-relative open
   path, `ramp_up_accel: 0`, 10 m stop pad, maneuver-finished completion, selected
   metrics, and a bounded execution deadline. Validate the generated files in an
   isolated temporary catalog through the existing `load_maneuver` and
   `load_definition` functions. Write resolved map actors into `world.actors`.
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

The map checks establish containment of the generated reference and actor centers
in the projected OSM surface with the configured margin. They do not prove
collision avoidance, steering feasibility, actual vehicle containment during
tracking, or scoring success. Run and inspect CARLA before treating a generated
path as an accepted controller test.

Actor placement and a nominal pass-around path are authored together, but the
host does not compute actual vehicle bounding-box clearance or a guaranteed
avoidance route. CARLA resolves actual poses and spawn overlap at preparation;
driving still needs validation in the simulator.

## Development checks

```bash
python3 -B -m unittest discover -s src/simulation/simulation_generation/tests -v
python3 -B -m unittest discover -s watod_scripts/tests -v
```

The generator tests cover the tool repair loop using mocked Responses output,
unbounded and malformed proposals, actor validation and repair, coordinate
composition, shared-loader round trips, source-map spawn/actor resolution,
projection, holes and disconnected surfaces, boundary margins, stop-pad
containment, off-surface repair, offline CLI use, and no-overwrite behavior.
They do not claim live API/CARLA acceptance.

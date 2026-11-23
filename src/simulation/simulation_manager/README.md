# Simulation Manager

This ROS package manages readiness, playback, evaluation and results for a
reusable CARLA 0.10.0 session. The host helper owns containers and starts application
runtimes. Those runtimes own launch groups. The CARLA scenario server remains the
sole tick and `/clock` authority.
The `simulation_runner` executable, ROS node name and `/sim/*` services remain.

The [standard manual workflow](../SIMULATION_README.md) uses
`session_mode:=false`. The manager is enabled only with `session_mode:=true`,
using the existing simulation image. Prepare images with your configured tag;
automated scenarios also need the action image, and autonomy needs world
modeling. Builds and runtime acceptance are operator tasks.

For catalog structure and legacy selectors, see the [catalog overview](../carla_ros_bridge/carla_scenarios/README.md).
For YAML templates, supported options and component-selection limits, see the
[authoring guide](../carla_ros_bridge/carla_scenarios/AUTHORING.md).

## Commands

This is the command reference for managed sessions. Run on the host;
`NAME` is a definition ID from the catalog.

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

Launch application nodes yourself in a playground. Close the current world
before opening another. Follow node output with
`docker logs --follow <sim_session-container>`. Application output is in the
corresponding `sim_action` or `sim_world` container log.

## Application integration

`SimulationStackAdapter` prepares the two reviewed presets. It waits for the
bridge, binds a unique command topic, then starts the existing launch files with
`use_sim_time:=true` and `autostart:=false`. Autonomy starts world modeling before
action. Each group's existing lifecycle manager owns its node transitions; the
runner never calls application-node `change_state` services.

The `application_runtime` executable lives in the existing
`wato_lifecycle_manager` package, installed in the action, world-modeling and
simulation images. Each runtime owns one launch process group. Its ROS
`start`, `stop` and `status` services are under `/sim/runtime/action` or
`/sim/runtime/world`. The stack adapter sets its declared `configuration`
parameter with one atomic request. Only reviewed presets and inputs are accepted.
No shell commands or Docker access are exposed over ROS.

Reset blocks commands and stops the owned application groups before reloading
the world. It creates a fresh attempt and starts new application processes. CARLA,
base-session processes and saved results remain. Close stops those application
groups; container removal belongs to the host. Both CLI and Foxglove reset use
this same ROS path.

Production nodes retain their existing topics and messages. Simulation launch
files supply remaps and tuning. The bridge publishes body-frame odometry velocity
and subscribes to `/sim/attempts/ATTEMPT_ID/command`. Only general ROS-clock timer
fixes and the lattice missing-input guard remain in production node sources.

Planner speed-limit and overtaking policies are unchanged. The parked-car
scenario evaluates the production behavior; it can fail if the planner overtakes
or stops outside the required clearance. Simulation orchestration does not
change planner outputs to satisfy its assertions.

## Results and diagnostics

The local workspace directory contains:

```text
.cache/watod-sim/
  session.json                  Container ownership, session ID and configuration
  compose.json                  Effective Compose snapshot
  cli.lock                      Serializes commands from different terminals
  results/<run-id>/
    configuration.yaml          Scenario/world settings, Git revision and images
    events.json                 Delivered events and execution timestamps
    result.json                 Outcome, reasons, elapsed time and metric values
  startup-failures/<session-id>/ Logs and state saved before failed-setup cleanup
  recovery/<session-id>/         Diagnostic archives, when explicitly captured
```

Only scored scenarios create attempt directories. Reset preserves previous
results. These are evaluation records; no rosbag or video is recorded by
default. Exit codes are 0/pass, 1/assertion failure, 2/setup or data error and
3/interrupted. Metric meanings and scoring rules are in the
[scoring reference](../carla_ros_bridge/carla_scenarios/AUTHORING.md#scoring-and-outputs).

## Maps and visualization

See [map and world options](../carla_ros_bridge/carla_scenarios/AUTHORING.md#world-options)
and [autonomy map prerequisites](../carla_ros_bridge/carla_scenarios/AUTHORING.md#parked-vehicle-template)
for assets, matching map pairs and provenance requirements.

The following visualization layer adds the Foxglove layout and simulation-time
propagation into infrastructure and robot TF.

## Shutdown

To stop both the managed session and regular modules from this checkout:

```bash
watod sim stop
watod down all
docker ps
```

The first command stops/removes verified session containers while preserving
maps, results and diagnostics. It leaves a shared network namespace running
while another container references it. The second command stops the standard
module project across profiles. The final command also lists unrelated Docker
projects that these commands do not manage.

## Session recovery

The workspace directory above is automatically generated local state, not an
online cache. Stop the session before deleting it; deleting the whole directory
also deletes its saved results and diagnostic logs. Docker images and maps are
stored separately.

If `session.json` disappears while containers remain, setup and close lose the
ownership record. Session stop verifies Compose project, workspace, services
and results mount before reconstructing ownership for cleanup. Expired SLURM
jobs can leave Docker's running state stale; failed graceful stops fall back
to removal of verified IDs. After cleanup, start a fresh session yourself with
prepared images in a valid allocation. Never edit ownership files in an active
session.

# Simulation Manager

This standalone ROS package manages readiness, playback, evaluation and results.
The executable and node remain `simulation_runner`; the scenario server remains
the sole CARLA tick and `/clock` authority. Enable it with `session_mode:=true`
on `simulation_bringup simulation.launch.yaml`. The default manual workflow
remains in the [simulation guide](../SIMULATION_README.md).

## Commands

`/sim/open` accepts `carla_msgs/srv/SwitchScenario`, selecting
`playground/NAME` or `scenario/NAME`. Trigger services under `/sim/` provide
`ready`, `play`, `pause`, `resume`, `stop`, `reset`, `close` and `status`.
Scenarios open stationary, then complete readiness before play; playgrounds
start immediately and create no evaluated attempt. The following host layer owns
containers and starts an application runtime in each experiment container.

The manager uses wall time for readiness and heartbeat, and CARLA simulation
time for events and scoring. Deliberate pause suspends execution deadlines.
Status snapshots publish on `/sim/status`, driven paths on `/sim/actual_path`,
and speed on `/sim/speed_mps`.

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

Each scored attempt writes `configuration.yaml`, `events.json` and
`result.json` below the configured results directory. The host integration uses
`.cache/watod-sim/results/`. Reset preserves prior attempts. Outcomes distinguish
pass, assertion failure, setup/data error and interruption. Required observations
and movement evidence gate success.

See the [authoring guide](../carla_ros_bridge/carla_scenarios/AUTHORING.md)
for templates, stack constraints, readiness and scoring requirements.

## Shutdown

`/sim/stop` saves an interrupted result while keeping the session available.
`/sim/close` releases the current experiment. Host container shutdown is supplied
by the next stack layer.

## Session recovery

The manager owns ROS experiment state. The following host layer owns container
records, recovery and cleanup. Image builds and live scenario acceptance remain
operator checks.

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
start immediately and create no evaluated attempt. The host CLI and application
process ownership are introduced in the next stack layer.

The manager uses wall time for readiness and heartbeat, and CARLA simulation
time for events and scoring. Deliberate pause suspends execution deadlines.
Status snapshots publish on `/sim/status`, driven paths on `/sim/actual_path`,
and speed on `/sim/speed_mps`.

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

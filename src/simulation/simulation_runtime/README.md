# Simulation runtime

`ros2 run simulation_runtime application_runtime` runs a separate process
supervisor for a managed simulation container. Set `WATOD_SIM_RUNTIME_ROLE` to
`action` or `world` and `WATOD_SIM_SESSION_ID` to the owning session ID. The host
CLI supplies these values.

The runtime accepts the reviewed presets in
[`sim_presets.py`](simulation_runtime/sim_presets.py). It starts existing launch
files with autostart disabled. The simulation stack adapter then calls the
lifecycle manager's group startup service. Runtime stop terminates only its own
launch process group, with bounded signal escalation.

See the [simulation manager](../simulation_manager/README.md#how-the-stack-works)
for ownership, reset ordering and ROS interfaces.

This package provides process ownership and shared launch configuration.
`simulation_manager` orchestrates attempts and evaluation;
`wato_lifecycle_manager` owns the launched nodes' lifecycle transitions.
The runtime can be installed in action and world-modeling images independently
of the simulation manager and CARLA packages.

## Scenario configuration

Version-two scenario data selects trajectory source, world-modeling scope,
actors, goal, completion, and metric limits. The executable groups are inferred;
there is no separate stack or mode field. `assets.py`
validates maneuver and map files under the configured data roots; new filenames
need no registration. `scenario_config.py` validates the schema and metric limits.
`launch_groups()` supplies the same ordered groups to host preflight and the runner.

A fake-planner trajectory launches only the fake planner and controller in the
action container. It may run with no world group, map-only visualization, or the
full world-modeling stack. Normal planner trajectories require full world modeling
and a configured goal. See the [authoring guide](../carla_ros_bridge/carla_scenarios/AUTHORING.md).

## Attempt Topics

The runner and application runtimes share `command_topic()` from `sim_presets.py`
so the bridge's bind request and controller launch remap agree. Raw attempt IDs
remain unchanged in result directories and runtime configuration.

Numeric IDs receive the reserved `_attempt_` prefix in their ROS token. For example,
`20261124T012353_3521a8663d94` uses
`/sim/attempts/_attempt_20261124T012353_3521a8663d94/command`.
Literal IDs beginning with `_attempt_` receive another copy of that prefix to
prevent collisions. Other IDs, including `attempt_2`, retain their topics.
Malformed IDs and topics exceeding Jazzy's full-topic length limit are rejected.
The bridge uses ROS's full-topic validator and returns failed binds as service
responses. Rejected subscription creation preserves the previous binding.

## Building Changes

The simulation, action, and world-modeling images include this package. Rebuild
all three when deploying a runtime or launch-code change, then recreate the
managed session. Scenario YAML, maneuver JSON, map configs and map files are
mounted data; editing them does not require rebuilding:

```bash
watod -m simulation -m action -m world_modeling build
watod sim stop
watod sim setup
```

The runtime moved from `wato_lifecycle_manager`; its executable is now
`ros2 run simulation_runtime application_runtime`. Service names and configuration
fields are unchanged. See the [manager quick start](../simulation_manager/README.md#quick-start)
for image prerequisites and scenario commands.

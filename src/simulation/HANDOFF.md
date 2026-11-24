# Simulation scenario handoff

Branch: `sim/stack-05-visualization`. This commit adds data-driven scenario building
on top of `71ba2078` (runtime packaging, ROS attempt-topic fixes, and setup docs).

## Current design

- Scenario YAML uses version 2; playground YAML stays version 1. Legacy Python
  selectors and scenario aliases were removed intentionally.
- `trajectory` and `world.modeling` infer the application nodes. There is no public
  `stack` or `mode` field. Internal launch profiles remain fixed capabilities.
- A named maneuver uses the fake planner and controller. World modeling can be
  `off`, `map`, or `full`. Normal planning requires full world modeling and a goal.
- YAML configures the map, ego spawn, named vehicle actors, goal, completion,
  selected metrics and limits, deadlines, and supported timed events.
- Managed sessions mount scenario YAML, maneuver JSON, map configs, and maps from
  the checkout. New data filenames need no code registration or image rebuild.
- Close/open reloads data. Reset reloads data when container roles remain the
  same; a changed role set requires close/open. Executable edits need rebuilds.

## Code and documentation

- [Host CLI](../../watod_scripts/watod-sim.py): owns Docker containers and preflight.
- [Runner](simulation_manager/simulation_manager/simulation_runner.py): world
  preparation, readiness, attempt execution, completion, and saved metrics.
- [Shared runtime](simulation_runtime/simulation_runtime/): schema normalization,
  asset validation, supported launches, and application process ownership.
- [World loader](carla_ros_bridge/carla_scenarios/carla_scenarios/worlds/configured_world.py):
  CARLA map loading, vehicle spawning, and actor/goal geometry.
- [Authoring guide](carla_ros_bridge/carla_scenarios/AUTHORING.md) and
  [manager commands](simulation_manager/README.md) describe the public interface.

## Spawn and deployment details

- Explicit ego coordinates belong under `world.ego.spawn.pose`, in the ROS map
  frame. The loader converts y and yaw to CARLA coordinates once.
- Lane-relative actors/goals require a lane-based ego spawn. Explicit map poses
  do not establish that lane waypoint.
- Preserve the user's tuned `brake_in_curve.yaml` spawn. The earlier position
  problem was a missing `pose:` wrapper, which the user corrected.
- `seed` currently seeds Python random choices; it does not guarantee deterministic
  CARLA physics, traffic-manager behavior, or pedestrian navigation.
- `sim_session` uses the image built by `simulation_bringup`; they are different
  Compose services. `setup` neither builds nor pulls images and may reuse a session.
- A missing `simulation_runtime` import was verified in an old November 22 image.
  Reconnecting does not update baked code. A running container can also retain an
  older image after its local tag is rebuilt. Compare image IDs when diagnosing.

```bash
watod -m simulation -m action -m world_modeling build
watod sim stop
watod sim setup
watod sim open scenario brake_in_curve
watod sim play
```

## Validation and remaining work

- Final source tests: **56 passed** in an isolated ROS 2 Jazzy Docker container.
  Host CLI tests: **14 passed**. No active session was reset by these checks.
- Python lint/format, Markdown, JSON, XML, and whitespace checks passed. YAML lint
  reported only existing comment-indentation warnings in the Compose file.
- New tests discover every playground, scenario, and maneuver, validate their
  data and referenced launch assets, and reject malformed schema/asset inputs.
  Per the user, avoid new tests that freeze individual scenario behavior or poses.
- Existing infrastructure and numeric attempt-ID regressions remain. The extra
  missing-`pose` guard and its specific tests were removed at the user's request.
- Full CARLA driving/scoring acceptance is still unverified by this agent. The
  user has been running scenarios, but that is not a complete acceptance run.
- `brake_in_curve` and `sine_with_dwell` JSON use `ramp_up_accel: 0`; the fake-planner
  node's timed release ramp handles startup. These remain authored maneuvers,
  not road-following routes.
- Prompt-based generation and arbitrary custom managed launch files are not
  implemented. Playgrounds support manually launched application nodes.
- Results are in `.cache/watod-sim/results/`. Stop preserves them; no dedicated
  `watod sim clean` command exists.
- Unrelated untracked `.simforge/` files are intentionally excluded. This commit
  is local; an earlier push failed because GitHub SSH authentication was unavailable.

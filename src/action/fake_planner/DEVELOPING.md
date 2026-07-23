# Developing fake_planner

## Build & Launch

```bash
colcon build --packages-select fake_planner
ros2 launch fake_planner fake_planner_sim.launch.yaml
```

The maneuver JSON is parsed with `nlohmann_json` at configure time; a bad file fails the lifecycle transition with the parse error in the node log rather than crashing.

## Architecture

Two pieces, split so the maneuver math can be tested without a running graph:

- **`FakePlannerNode`** (`fake_planner_node`) — the ROS lifecycle node. Owns parameters, the publishers/subscriber, the services, the publish timer, and the odom-driven anchoring and respawn logic.
- **`FakePlannerCore`** (`fake_planner_core`) — all maneuver math, holds no ROS state.

The core turns a maneuver into a published window in four steps:

1. **`loadManeuver`** — parse the JSON, walk the segments as a moving cursor sampling a fine polyline, resample to uniform `sample_spacing_m`, then (open maneuvers only) append the stop pad. Result: the maneuver in local coordinates.
2. **`anchor(x, y, yaw)`** — apply one SE(2) transform to every waypoint, building the full trajectory in `frame_id`. Point headings are taken from consecutive points.
3. **`updateWindow(veh_x, veh_y)`** — on each odom, find the nearest point (searching forward from the last match so a self-crossing path doesn't snap back a lap) and slice `trail_m` behind through `horizon_m` ahead.
4. **`window()`** — the slice the node restamps and publishes.

## Path construction

Segments are built in a local frame (`u` forward, `v` left) that continues from the previous segment's end, then transformed to world coordinates:

- **straight / dwell** — advance `u`, hold `v`.
- **shift** — raised-cosine lateral move to `lateral`, tangent to the heading at both ends.
- **slalom** — sine weave, amplitude tapered over the first and last half wavelength so it joins the centreline tangentially; the middle is a pure sine (the steady-state region a frequency-response test cares about).
- **arc** — constant-radius turn; also rotates the heading by `angle`.

Everything is sampled at `min(0.05 m, spacing)`, then `resampleUniform` re-lays it at exactly `sample_spacing_m`. Uniform spacing is what makes the output indistinguishable from a real planner's — the extended comment in `fake_planner_core.cpp` explains why spacing tracks the lanelet centreline node spacing, not any planner setting. Speed ramps `speed`→`end_speed` within a segment and carries across segments.

## Anchoring and respawns

With `anchor_to_first_pose` true (default), the node waits for the first odom and anchors the maneuver at that pose, so a relative maneuver starts at the car wherever it is. With it false, the core anchors at the origin during configure and publishes the file's coordinates verbatim — used by maneuvers that carry an absolute `start` (e.g. the oval).

`respawn_jump_m` guards against a stale anchor: a pose that jumps more than this between two consecutive odom messages is treated as a teleport (in sim, an ego respawn), not driving. The node rewinds, re-arms `start_on_activate`, and — when anchoring — re-anchors at the new pose on the next odom, so "respawn ego" in Foxglove is a one-click reset. Set 0 to disable; the vehicle launch does, because a localization correction of that size on the real car is something to drive through, not re-lay the maneuver on. The `have_pose_ && anchored_` guard keeps it from firing on the first message, when there is no previous pose to compare against.

## Rolling window and stops

The window is `trail_m` of path behind the vehicle through `horizon_m` ahead, re-sliced every odom. The trail keeps pure pursuit from creeping — it steers only to points ahead of it and needs some path behind. The horizon matches the real stack's ~35 m rather than dumping the whole route.

Open maneuvers get a **stop pad**: a backward pass brakes the speed profile to zero into the last authored point at 1.0 m/s² (matching `trajectory_planner`'s `max_tangential_accel`), then a 10 m run of zero-speed points extends past the stop line so pure pursuit still has path to hold on after the car stops. Closed maneuvers skip this; the window wraps at the seam and the car laps.

## Adding a maneuver

1. Drop a JSON under `maneuvers/` (schema in the [README](README.md)). Rebuild — `install/` serves the file from `share/`, so an un-built file won't be found.
2. Run it: `maneuver:=<name>`.
3. If it uses an absolute `start`, launch with `anchor_to_first_pose:=false`.

## Adding a segment type

1. Add a `build*` primitive in the anonymous namespace of `fake_planner_core.cpp`. It appends samples for `t` in `(0, L]` and moves the cursor to the segment end; the caller seeds the first point and the speed ramp.
2. Add a branch in the `loadManeuverJson` dispatch, pulling params off the JSON with `p.at(...)` (throwing on a missing field is caught and reported per-segment).
3. Add the type to the maneuver schema table in the README.

## After Launching

```bash
# Node reaches active
ros2 lifecycle get /action/fake_planner/fake_planner_node        # expect: active

# Trajectory publishing at publish_rate_hz
ros2 topic hz /action/fake_planner/trajectory                    # expect: ~10 Hz
```

Watch the path in Foxglove on `/action/fake_planner/trajectory_markers` — it publishes even while held in standby, so the maneuver can be inspected before release.

If the node logs `Waiting for first odom on '<topic>' to anchor trajectory...` and never publishes, no odometry is reaching it — check the `odom_topic` remap and that the source is up. The node needs odom to anchor, so nothing goes out until the first message arrives.

## Definition of Good Result

| Check | Expected |
|-------|----------|
| Lifecycle state | `active` |
| `/action/fake_planner/trajectory` rate | ≈ `publish_rate_hz` (10 Hz) |
| Markers in Foxglove | A window of purple spheres around the car, sliding as it drives |
| Open maneuver end | Car brakes smoothly to a stop at the last waypoint, no drift off the path |
| Closed maneuver | Car laps continuously through the seam |

## Troubleshooting

- **Controller doesn't move though the trajectory publishes** — in CARLA this is almost always `use_sim_time`: without it, trajectory stamps on wall clock can't transform against the sim-time TF tree and the controller aborts every cycle silently. Also check `standby_speed:=0.0` — the controllers' own `-0.5` default reverses the car in standby. Both wrappers set these correctly.
- **Path lands off the oval track** — `oval_track` is in absolute coordinates and must run with `anchor_to_first_pose:=false`. Anchoring a 400 m closed lap throws the far side off the road with a fraction of a degree of spawn yaw.

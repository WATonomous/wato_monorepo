# Developing fake_planner

## Build & Launch

```bash
colcon build --packages-select fake_planner
ros2 launch fake_planner fake_planner_sim.launch.yaml
# comes up held in standby -- nothing moves until you release it
ros2 service call /action/fake_planner/fake_planner_node/start_trajectory std_srvs/srv/Trigger
```

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

When anchoring is on, the node waits for the first odom and anchors the maneuver at that pose, so a relative maneuver starts at the car wherever it is. When it is off, the core anchors at the origin during configure and publishes the file's coordinates verbatim — what maneuvers carrying an absolute `start` (e.g. the oval) need.

Which one you get is the maneuver's own business, not the launch's: `anchoring` defaults to `auto`, and `on_configure` resolves it from `core_->hasAbsoluteStart()` *after* the JSON is loaded — a file with a `start` publishes verbatim, everything else anchors. This mirrors `closed`, the other file-level fact that changes runtime behaviour, and it means a maneuver can't be run the wrong way by forgetting a flag. `anchoring:=relative|absolute` forces it; forcing `relative` on a maneuver with a `start` is legal (replay the oval's shape from wherever the car is parked) and logs a warning, since on the real track it is always a mistake. The override values are named after the two kinds of maneuver rather than `on`/`off` because YAML reads those two as booleans, and a bool override on a string parameter aborts the node at construction.

`respawn_jump_m` guards against a stale anchor: a pose that jumps more than this between two consecutive odom messages is treated as a teleport (in sim, an ego respawn), not driving. The node rewinds, re-arms `start_on_activate`, and — when anchoring — re-anchors at the new pose on the next odom, so "respawn ego" in Foxglove is a one-click reset. Set 0 to disable; the vehicle launch does, because a localization correction of that size on the real car is something to drive through, not re-lay the maneuver on. The `have_pose_ && anchored_` guard keeps it from firing on the first message, when there is no previous pose to compare against.

## Rolling window and stops

The window is `trail_m` of path behind the vehicle through `horizon_m` ahead, re-sliced every odom. The trail keeps pure pursuit from creeping — it steers only to points ahead of it and needs some path behind. The horizon matches the real stack's ~35 m rather than dumping the whole route.

Open maneuvers get a **launch ramp** and a **stop pad**, one at each end, and both are speed-profile passes only — no geometry is added at the start, and the pad's points are all past the stop line.

The **launch ramp** pins the first waypoint at zero speed and propagates that forwards at `ramp_up_accel` (default 1.0 m/s², the mirror of the stop pad's decel), so the profile asks the car to pull away gently instead of commanding the full `default_speed` from the first point. The distance it consumes is `v²/2a` — 4.5 m at the shipped 3.0 m/s and 1.0 m/s² — and every shipped maneuver opens with a straight longer than that, so acceleration is finished before any curvature. That fit is the thing to re-check when editing a maneuver: raise its speed or soften its accel and the ramp grows, and past the opening straight's length the car is still gaining speed when the steering starts. `ramp_up_accel: 0` disables the ramp for a maneuver that wants to start at speed.

Note the ramp does not stop the *controller* from lurching, only the trajectory from asking it to: pure pursuit reads `max_speed` from the point `speed_lookahead_distance` (1.0 m) ahead of the car, so at launch it sees the ramp's value there (1.4 m/s at 1.0 m/s²) rather than zero. That is also why the ramp starting at exactly zero doesn't deadlock the car at the start line.

The **stop pad** is the same idea at the far end: a backward pass brakes the speed profile to zero into the last authored point at 1.0 m/s² (matching `trajectory_planner`'s `max_tangential_accel`), then a 10 m run of zero-speed points extends past the stop line so pure pursuit still has path to hold on after the car stops. It runs *after* the ramp, so on a maneuver too short to both reach speed and brake back down its backward pass simply lowers the ramp's peak — the lower of the two envelopes, which is the right answer.

Closed maneuvers skip both. The window wraps at the seam, so there is no end to stop at and no start to pull away from; a ramp there would make the car brake to a crawl once a lap.

## Adding a maneuver

1. Drop a JSON under `maneuvers/` (schema in the [README](README.md)). Rebuild — `install/` serves the file from `share/`, so an un-built file won't be found.
2. Run it: `maneuver:=<name>`. Nothing else to pass — a file with an absolute `start` turns anchoring off for itself.

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

- **Nothing publishes on `trajectory` and the car sits still** — expected on bringup. Every launch sets `start_on_activate:=false`, so the node logs `Holding: waiting for the start_trajectory service.` every 5 s until it is released. Call `start_trajectory` (or launch with `start_on_activate:=true` in sim). A respawned ego puts it back in the hold.
- **Controller doesn't move though the trajectory publishes** — in CARLA this is almost always `use_sim_time`: without it, trajectory stamps on wall clock can't transform against the sim-time TF tree and the controller aborts every cycle silently. Also check `standby_speed:=0.0` — the controllers' own `-0.5` default reverses the car in standby. Both wrappers set these correctly.
- **Path lands off the oval track** — `oval_track` is in absolute coordinates and must be published verbatim; anchoring a 420 m closed lap throws the far side off the road with a fraction of a degree of spawn yaw. `auto` handles this, so check the configure log for `anchor=false (auto: absolute 'start')` and that nothing passed `anchoring:=relative` (which warns).

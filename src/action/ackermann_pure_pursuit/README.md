# Ackermann Pure Pursuit

ROS2 lifecycle node that tracks a trajectory using the pure pursuit algorithm, publishing Ackermann drive commands for vehicle steering and speed control.

## Overview

The pure pursuit node receives a planned trajectory, transforms each waypoint into the vehicle's **rear-axle reference frame** (once per cycle), selects a lookahead point, and computes the steering angle and speed via the pure pursuit geometric algorithm. The lookahead distance is adapted to speed and path curvature, speed is limited by a lateral-acceleration budget, and the steering/speed commands are rate-limited for smoothness. Each cycle it measures cross-track and heading error and, if either exceeds a tunable threshold for a sustained window, **raises a disengage recommendation** in its `ControllerStatus` telemetry. This is advisory only: the controller keeps tracking normally and does not alter its command, yield the mux, or talk to the OSCC layer — acting on the recommendation is left to a higher-level supervisor. It publishes an idle signal and standby Ackermann commands when no valid trajectory is available, the trajectory has gone stale, or the behaviour tree requests standby.

**Current Status**: Adaptive pure pursuit controller with rear-axle geometry, TF-based wheelbase measurement, model-aware variable lookahead, an optional multi-look-ahead steering blend, curvature/accel-limited speed control, command-rate limiting, an optional low-speed Stanley cross-track blend, a debounced disengage monitor, and live-reconfigurable parameters.

## ROS Interface

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `trajectory` | `wato_trajectory_msgs/Trajectory` | Planned trajectory from the freeroam planner |
| `execute_behaviour` | `behaviour_msgs/ExecuteBehaviour` | Requested behaviour from the behaviour tree |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/action/ackermann` | `ackermann_msgs/AckermannDriveStamped` | Steering angle and speed command |
| `/action/is_idle` | `std_msgs/Bool` | `true` when idle or in standby (the disengage monitor does **not** raise this) |
| `controller_status` | `wato_trajectory_msgs/ControllerStatus` | Per-cycle tracking error, effective lookahead, path curvature, per-sample multi-look-ahead diagnostics, commands, and disengage state/reason |

Also subscribes to `odom` (`nav_msgs/Odometry`) for the current longitudinal speed used by the adaptive lookahead and speed scheduling.

## Configuration

Parameters are loaded from `config/params.yaml` under the namespace `action/pure_pursuit_node/ros__parameters`.

Parameters are read once in `on_configure`. To change them at runtime, transition the lifecycle node back through `cleanup` → `configure` (which reloads `config/params.yaml`), as with the other action nodes.

**Topics / frames**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `controller_status_topic` | string | `"controller_status"` | Topic for the `ControllerStatus` telemetry |
| `base_frame` | string | `"base_footprint"` | Frame stamped on published Ackermann commands |
| `rear_axle_frame` | string | `"rear_axle"` | Frame the pure-pursuit geometry and tracking error are computed in; also one end of the TF wheelbase measurement |
| `front_axle_frame` | string | `"front_axle"` | Other end of the TF wheelbase measurement |
| `standby_msg` | string | `"standby"` | Behaviour string that triggers standby output |

**Adaptive lookahead**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `lookahead_distance` | double | `5.0` | Maximum lookahead distance (m) |
| `min_lookahead_distance` | double | `2.0` | Minimum lookahead distance (m) |
| `lookahead_time` | double | `1.5` | Time-headway: `ld = lookahead_time · speed`, clamped to min/max (s) |
| `curvature_lookahead_gain` | double | `2.0` | Shrinks lookahead in curves: `ld_eff = ld / (1 + gain·|κ|)` |
| `speed_lookahead_distance` | double | `1.0` | Minimum distance ahead used to sample target speed (m) |
| `speed_lookahead_time` | double | `2.0` | Time-headway for the (decoupled, longer) speed sample horizon (s) |

**Multi-look-ahead blend (MLPP)**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `enable_multi_lookahead` | bool | `false` | Blend several lookahead samples into one steering curvature; `false` keeps the single-lookahead law |
| `lookahead_weights` | double[] | `[0.5, 0.3, 0.2]` | Blend weights ordered near → far. The length sets how many samples are taken; only the ratios matter |

**Steering & speed**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `steering_angle_gain` | double | `1.0` | Gain on the pure-pursuit steering output |
| `max_steering_angle` | double | `0.5` | Maximum steering magnitude (rad) |
| `max_steering_rate` | double | `1.0` | Steering slew-rate limit (rad/s) |
| `max_speed` / `min_speed` | double | `5.0` / `0.0` | Commanded speed bounds while tracking (m/s) |
| `max_lateral_accel` | double | `2.5` | Cornering-speed budget: `v ≤ sqrt(a_lat/|κ|)` (m/s²) |
| `max_accel` / `max_decel` | double | `1.5` / `3.0` | Command accel/decel slew limits (m/s²) |

**Low-speed cross-track blend (Stanley)**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `enable_stanley_blend` | bool | `false` | Blend a cross-track feedback term into steering at low speed |
| `stanley_gain` | double | `0.5` | Cross-track feedback gain `k_e` |
| `stanley_softening` | double | `1.0` | Softening constant `k_soft` (avoids blow-up near zero speed) |
| `stanley_speed_threshold` | double | `2.0` | Blend weight ramps from 1 (at 0) to 0 at this speed (m/s) |

**Disengage monitor**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `enable_disengage` | bool | `true` | Enable the tracking-threshold disengage monitor |
| `max_cross_track_error` | double | `1.5` | Cross-track error that (sustained) trips a disengage (m) |
| `max_heading_error` | double | `0.6` | Heading error that (sustained) trips a disengage (rad) |
| `disengage_debounce_sec` | double | `0.5` | How long the error must be exceeded before tripping (s) |
| `disengage_latch` | bool | `false` | `true` = stay disengaged until reconfigured; `false` = auto-recover |
| `disengage_speed` | double | `0.0` | Safe speed commanded when no valid lookahead point exists (m/s) |

**Standby / misc**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `standby_speed` | double | `-0.5` | Speed commanded during standby (m/s) |
| `standby_steering` | double | `0.0` | Steering commanded during standby (rad) |
| `control_rate_hz` | double | `20.0` | Control publish rate (Hz) |
| `wheelbase_fallback` | double | `2.5667` | Fallback wheelbase if the TF lookup fails (m) |
| `idle_timeout_sec` | double | `2.0` | Time since last trajectory before declaring idle (s) |
| `invert_steering` | bool | `true` | Invert steering sign — `true` means positive angle = clockwise |
| `disable_standby` | bool | `false` | Bypass standby and track even when `standby_msg` is received |

## Algorithm Details

### Control Flow

1. **Idle / standby** — if no/empty/stale trajectory, empty behaviour string, or (unless `disable_standby`) the behaviour equals `standby_msg`, publish `is_idle = true`, a standby Ackermann command, and a `ControllerStatus` with `reason = "idle"`; the command-shaping state is reset so re-engagement is smooth.
2. **Track** — otherwise transform the trajectory into `rear_axle_frame` (one TF lookup) and run the pipeline below.

### Rear-axle geometry

Trajectory points are transformed into `rear_axle_frame` (default `rear_axle`), the textbook reference point for pure pursuit — the rear axle traces the arc to the lookahead point. Measuring from `base_footprint` would add the base-to-axle offset and bias curvature, worst in tight turns.

### Tracking error

Each cycle the vehicle (origin, heading +x) is projected onto the nearest **segment** of the path (not merely the nearest vertex) to compute a signed **cross-track error** (left of path positive) and **heading error** (path tangent angle in the vehicle frame). These drive the disengage monitor and the optional Stanley blend, and are published in `ControllerStatus`.

### Adaptive lookahead

The lookahead distance combines three effects:

1. **Time-headway schedule** — `ld = lookahead_time · speed`, clamped to `[min_lookahead_distance, lookahead_distance]`. A short `ld` gives high-gain, oscillatory steering; a long `ld` cuts corners. Scheduling on speed keeps a consistent *reaction time*; the min clamp preserves low-speed stability, the max clamp caps corner-cutting.
2. **Curvature shrink** — `ld_eff = ld / (1 + curvature_lookahead_gain·|κ_ahead|)`, where `κ_ahead` is estimated from the path near the lookahead. This tightens tracking through curves (where long lookahead cuts corners) while relaxing to a long, smooth lookahead on straights.
3. **Interpolation** — the lookahead point is the exact intersection of the path with the circle of radius `ld_eff`, avoiding steering chatter from snapping between sparse waypoints.

Steering is then the pure-pursuit law about the rear axle:

$$\delta = k_\delta \cdot \arctan\left(L \cdot \frac{2y}{x^2 + y^2}\right)$$

where $L$ is the wheelbase, $(x, y)$ is the lookahead point in `rear_axle_frame`, and $k_\delta$ is `steering_angle_gain`. Steering is clamped to `±max_steering_angle` and slew-limited by `max_steering_rate`.

### Multi-look-ahead blend (MLPP)

A single lookahead distance has to serve two conflicting jobs: short `ld` tracks tightly but oscillates, long `ld` is smooth but cuts corners and cannot begin turning in for a curve it can already see. The adaptive schedule above picks one scalar compromise between those failure modes; with `enable_multi_lookahead` the controller instead samples *several* points across the horizon and blends them.

`lookahead_weights.size()` samples are placed evenly across `[min_lookahead_distance, ld_eff]` in a single forward sweep of the path, and their pure-pursuit curvatures are combined into the commanded curvature:

$$\kappa_{cmd} = \frac{\sum_i w_i \cdot \kappa_{pp}(p_i)}{\sum_i w_i}, \qquad \delta = k_\delta \cdot \arctan(L \cdot \kappa_{cmd})$$

Near samples supply **tracking authority** (they drive cross-track error to zero), far samples supply **anticipation** (they start the turn-in before the curve arrives and damp the near-sample gain). Because it is a weighted mean rather than a hard choice, the command does not jump when the scheduled lookahead crosses a regime boundary.

The blend replaces only the aggregation step. The adaptive schedule still sets the horizon, so the samples tighten in curves and stretch at speed exactly as the single lookahead does, and the Stanley blend, steering clamp, slew limit, and speed control downstream are unchanged. Only the ratios of the weights matter — they are normalised — and samples that fall past the end of the path clamp to the final path point rather than dropping out.

With one weight the blend reduces exactly to the single-lookahead law, so `enable_multi_lookahead: false` and a one-element `lookahead_weights` are both faithful fallbacks. Each sample's reached distance, curvature, and weight are published on `ControllerStatus` so a blend can be reconstructed from a bag while tuning.

### Speed control

Target speed is the minimum of the path's requested speed (sampled at a decoupled, speed-scaled horizon `max(speed_lookahead_distance, speed_lookahead_time·speed)`) and the curvature-limited speed `sqrt(max_lateral_accel / |κ|)` — so the vehicle slows *before* a curve within a lateral-acceleration budget rather than reacting to steering after the fact. The result is clamped to `[min_speed, max_speed]` (or `0` if the path commands a stop) and then accel/decel slew-limited by `max_accel` / `max_decel`.

### Optional low-speed blend

When `enable_stanley_blend` is set, a Stanley-style cross-track correction `-atan(k_e·e / (k_soft + |v|))` is added to the steering, weighted from 1 at standstill down to 0 at `stanley_speed_threshold`. This directly regulates cross-track error where pure pursuit's effective gain is weakest.

### Disengage monitor

If cross-track or heading error exceeds its threshold continuously for `disengage_debounce_sec`, the monitor raises a disengage **recommendation**: it publishes `ControllerStatus.disengaged = true` with the tripping `reason`. This is **advisory only** — the controller keeps tracking normally and does *not* alter its command, raise `is_idle`, yield to the [ackermann_mux](../../interfacing/ackermann_mux), or talk to the OSCC layer. Acting on the recommendation (driver takeover, OSCC disable, mux handoff) is left to a higher-level supervisor that consumes the flag. The debounce rejects single-cycle spikes from TF/localization jitter. With `disengage_latch = false` the flag clears when error returns within bounds; with `true` it stays raised until reconfigured.

### Wheelbase Measurement

Wheelbase is resolved at runtime via a TF lookup between `rear_axle_frame` and `front_axle_frame`. On failure, `wheelbase_fallback` is used. The result is cached for all subsequent control cycles.

## Dependencies

- ROS 2 (tested on Humble)
- `tf2_ros`, `tf2_geometry_msgs`
- `ackermann_msgs`, `std_msgs`, `geometry_msgs`
- `wato_trajectory_msgs` (custom trajectory message)
- `behaviour_msgs` (custom behaviour message)
- `rclcpp_lifecycle`

## License

Copyright (c) 2025-present WATonomous. All rights reserved.
Licensed under the [Apache License, Version 2.0](http://www.apache.org/licenses/LICENSE-2.0).

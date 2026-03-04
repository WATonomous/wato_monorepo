# Ackermann Pure Pursuit

ROS2 lifecycle node that tracks a trajectory using the pure pursuit algorithm, publishing Ackermann drive commands for vehicle steering and speed control.

## Overview

The pure pursuit node receives a planned trajectory, transforms each waypoint into the vehicle's base frame, selects a lookahead point, and computes the steering angle and speed via the pure pursuit geometric algorithm. It also publishes an idle signal when no valid trajectory is available or the trajectory has gone stale.

**Current Status**: Functional pure pursuit controller with TF-based wheelbase measurement and speed scaling proportional to steering magnitude.

## ROS Interface

### Subscribed Topics

| Topic | Remapped From | Type | Description |
|-------|--------------|------|-------------|
| `trajectory` | `/action/freeroam_planner/trajectory` | `wato_trajectory_msgs/Trajectory` | Planned trajectory from the freeroam planner |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/action/ackermann` | `ackermann_msgs/AckermannDriveStamped` | Steering angle and speed command |
| `/action/is_idle` | `std_msgs/Bool` | `true` when no valid trajectory is being tracked |

## Configuration

Parameters are loaded from `config/params.yaml` under the namespace `action/pure_pursuit/pure_pursuit_node/ros__parameters`.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `base_frame` | string | `"base_footprint"` | Robot base frame for trajectory transforms |
| `rear_axle_frame` | string | `"rear_axle"` | Rear axle TF frame for wheelbase measurement |
| `front_axle_frame` | string | `"front_axle"` | Front axle TF frame for wheelbase measurement |
| `lookahead_distance` | double | `3.5` | Target lookahead distance for pure pursuit (m) |
| `min_lookahead_distance` | double | `2.0` | Minimum accepted lookahead distance (m) |
| `max_speed` | double | `5.0` | Maximum commanded speed (m/s) |
| `min_speed` | double | `0.5` | Minimum commanded speed while tracking (m/s) |
| `control_rate_hz` | double | `20.0` | Frequency at which control commands are published (Hz) |
| `wheelbase_fallback` | double | `2.5667` | Fallback wheelbase if TF lookup fails (m) |
| `max_steering_angle` | double | `0.5` | Maximum steering angle magnitude (rad) |
| `idle_timeout_sec` | double | `2.0` | Time since last trajectory before declaring idle (s) |
| `invert_steering_` | bool | `true` | Invert the steering angle sign if needed [true - positive cw] |

## Algorithm Details

### Pure Pursuit
The node transforms trajectory waypoints into `base_frame` and selects the first point ahead of the vehicle (`x > 0`) that is at least `min_lookahead_distance` away and at least `lookahead_distance` from the vehicle, or the last point if none qualify.

Steering and speed are then computed as:

$$\delta = \arctan\left(\frac{L \cdot 2y}{x^2 + y^2}\right)$$

where \(L\) is the wheelbase, and \((x, y)\) is the lookahead point in `base_frame`.

### Speed Scaling
Speed is reduced proportionally to steering demand:

$$v = v_{\text{target}} \cdot \left(1 - 0.5 \cdot \frac{|\delta|}{\delta_{\max}}\right)$$

and clamped between `min_speed` and `max_speed`. If the target trajectory point has `max_speed ≤ 0`, the vehicle is commanded to stop.

### Wheelbase Measurement
Wheelbase is resolved at runtime via a TF lookup between `rear_axle_frame` and `front_axle_frame`. On failure, `wheelbase_fallback` is used and the result is cached for subsequent control cycles.

## Dependencies

- ROS 2 (tested on Humble)
- `tf2_ros`, `tf2_geometry_msgs`
- `ackermann_msgs`, `std_msgs`, `geometry_msgs`
- `wato_trajectory_msgs` (custom trajectory message)
- `rclcpp_lifecycle`

## License

Copyright (c) 2025-present WATonomous. All rights reserved.
Licensed under the [Apache License, Version 2.0](http://www.apache.org/licenses/LICENSE-2.0).

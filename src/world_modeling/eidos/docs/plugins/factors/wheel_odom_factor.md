# WheelOdomFactor

**Class:** `eidos::WheelOdomFactor`
**Type:** Factor (latching via `latchFactor`)
**XML:** `factor_plugins.xml`

Wheel odometry plugin. Combines rear-axle wheel speed from `sensor_msgs/JointState` wheel joint velocities with the IMU yaw rate, and publishes `nav_msgs/Odometry` with a per-reading twist covariance for `eidos_transform` to fuse. When `add_factors` is true, it also attaches a planar `BetweenFactor<Pose3>` between consecutive states created by other plugins.

The plugin only consumes sensor topics (joint states, IMU) and never reads other plugins' outputs.

## Kinematics

1. **Axle speed:** `v = wheel_radius * (w_left + w_right) / 2`. The rear wheels are not steered, so no steering angle is needed. Speeds are clamped to `>= 0` (forward only; see Limitations).
2. **Yaw rate:** the latest IMU gyro, rotated into the base frame (extrinsic from TF, cached on first use).
3. **Lever arm:** the axle velocity is transferred to the base frame with `v_base = v_axle - omega x p_axle`, where `p_axle` is the `axle_frame` position in the base frame (from TF). With the rear axle behind `base_footprint`, turning produces lateral velocity at the base origin (`yaw_rate * distance`), so the nonholonomic constraint is correct at the base frame.
4. **Covariance:** forward speed std-dev is `speed_std_moving` while the wheels report motion and `zero_speed_std` when they read zero. ABS sensors report zero below a cutoff speed, so a zero reading cannot distinguish "stopped" from "creeping".

Pure math lives in `include/eidos/utils/wheel_kinematics.hpp` (ROS-free, unit-tested in `test/test_wheel_kinematics.cpp`).

## Odometry Output

Published on `odom_topic` for every joint state message (after TF and IMU are available):
- **Twist:** base-frame `linear.x`, `linear.y`, and `angular.z`, with the covariance described above. This is the part meant for fusion.
- **Pose:** dead-reckoned planar pose in the odom frame, for visualization and `setOdomPose()`. Its covariance is set very large; it is not meant to be fused.

Fuse it in `eidos_transform` as an odom source with `use_msg_covariance: true` and `twist_mask: [false, false, false, true, true, true]` (yaw rate already comes from the IMU source).

## Graph Factors

On `latchFactor(key, t)`, samples in `(last_key_time, t]` are integrated with a zero-order hold into a planar delta. Noise (GTSAM order `[roll, pitch, yaw, x, y, z]`):
- `x`, `y`: `trans_std_per_meter * distance + zero_speed_std * time_at_zero_speed`, floored at `min_std`.
- `yaw`: `yaw_std_per_rad * |delta_yaw|`, floored at `min_std`.
- `z`, `roll`, `pitch`: `loose_std` (wheels say nothing about them).

If no wheel samples fall in the interval, no factor is added (rather than asserting zero motion).

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `joint_states_topic` | string | `"wheel_joint_states"` | Input `sensor_msgs/JointState` topic. |
| `left_joint` / `right_joint` | string | `"rear_left_joint"` / `"rear_right_joint"` | Joint names of the rear wheels. |
| `wheel_radius` | double | `0.31235` | Effective rolling radius (m). Also the tire-scale calibration knob. |
| `axle_frame` | string | `"rear_axle"` | TF frame at the axle centre (lever arm). |
| `imu_topic` | string | `"/imu/data"` | IMU topic for yaw rate. |
| `imu_frame` | string | `"imu_link"` | TF frame of the IMU. |
| `odom_topic` | string | `"<name>/odometry"` | Published odometry topic. |
| `add_factors` | bool | `false` | Whether to add BetweenFactors to the graph. |
| `max_dt` | double | `0.5` | Integration gaps longer than this (s) are skipped. |
| `speed_std_moving` | double | `0.05` | Forward speed std-dev when moving (m/s). |
| `zero_speed_std` | double | `0.4` | Forward speed std-dev at zero reading (m/s). Also the creep rate used in factor noise. |
| `lateral_std` | double | `0.05` | Lateral (nonholonomic) std-dev (m/s). |
| `vertical_std` | double | `0.05` | Vertical std-dev (m/s). |
| `yaw_rate_std` | double | `0.01` | Yaw rate std-dev (rad/s). |
| `factor.trans_std_per_meter` | double | `0.02` | Factor translation std-dev growth per metre. |
| `factor.yaw_std_per_rad` | double | `0.05` | Factor yaw std-dev growth per radian. |
| `factor.min_std` | double | `0.01` | Floor on constrained factor std-devs. |
| `factor.loose_std` | double | `10.0` | Factor std-dev for z, roll, pitch. |

## Notes

- `isReady()` always returns true, so missing wheel data never blocks `WARMING_UP`.
- The plugin waits (with a throttled warning) until both the IMU and axle extrinsics are available from TF and an IMU message has arrived.

## Limitations

- **Direction:** wheel speeds on the Kia Soul EV CAN bus are unsigned. Reverse is reported as forward motion until gear state is decoded.
- **Low-speed cutoff:** the ABS cutoff speed has not been measured on the vehicle; tune `zero_speed_std` from a slow roll-off test.

# ImuFactor

**Class:** `eidos::ImuFactor`
**Type:** Factor (latching via `latchFactor`)
**XML:** `factor_plugins.xml`

IMU preintegration factor plugin. Subscribes to IMU, performs GTSAM preintegration, and produces `BetweenFactor<Pose3>` constraints between consecutive states. When another plugin (e.g. LISO) creates a state, `latchFactor()` drains the IMU buffer over the time interval and attaches an IMU-derived relative pose constraint.

Also publishes an odometry topic and writes an odom-frame pose via `setOdomPose()` for lock-free pose sharing within eidos and consumption by `eidos_transform`.

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `imu_topic` | string | `"/imu/data"` | Input IMU topic (`sensor_msgs/Imu`). |
| `imu_frame` | string | `"imu_link"` | TF frame of the IMU sensor. |
| `add_factors` | bool | `true` | Whether to inject IMU BetweenFactors into the graph. Set to `false` in localization mode to use IMU only for odom publishing. |
| `odom_topic` | string | `"imu_factor/odometry"` | Published odometry topic (`nav_msgs/Odometry`). Contains preintegrated pose and body-frame twist. |
| `acc_cov` | double[3] | `[9e-6, 9e-6, 9e-6]` | Accelerometer noise covariance (diagonal). |
| `gyr_cov` | double[3] | `[1e-6, 1e-6, 1e-6]` | Gyroscope noise covariance (diagonal). |
| `integration_cov` | double[3] | `[1e-4, 1e-4, 1e-4]` | Integration uncertainty covariance (diagonal). |
| `gravity` | double | `9.80511` | Gravity magnitude (m/s^2). |
| `default_imu_dt` | double | `0.002` | Default IMU dt (seconds) used for the first message before a dt can be computed. |
| `initialization.warmup_samples` | int | `200` | Number of consecutive stationary IMU samples required before warmup is complete. |
| `initialization.stationary_gyr_threshold` | double | `0.005` | Gyroscope magnitude threshold (rad/s) for stationary detection during warmup. If gyro exceeds this, the warmup counter resets. |

## Warmup and `isReady()`

ImuFactor overrides `isReady()` to return `false` until warmup is complete. This gates the `InitSequencer` state machine: the system stays in `WARMING_UP` until **all** factor plugins report `isReady() == true`.

Warmup performs stationary detection and gravity alignment:

1. Each IMU callback checks if gyroscope magnitude is below `stationary_gyr_threshold`.
2. If stationary and the IMU provides orientation, the quaternion is accumulated.
3. If gyro exceeds the threshold, the counter and accumulator reset to zero.
4. After `warmup_samples` consecutive stationary samples, the average quaternion is used to compute a gravity-aligned initial orientation (roll + pitch only, yaw = 0).
5. The preintegration reference state is initialized with this orientation, and `warmup_complete_` is set to `true`.

This ensures that the IMU preintegrator starts from a gravity-aligned reference, preventing drift from incorrect initial orientation. It also ensures that relocalization does not fire on stale buffered sensor data before the IMU is ready.

## Odometry Output

After warmup, each IMU callback integrates the measurement and publishes odometry with:
- **Pose:** preintegrated position and orientation in the odom frame
- **Twist:** body-frame linear velocity (from preintegration) and angular velocity (from raw gyro)

This odometry is consumed by `eidos_transform` as a measurement source (typically for angular velocity fusion via twist mask).

## Notes

- IMU extrinsics (`imu_frame` to `base_link`) are resolved from TF on first message. All IMU data is rotated into the base_link frame before processing.
- Buffers up to 5000 IMU messages for consumption by `latchFactor()`.
- Currently produces `BetweenFactor<Pose3>` from the preintegrated pose delta. Full `gtsam::ImuFactor` with velocity + bias states is a future enhancement.
- `onOptimizationComplete()` re-anchors the preintegration reference state from optimized values, keeping the integrator consistent with graph corrections.

## Mapping vs Localization

| Parameter | Mapping | Localization |
|---|---|---|
| `add_factors` | `true` | `false` |

In **localization** mode, ImuFactor is loaded with `add_factors: false`. It still runs warmup, publishes odometry, and writes `setOdomPose()`, but does not inject BetweenFactors into the graph. Its primary role in localization is providing angular velocity data to `eidos_transform` for EKF fusion.

In **mapping** mode, ImuFactor can optionally be loaded with `add_factors: true` to provide IMU-derived constraints between consecutive keyframes alongside LISO's scan-matching constraints.

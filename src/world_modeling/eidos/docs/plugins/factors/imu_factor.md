# ImuFactor

**Class:** `eidos::ImuFactor`
**Type:** Factor (latching via `latchFactor`)
**XML:** `factor_plugins.xml`

IMU preintegration factor plugin. Subscribes to IMU, performs GTSAM preintegration, and produces `BetweenFactor<Pose3>` constraints between consecutive states. When another plugin (e.g. LISO) creates a state, `latchFactor()` drains the IMU buffer over the time interval and attaches an IMU-derived relative pose constraint.

Also writes an odom-frame pose via `setOdomPose()` for lock-free pose sharing within eidos.

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `imu_topic` | string | `"/imu/data"` | Input IMU topic. |
| `imu_frame` | string | `"imu_link"` | TF frame of the IMU sensor. |
| `acc_cov` | double[3] | `[9e-6, 9e-6, 9e-6]` | Accelerometer noise covariance. |
| `gyr_cov` | double[3] | `[1e-6, 1e-6, 1e-6]` | Gyroscope noise covariance. |
| `integration_cov` | double[3] | `[1e-4, 1e-4, 1e-4]` | Integration uncertainty covariance. |
| `gravity` | double | `9.80511` | Gravity magnitude (m/s^2). |
| `default_imu_dt` | double | `0.002` | Default IMU dt (seconds) for the first message. |

## Notes

- IMU extrinsics (imu_frame to base_link) are resolved from TF on first message.
- Buffers up to 5000 IMU messages for consumption by `latchFactor()`.
- Currently produces `BetweenFactor<Pose3>` from the preintegrated pose delta. Full `gtsam::ImuFactor` with velocity + bias states is a future enhancement.
- `onOptimizationComplete()` re-anchors the preintegration reference state from optimized values.

## Mapping vs Localization

Not loaded by default in either mode. Add `"imu_factor"` to the `factor_plugins` list in the config to enable. When enabled, it provides additional constraints between consecutive states created by other plugins (e.g. LISO).

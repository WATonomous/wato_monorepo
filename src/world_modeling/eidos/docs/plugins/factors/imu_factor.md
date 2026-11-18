# ImuFactor

**Class:** `eidos::ImuFactor`
**Type:** Factor (latching via `latchFactor`)
**XML:** `factor_plugins.xml`

IMU preintegration. Every state another plugin creates gets a velocity and an IMU bias of its own, and consecutive states are joined by a `gtsam::CombinedImuFactor`: the IMU measurements between the two states, preintegrated, constrain their relative pose, both velocities and the bias random walk between them.

It also publishes an odometry topic and writes an odom-frame pose via `setOdomPose()` for lock-free pose sharing within eidos and consumption by `eidos_transform`.

## Graph variables

| Key | Type | Meaning |
|---|---|---|
| `x<i>` | `Pose3` | The state, created by another plugin (e.g. `LisoFactor`, `MultiViewOdometryFactor`). |
| `v<i>` | `Vector3` | Velocity of `base_link` at state i, in the map frame. |
| `b<i>` | `imuBias::ConstantBias` | Accelerometer and gyroscope bias at state i, in the IMU frame. |

The first state gets two priors instead of an IMU factor:
- **velocity:** zero, with `prior.velocity_sigma_stationary` if the gyro stayed below `initialization.stationary_gyr_threshold` for the 0.5 s before it, otherwise `prior.velocity_sigma_moving`;
- **bias:** the warmup estimate, with `prior.acc_bias_sigma` / `prior.gyr_bias_sigma`.

Each latch also returns initial values for the new velocity and bias and the IMU's prediction of the new state's pose. Eidos uses that prediction as the state's initial value when the state's creator gives none.

## Input

The accelerometer must report **specific force including gravity**: preintegration subtracts gravity itself. On the Novatel that is `/novatel/oem7/imu/data_raw`; `/novatel/oem7/imu/data` has gravity removed. If the IMU does not read gravity at rest, the plugin logs an error and adds no factors.

The Novatel driver stamps `imu/data_raw` with `imu_link` but publishes it in the IMU's own axes, which are turned 90° from `imu_link` (forward is +y): only the INS outputs get the IMU-to-vehicle rotation. Fitted on the July 7 2026 bag against `imu/data` (gyro and acceleration agree within 0.4°): `imu_link = Rz(−90°) · data_raw`, hence `axes_rotation_rpy: [0, 0, -90]`. Without it heading is still right (the z axis is shared) but the accelerations, and so the velocities and translations, are turned 90°.

Measurements are integrated in the IMU frame. The `base_link` ← IMU extrinsic from TF is set as gtsam's `body_P_sensor`, which accounts for its rotation and for the centripetal acceleration of its lever arm.

## Parameters

Noise parameters follow gtsam's convention: continuous-time covariances, i.e. noise densities squared.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `imu_topic` | string | `"/imu/data"` | Input IMU topic (`sensor_msgs/Imu`), specific force with gravity. |
| `imu_frame` | string | `"imu_link"` | TF frame of the IMU sensor. |
| `axes_rotation_rpy` | double[3] | `[0, 0, 0]` | Roll, pitch, yaw (deg, applied as Rz·Ry·Rx) taking the message's axes into `imu_frame`, for drivers whose raw topic is not in the frame it is stamped with. Novatel `imu/data_raw`: `[0, 0, -90]` (see below). |
| `add_factors` | bool | `true` | Add IMU factors and velocity/bias states. `false`: odometry output only (localization). |
| `odom_topic` | string | `"imu_factor/odometry"` | Published odometry topic (`nav_msgs/Odometry`). |
| `acc_cov` | double[3] | `[9e-6, 9e-6, 9e-6]` | Accelerometer white noise, (m/s²/√Hz)². |
| `gyr_cov` | double[3] | `[1e-6, 1e-6, 1e-6]` | Gyroscope white noise, (rad/s/√Hz)². |
| `integration_cov` | double[3] | `[1e-8, 1e-8, 1e-8]` | Integration uncertainty, (m/s/√Hz)². |
| `acc_bias_cov` | double[3] | `[1e-6, 1e-6, 1e-6]` | Accelerometer bias random walk, (m/s³/√Hz)². |
| `gyr_bias_cov` | double[3] | `[4e-10, 4e-10, 4e-10]` | Gyroscope bias random walk, (rad/s²/√Hz)². |
| `gravity` | double | `9.80511` | Gravity magnitude (m/s²). |
| `initialization.warmup_samples` | int | `200` | Consecutive samples at rest needed to finish the warmup. |
| `initialization.stationary_gyr_threshold` | double | `0.005` | Gyro magnitude (rad/s) below which a sample counts as at rest. |
| `prior.velocity_sigma_stationary` | double | `0.05` | First state's velocity prior when the IMU shows it at rest (m/s). |
| `prior.velocity_sigma_moving` | double | `10.0` | First state's velocity prior otherwise (m/s). |
| `prior.acc_bias_sigma` | double | `0.1` | First state's accelerometer bias prior (m/s²). |
| `prior.gyr_bias_sigma` | double | `0.002` | First state's gyroscope bias prior around the warmup estimate (rad/s). |

## Warmup and `isReady()`

ImuFactor returns `isReady() == false` until warmup is complete, so eidos stays in `WARMING_UP` until then.

1. A sample counts as at rest if its gyro magnitude is below `stationary_gyr_threshold`; a moving sample restarts the count.
2. After `warmup_samples` consecutive samples at rest:
   - **roll and pitch** come from the mean accelerometer reading (the direction of gravity). If the accelerometer does not read gravity, the mean of the message orientations is used instead, and no factors are added.
   - **gyro bias** is the mean gyro reading (the earth's rotation, 7e-5 rad/s, stays in it).
   - yaw is 0; the first state takes the heading tracking begins with.

## Odometry Output

After warmup each IMU sample is integrated and published with:
- **pose:** in the odom frame;
- **twist:** body-frame linear velocity, and the bias-corrected angular rate in `base_link`.

With `add_factors: true` the integration restarts from the newest optimized state (pose, velocity, bias) after every optimization and catches up over the buffered samples. With `add_factors: false` it integrates from the warmup orientation at rest, without corrections: only its angular rate is meaningful over long periods.

## Mapping vs Localization

| Parameter | Mapping | Localization |
|---|---|---|
| `add_factors` | `true` | `false` |

In **localization** mode ImuFactor is typically loaded with `add_factors: false` to provide angular velocity to `eidos_transform`.

In **mapping** mode it can join the states of any state-creating plugin. With `MultiViewOdometryFactor` and `GpsFactor` it makes a lidar-free camera + IMU + GPS graph.

## Notes

- Buffers up to 5000 IMU samples (40 s at 125 Hz). States may be created up to that long after their time, e.g. by `MultiViewOdometryFactor`, which creates a state once its camera measurement is back.
- A state that is not later than the previous one, or has no IMU data since it, gets no IMU factor and no velocity or bias; the next IMU factor then spans from the previous state with one.

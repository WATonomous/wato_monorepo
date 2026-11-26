# GravityFactor

**Plugin:** `eidos::GravityFactor`
**Type:** Latching (attaches a unary attitude factor to states created by other plugins)
**Source:** `src/plugins/factors/gravity_factor.cpp`

A roll/pitch prior from the IMU's gravity reading. The LiDAR odometry constrains each state only relative to the previous one, so roll and pitch drift as a random walk, and with them the map's height (77–149 m over the 7.9 km Boreas drive without it). For each new state, this plugin measures the gravity direction in the body frame over a window ending at the state's timestamp and latches a `gtsam::Pose3AttitudeFactor`: a 2-DoF constraint on the direction of "up" that leaves yaw and position free. It never creates states and links no other plugin; in `accelerometer` mode it reads one ROS topic (a body-frame velocity) besides the IMU.

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `add_factors` | bool | `true` | Attach factors. `false` loads the plugin without changing the graph. |
| `mode` | string | `"accelerometer"` | `accelerometer`: mean specific force over the window minus the vehicle's own acceleration (from `velocity_topic`); absolute, so it cannot drift. `orientation`: "up" from the IMU message orientation (needs an INS-grade orientation that does not drift; a gyro-propagated orientation without Earth-rate compensation drifts by degrees within minutes). |
| `imu_topic` | string | `"imu/data"` | IMU topic (specific force including gravity, angular rate, optional orientation). |
| `imu_frame` | string | `"imu_link"` | IMU frame for the `base_link <- imu` rotation (TF). Empty uses the message's `frame_id`. |
| `velocity_topic` | string | `"liso/odometry"` | `nav_msgs/Odometry` whose twist is the body-frame velocity, used to remove the vehicle's acceleration (`dv/dt + w x v`). The LiDAR odometry publishes it. Empty skips the compensation. |
| `window` | double | `1.0` | Length of the measurement window ending at the state (seconds). |
| `sigma` | double | `0.03` | Standard deviation of the measured gravity direction (radians, both axes). |
| `huber_k` | double | `1.345` | Huber threshold in sigmas; `0` uses a plain Gaussian. |
| `gravity` | double | `9.81` | Gravity magnitude for the specific-force gate (m/s^2). |
| `max_accel_deviation` | double | `0.5` | Skip the window if the compensated specific force differs from `gravity` by more (m/s^2). |
| `max_angular_rate` | double | `0.3` | Skip the window if the mean angular rate is higher (rad/s). |
| `min_imu_samples` | int | `10` | Minimum IMU samples in the window; the window must also be covered at both ends with no gap over 0.1 s. |

## Notes

- Every 100 states it logs the measured "up" in the body frame, its tilt, and how many states got a factor or were skipped (dynamics gate, or no data).
- Only roll and pitch are constrained; with `sigma` 0.03 rad against the LiDAR odometry's 1 mrad per-step roll/pitch noise, about 30 consecutive factors outweigh the odometry, which bounds roll/pitch error near 0.3 degrees instead of letting it grow.
- The factor needs nothing from other plugins beyond the velocity topic, so it works with any state-producing plugin.

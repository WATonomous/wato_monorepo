# eidos_transform -- Developer Guide

## EKFModelPlugin base class

All EKF models inherit from `eidos_transform::EKFModelPlugin` (defined in `include/eidos_transform/ekf_model_plugin.hpp`) and are loaded at runtime via `pluginlib`.

### Interface

```cpp
class EKFModelPlugin {
public:
  // Called once by the framework. Stores name/node, then calls onInitialize().
  void initialize(const std::string & name, LifecycleNode::SharedPtr node);

  // Lifecycle hooks (pure virtual).
  virtual void onInitialize() = 0;
  virtual void activate() = 0;
  virtual void deactivate() = 0;

  // Propagate state forward by dt seconds.
  virtual void predict(double dt) = 0;

  // Fuse a 6-DOF pose measurement. Only DOFs where mask[i]==true are used.
  virtual void updatePose(
    const gtsam::Pose3 & meas,
    const std::array<bool, 6> & mask,
    const gtsam::Vector6 & noise) = 0;

  // Fuse a 6-DOF twist measurement (body-frame velocity). Same masking.
  virtual void updateTwist(
    const gtsam::Vector6 & meas,
    const std::array<bool, 6> & mask,
    const gtsam::Vector6 & noise) = 0;

  // Fuse a body-frame linear acceleration measurement (gravity-compensated).
  // Default implementation is a no-op. Override to support IMU acceleration.
  virtual void updateAcceleration(
    const Eigen::Vector3d & accel,
    const Eigen::Vector3d & noise,
    double dt);

  // Current estimated pose.
  virtual gtsam::Pose3 pose() const = 0;

  // Current estimated body-frame velocity (6-DOF).
  virtual gtsam::Vector6 velocity() const = 0;

  // Current accelerometer bias estimate. Default: zero.
  virtual Eigen::Vector3d accelBias() const;

  // Hard-reset to a known pose, zero velocity, default covariance.
  virtual void reset(const gtsam::Pose3 & initial) = 0;

  // Capture/restore full internal state for rewind-replay.
  virtual StateSnapshot snapshot(double time) const = 0;
  virtual void restore(const StateSnapshot & snap) = 0;

protected:
  std::string name_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};
```

`name_` and `node_` are set before `onInitialize()` is called. Use `node_` to declare and read plugin-specific parameters, access the logger, etc.

### StateSnapshot

Used by the rewind-replay system. Captures the full EKF state at a point in time:

```cpp
struct StateSnapshot {
  double time = 0.0;
  gtsam::Pose3 pose;
  gtsam::Vector6 velocity = gtsam::Vector6::Zero();
  Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
  Eigen::MatrixXd P;  // Covariance (size depends on plugin)
};
```

### 6-DOF ordering

All 6-element vectors and masks follow the convention:

```
[rx, ry, rz, tx, ty, tz]
```

Rotation first (Lie algebra), then translation. This matches gtsam's convention.

## HolonomicEKF

Located in `plugins/holonomic_ekf.hpp` / `src/holonomic_ekf.cpp`.

### 15-state model

State layout (`kStateDim = 15`):

| Indices | Name | Storage | Description |
|---|---|---|---|
| 0--5 | Pose | `gtsam::Pose3 pose_` | 6-DOF pose: rx, ry, rz, tx, ty, tz (Lie algebra, stored as Pose3) |
| 6--11 | Velocity | `gtsam::Vector6 velocity_` | Body-frame velocity: angular_x, angular_y, angular_z, linear_x, linear_y, linear_z |
| 12--14 | Accel bias | `Eigen::Vector3d accel_bias_` | Accelerometer bias: bias_ax, bias_ay, bias_az |

### Full 15x15 covariance

The covariance matrix `P_` is a full 15x15 dense matrix (`Eigen::Matrix<double, 15, 15>`). Cross-covariance terms between pose, velocity, and accel bias are maintained and updated by the Kalman gain. This means:

- A pose correction propagates to velocity and bias via cross-covariance.
- A twist correction propagates to pose and bias via cross-covariance.
- An acceleration correction propagates to pose, velocity, and bias.

### Predict step

Constant-velocity model with random-walk bias:

```
pose = pose * Expmap(velocity * dt)    // integrate velocity into pose
velocity = velocity                     // unchanged by prediction
bias = bias                             // random walk (grows via process noise)
```

A safety check rejects the integration if the velocity delta is non-finite or the translation magnitude exceeds 5.0 m per step.

**State transition Jacobian F (15x15):**

```
F = [I_6   dt*I_6   0_6x3]    (pose depends on velocity)
    [0_6      I_6    0_6x3]    (velocity unchanged)
    [0_3x6   0_3x6   I_3  ]    (bias random walk)
```

The off-diagonal `dt*I_6` block couples velocity uncertainty into pose uncertainty during propagation.

**Covariance propagation:**

```
P = F * P * F^T + Q * dt
```

where `Q` is the 15x15 diagonal process noise matrix.

### updatePose: full Kalman gain with sequential DOF updates

For each DOF `i` where `mask[i]` is true, a scalar update is applied using the full cross-covariance:

```
innovation = Logmap(pose^{-1} * measurement)    // 6-DOF error in Lie algebra
R_i = noise(i)^2                                 // scalar measurement variance
S_i = P(i, i) + R_i                              // scalar innovation variance

K = P.col(i) / S_i                               // full 15x1 Kalman gain vector

// Apply correction to ALL states via cross-covariance:
pose_delta    = K[0..5]  * innovation(i)    -> pose = pose * Expmap(pose_delta)
velocity     += K[6..11] * innovation(i)
accel_bias   += K[12..14] * innovation(i)

// Covariance update:
P -= K * P.row(i)
P = (P + P^T) / 2                                // enforce symmetry
```

After each DOF update, the innovation is **recomputed** from the corrected pose (`re-innovation`). This handles nonlinear coupling between rotation and translation -- correcting one DOF changes the innovation for subsequent DOFs.

### updateTwist: full Kalman gain with sequential DOF updates

Same structure as updatePose, but the observation maps to velocity states (indices 6..11):

```
innovation = measurement - velocity
R_i = noise(i)^2
S_i = P(i+6, i+6) + R_i

K = P.col(i+6) / S_i                             // full 15x1 Kalman gain

// Apply correction to ALL states:
pose_delta    = K[0..5]  * innovation(i)    -> pose = pose * Expmap(pose_delta)
velocity     += K[6..11] * innovation(i)
accel_bias   += K[12..14] * innovation(i)

P -= K * P.row(i+6)
P = (P + P^T) / 2
```

Re-innovation is recomputed after each DOF update (`innovation = measurement - velocity`).

### updateAcceleration: proper H matrix with velocity and bias

The acceleration update uses an explicit observation matrix `H` that maps the 15-state to the acceleration measurement. For each axis `i` (0, 1, 2):

The measurement model is: `z = true_accel + bias + noise`. Under the constant-velocity prediction, predicted acceleration is zero, so:

```
innovation = accel(i) - accel_bias(i)

H (1x15):
  H[vel_idx]  = 1/dt      where vel_idx = 9 + i  (linear velocity states)
  H[bias_idx] = 1.0       where bias_idx = 12 + i (bias states)
  all other entries = 0

S = H * P * H^T + R_i
K = P * H^T / S           // full 15x1 Kalman gain

// State correction (all states):
pose_delta    = K[0..5]  * innovation
velocity     += K[6..11] * innovation
accel_bias   += K[12..14] * innovation

// Joseph form covariance update:
P = (I - K * H) * P
P = (P + P^T) / 2
```

This couples acceleration measurements to velocity and bias states through the cross-covariance. The `1/dt` term in H means acceleration is treated as the rate of change of velocity.

### Process noise

Read from `<ekf_name>.process_noise` as a 15-element vector. Stored as the diagonal of `Q_` (15x15 diagonal matrix):

```
[1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4,   # pose (indices 0..5)
 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2,    # velocity (indices 6..11)
 1e-6, 1e-6, 1e-6]                        # accel bias (indices 12..14)
```

### Reset

Sets pose to the given value, velocity to zero, accel bias to zero, and covariance `P` to 15x15 identity.

### snapshot / restore

`snapshot(time)` captures `pose_`, `velocity_`, `accel_bias_`, and the full `P_` matrix into a `StateSnapshot`. `restore(snap)` writes them back, with a dimension check on P.

## MeasurementSource

Defined in `eidos_transform_node.hpp`. The unified struct supports both odom and IMU sources:

```cpp
struct MeasurementSource {
  std::string name;
  std::string type = "odom";   // "odom" or "imu"
  std::string topic;

  // Common: per-DOF masks and noise (odom type)
  std::array<bool, 6> pose_mask;
  std::array<bool, 6> twist_mask;
  gtsam::Vector6 pose_noise;
  gtsam::Vector6 twist_noise;

  // Odom-type state
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  nav_msgs::msg::Odometry::SharedPtr latest_odom;

  // IMU-type config
  std::string imu_frame = "imu_link";
  bool use_orientation = true;
  bool use_angular_velocity = true;
  bool use_linear_acceleration = false;
  double gravity = 9.80511;
  bool gravity_compensated = false;
  gtsam::Vector6 orientation_noise;
  gtsam::Vector6 angular_velocity_noise;
  gtsam::Vector6 linear_velocity_noise;

  // IMU-type state
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  sensor_msgs::msg::Imu::SharedPtr latest_imu;
  Eigen::Matrix3d R_base_imu;      // cached imu_frame->base_link rotation
  bool has_imu_tf = false;
  double last_imu_time = 0.0;

  // Common state
  bool has_new_data = false;
  bool logged_first_msg = false;
};
```

Sources are configured from the `odom_sources` and `map_sources` parameter lists. The `parseSources` function reads parameters, creates subscriptions, and populates the source vector. Subscriber callbacks store the latest message and set `has_new_data = true` under the sources mutex.

## MeasurementRecord and measurement history

For the global EKF's rewind-replay system, each measurement is recorded as a `MeasurementRecord`:

```cpp
struct MeasurementRecord {
  double time;                    // measurement timestamp (from msg header)
  Target target;                  // LOCAL, GLOBAL, or BOTH
  std::string source_name;
  std::string source_type;        // "odom" or "imu"

  // Odom data (captured at time of arrival)
  gtsam::Pose3 pose;
  gtsam::Vector6 twist;
  std::array<bool, 6> pose_mask;
  std::array<bool, 6> twist_mask;
  gtsam::Vector6 pose_noise;
  gtsam::Vector6 twist_noise;

  // IMU data (captured at time of arrival)
  Eigen::Vector3d gyro, accel, orientation_rpy;
  bool has_orientation, use_orientation;
  bool use_angular_velocity, use_linear_acceleration;
  gtsam::Vector6 imu_orientation_noise;
  gtsam::Vector6 imu_angular_velocity_noise;
  Eigen::Vector3d imu_accel_noise;
  double imu_dt;
};
```

Measurement records are appended to `global_measurement_history_` (bounded deque, max 500 entries). State snapshots are stored in `global_state_history_` (also bounded at 500).

## Rewind-replay algorithm

When a map source measurement arrives with a timestamp older than `global_ekf_time_ - dt - 0.01`:

1. **Find restore point:** Walk `global_state_history_` backwards to find the latest snapshot with `time <= delayed_time`.
2. **Restore:** Call `global_ekf_->restore(restore_point)` to reset to that state.
3. **Collect measurements:** Gather all records from `global_measurement_history_` with `time > restore_point.time`.
4. **Sort:** Order collected measurements by timestamp.
5. **Replay:** For each measurement, predict the EKF forward by `(rec.time - last_time)`, then call `applyMeasurement`.
6. **Final predict:** Predict from the last replayed measurement to `global_ekf_time_`.
7. **Reset history:** Clear `global_state_history_` and save a fresh snapshot.

The `applyMeasurement` function checks the `source_type` and applies pose/twist updates for odom-type records (respecting masks).

## Threading model

- **Tick timer:** Runs in its own `MutuallyExclusive` callback group (`tick_callback_group_`). This guarantees the tick loop never runs concurrently with itself.
- **Subscriber callbacks** (odom, IMU, UTM): Run in the default callback group. They acquire `sources_mutex_` to write `latest_odom`/`latest_imu` and set `has_new_data`.
- **Tick loop:** Acquires `sources_mutex_` while iterating over sources. All EKF predict/update, TF broadcasting, and odometry publishing happen inside the tick callback, so they are serialized.
- **PredictRelativeTransform service:** Acquires `sources_mutex_` to read the local EKF velocity. May run concurrently with subscriber callbacks but not with the tick (if using a multi-threaded executor, the tick's mutual exclusion prevents overlap only within its callback group -- the service runs in the default group and protects shared state via the mutex).

Key synchronization:

| Resource | Guard | Writers | Readers |
|---|---|---|---|
| `odom_sources_[*].latest_odom/imu` | `sources_mutex_` | Subscriber callbacks | Tick loop |
| `odom_sources_[*].has_new_data` | `sources_mutex_` | Subscriber callbacks | Tick loop |
| `map_sources_[*].latest_odom/imu` | `sources_mutex_` | Subscriber callbacks | Tick loop |
| `local_ekf_` state | Tick callback group serialization | Tick loop | PredictRelativeTransform (via mutex) |
| `global_ekf_` state | Tick callback group serialization | Tick loop | -- |
| `cached_utm_to_map_` | Single writer (callback) | UTM callback | Tick broadcasts it once |

## Writing a new EKF model plugin

Example: creating an `AckermannEKF`.

### Step 1: Create the header

`include/eidos_transform/plugins/ackermann_ekf.hpp`:

```cpp
#pragma once
#include "eidos_transform/ekf_model_plugin.hpp"

namespace eidos_transform {

class AckermannEKF : public EKFModelPlugin {
public:
  void onInitialize() override;
  void activate() override;
  void deactivate() override;

  void predict(double dt) override;
  void updatePose(const gtsam::Pose3 & meas,
                  const std::array<bool, 6> & mask,
                  const gtsam::Vector6 & noise) override;
  void updateTwist(const gtsam::Vector6 & meas,
                   const std::array<bool, 6> & mask,
                   const gtsam::Vector6 & noise) override;
  void updateAcceleration(const Eigen::Vector3d & accel,
                          const Eigen::Vector3d & noise,
                          double dt) override;

  gtsam::Pose3 pose() const override;
  gtsam::Vector6 velocity() const override;
  Eigen::Vector3d accelBias() const override;
  void reset(const gtsam::Pose3 & initial) override;

  StateSnapshot snapshot(double time) const override;
  void restore(const StateSnapshot & snap) override;

private:
  // Your state variables, covariance, etc.
};

}  // namespace eidos_transform
```

### Step 2: Implement and register

`src/ackermann_ekf.cpp`:

```cpp
#include "eidos_transform/plugins/ackermann_ekf.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace eidos_transform {

void AckermannEKF::onInitialize() {
  // Use node_->declare_parameter / get_parameter for plugin-specific config.
  // Parameters are namespaced under name_ (e.g., "ackermann_ekf.wheelbase").
}

// ... implement all methods ...

}  // namespace eidos_transform

PLUGINLIB_EXPORT_CLASS(eidos_transform::AckermannEKF, eidos_transform::EKFModelPlugin)
```

### Step 3: Add to CMakeLists.txt

Add the `.cpp` file to the `eidos_transform_plugins` library target.

### Step 4: Register in ekf_plugins.xml

```xml
<class name="eidos_transform::AckermannEKF"
       type="eidos_transform::AckermannEKF"
       base_class_type="eidos_transform::EKFModelPlugin">
  <description>Ackermann kinematic EKF</description>
</class>
```

### Step 5: Configure

Set `ekf.plugin` and `ekf.name` in the config YAML. Both the local and global EKF will use the same plugin class but with separate instances and parameter namespaces (`<name>` and `<name>_global`).

### Important notes for plugin authors

- The node calls `initialize(name, node)` once during `on_configure`, then `activate()`/`deactivate()` on lifecycle transitions.
- The tick loop calls `predict(dt)` once per tick, then `updatePose`/`updateTwist`/`updateAcceleration` for each source with new data.
- `snapshot()` and `restore()` must capture and restore the complete internal state (pose, velocity, bias, covariance, any other state). The rewind-replay system depends on this for correctness.
- `updateAcceleration` has a default no-op implementation. Override it only if your model supports acceleration/bias estimation.
- `accelBias()` has a default that returns zero. Override if your model tracks bias.

## Process noise tuning guide

The 15-element process noise vector controls how fast uncertainty grows during prediction:

| Indices | States | Effect of increasing | Effect of decreasing |
|---|---|---|---|
| 0--5 | Pose (rx, ry, rz, tx, ty, tz) | EKF trusts measurements more, faster correction but more jitter | EKF trusts its own prediction more, smoother but slower correction |
| 6--11 | Velocity (angular, linear) | Velocity estimates change faster in response to measurements | Velocity is more damped, slower to track changes |
| 12--14 | Accel bias (ax, ay, az) | Bias estimate adapts faster (good for changing bias, risky for noise) | Bias estimate is more stable (good for constant bias) |

General guidelines:

- **Pose noise** should be small (1e-4 to 1e-3) since the constant-velocity model is reasonable over short time steps.
- **Velocity noise** should be larger (1e-2 to 1e-1) to allow the EKF to track velocity changes from measurements.
- **Bias noise** should be very small (1e-6 to 1e-5) since accelerometer bias changes slowly.
- The local and global EKFs can have different process noise. The global EKF may need higher pose noise to accommodate map-frame corrections arriving at lower rates.
- Process noise is scaled by `dt` in the propagation (`P = F*P*F^T + Q*dt`), so the values are per-second rates.

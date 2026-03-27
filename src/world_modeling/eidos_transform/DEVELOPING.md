# eidos_transform -- Developer Guide

## EKFModelPlugin base class

All EKF models inherit from `eidos_transform::EKFModelPlugin` (defined in `include/eidos_transform/ekf_model_plugin.hpp`) and are loaded at runtime via `pluginlib`.

### Interface

```cpp
class EKFModelPlugin {
public:
  // Called once by the framework. Calls onInitialize() internally.
  void initialize(const std::string & name, LifecycleNode::SharedPtr node);

  // Lifecycle hooks (pure virtual).
  virtual void onInitialize() = 0;
  virtual void activate() = 0;
  virtual void deactivate() = 0;

  // Propagate state forward by dt seconds.
  virtual void predict(double dt) = 0;

  // Fuse a 6-DOF pose measurement. Only DOFs where mask[i]==true are used.
  // noise contains the standard deviation for each DOF.
  virtual void updatePose(
    const gtsam::Pose3 & meas,
    const std::array<bool, 6> & mask,
    const gtsam::Vector6 & noise) = 0;

  // Fuse a 6-DOF twist measurement (body-frame velocity). Same masking.
  // Layout: [angular_x, angular_y, angular_z, linear_x, linear_y, linear_z].
  virtual void updateTwist(
    const gtsam::Vector6 & meas,
    const std::array<bool, 6> & mask,
    const gtsam::Vector6 & noise) = 0;

  // Current estimated pose (odom frame).
  virtual gtsam::Pose3 pose() const = 0;

  // Current estimated body-frame velocity (6-DOF).
  virtual gtsam::Vector6 velocity() const = 0;

  // Hard-reset to a known pose, zero velocity, default covariance.
  virtual void reset(const gtsam::Pose3 & initial) = 0;

protected:
  std::string name_;                          // Instance name from config
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;  // Parent node
};
```

`name_` and `node_` are set before `onInitialize()` is called. Use `node_` to declare and read plugin-specific parameters, access the logger, etc.

### 6-DOF ordering

All 6-element vectors and masks follow the convention:

```
[rx, ry, rz, tx, ty, tz]
```

Rotation first (Lie algebra), then translation.

## Writing a new EKF model

1. Create a new class inheriting `EKFModelPlugin`. Implement all pure virtual methods.

2. Register the plugin with the `PLUGINLIB_EXPORT_CLASS` macro at the bottom of the `.cpp` file:

```cpp
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(eidos_transform::AckermannEKF, eidos_transform::EKFModelPlugin)
```

1. Add the `.cpp` file to the `eidos_transform_plugins` library in `CMakeLists.txt`.

2. Add a `<class>` entry in `ekf_plugins.xml`:

```xml
<class name="eidos_transform::AckermannEKF"
       type="eidos_transform::AckermannEKF"
       base_class_type="eidos_transform::EKFModelPlugin">
  <description>Ackermann kinematic EKF</description>
</class>
```

1. Set `ekf.plugin` and `ekf.name` in the config YAML to use the new plugin.

The node calls `initialize(name, node)` once during `on_configure`, then `activate()`/`deactivate()` on lifecycle transitions. The tick loop calls `predict(dt)` then `updatePose`/`updateTwist` for each source with new data, every tick.

## HolonomicEKF

Located in `holonomic_ekf.hpp` / `holonomic_ekf.cpp`.

### State representation

12-state model: 6-DOF pose + 6-DOF body-frame velocity.

- **Pose:** stored as `gtsam::Pose3` (not a raw vector) to avoid singularities.
- **Velocity:** stored as `gtsam::Vector6` with layout `[angular_x, angular_y, angular_z, linear_x, linear_y, linear_z]`.
- **Covariance:** `Eigen::Matrix<double, 12, 12>` -- diagonal approximation. Indices 0--5 are pose, 6--11 are velocity.

### Predict step

Constant-velocity model:

```
pose = pose.compose(Pose3::Expmap(velocity * dt))
velocity = velocity  (unchanged)
```

Covariance grows linearly: `P += Q * dt`. The state transition Jacobian `F` is approximated as identity.

### Masked Kalman update

Both `updatePose` and `updateTwist` iterate over the 6 DOFs. For each DOF `i` where `mask[i]` is true, a scalar Kalman update is applied:

**Pose update** (DOFs 0..5 of the 12-state):

```
innovation = Logmap(pose^{-1} * measurement)
R_i = noise(i)^2
K_i = P(i,i) / (P(i,i) + R_i)
correction(i) = K_i * innovation(i)
P(i,i) *= (1 - K_i)
```

After iterating all DOFs, the correction is applied: `pose = pose.compose(Expmap(correction))`.

**Twist update** (DOFs 6..11 of the 12-state):

```
innovation = measurement - velocity
R_i = noise(i)^2
K_i = P(i+6, i+6) / (P(i+6, i+6) + R_i)
velocity(i) += K_i * innovation(i)
P(i+6, i+6) *= (1 - K_i)
```

The diagonal approximation makes each DOF independent, which avoids the cost of a full matrix inversion and simplifies the masked update.

### Process noise

Read from the parameter `<ekf_name>.process_noise` as a 12-element vector. Stored as the diagonal of `Q_`. Default:

```
[1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4,   # pose
 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2]    # velocity
```

### Reset

Sets pose to the given value, velocity to zero, and covariance `P` to 12x12 identity (all diagonal elements = 1.0).

## MeasurementSource

Defined in `eidos_transform_node.hpp`:

```cpp
struct MeasurementSource {
  std::string name;
  std::string odom_topic;

  std::array<bool, 6> pose_mask;    // which pose DOFs to fuse
  std::array<bool, 6> twist_mask;   // which twist DOFs to fuse
  gtsam::Vector6 pose_noise;        // std devs for pose
  gtsam::Vector6 twist_noise;       // std devs for twist

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber;
  nav_msgs::msg::Odometry::SharedPtr latest_msg;
  bool has_new_data = false;

  gtsam::Pose3 last_pose;           // for change detection
  bool has_last_pose = false;
};
```

Sources are configured from the `odom_sources` parameter list. Each name becomes a parameter namespace with `odom_topic`, `pose_mask`, `twist_mask`, `pose_noise`, and `twist_noise`. The subscriber callback stores the latest message and sets `has_new_data = true`. The tick loop drains new data under the mutex.

## Threading model

The node uses a `MultiThreadedExecutor` (or however the launch file provides one).

- **Tick timer** runs in its own `MutuallyExclusive` callback group (`tick_callback_group_`). This guarantees the tick loop never runs concurrently with itself.
- **Odometry subscriber callbacks** run in the default callback group. They acquire `sources_mutex_` to write `latest_msg` and `has_new_data`.
- **Map source and UTM callbacks** run in the default callback group. The map source callback acquires `map_to_odom_mtx_` to write `cached_map_to_odom_`, then sets `has_map_to_odom_` atomically. The UTM callback writes `cached_utm_to_map_` and its boolean flag.
- **Tick loop** acquires `sources_mutex_` while reading source data. Reads of `cached_map_to_odom_` are guarded by `map_to_odom_mtx_`. The `has_map_to_odom_` flag is an `std::atomic<bool>` set with release ordering after the pose is written, so it can be checked without the mutex.

The predict-then-update cycle and all TF broadcasting happen inside the tick callback, so they are serialized.

## PredictRelativeTransform service

Handles `eidos_msgs/srv/PredictRelativeTransform` requests. Given `timestamp_from` and `timestamp_to`:

1. Computes `dt = timestamp_to - timestamp_from`.
2. Rejects if `dt < 0`.
3. Reads the current EKF velocity.
4. Integrates forward: `relative = Pose3::Expmap(velocity * dt)`.
5. Returns the resulting `geometry_msgs/Pose` and `success = true`.

This is a constant-velocity dead-reckoning prediction. It does not replay historical state -- it uses the velocity estimate at the time the service is called.

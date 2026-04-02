# eidos_transform

Dual-EKF multi-source odometry fusion and TF broadcasting for ROS 2.

`eidos_transform` runs as a standalone lifecycle node, separate from eidos SLAM. It maintains two independent EKF instances -- a **local EKF** operating in the odom frame and a **global EKF** operating in the map frame -- fuses configurable measurement sources into each, broadcasts the resulting TF tree, and publishes a fused odometry topic.

## Architecture

### Dual-EKF design

The node follows a robot_localization-style dual-EKF architecture:

- **Local EKF (odom frame):** Fuses `odom_sources` only. Produces a smooth, continuous `odom -> base_link` transform and the published `/odom` topic. This EKF never receives map-frame corrections, so it remains jitter-free.
- **Global EKF (map frame):** Fuses `odom_sources` (same data as the local EKF) plus `map_sources` (e.g. SLAM pose, GPS corrections). Used solely to derive the `map -> odom` transform.

Both EKFs are the same pluginlib-loaded model (e.g. `HolonomicEKF`), instantiated twice with separate parameter namespaces (`holonomic_ekf` for local, `holonomic_ekf_global` for global).

### map -> odom computation

The `map -> odom` transform is computed directly from the two EKF poses without any TF lookup:

```
map_to_odom = global_ekf_pose * inverse(local_ekf_pose)
```

Both EKFs are updated in the same tick, so the local EKF pose used here is exactly the `odom -> base_link` that was just broadcast. This eliminates timestamp-matching issues entirely.

### Rewind-replay for delayed measurements

Map-frame sources (e.g. SLAM) often arrive with latency. The global EKF handles this with a rewind-replay mechanism:

1. After each predict step, a `StateSnapshot` of the global EKF is saved to a bounded history (max 500 entries).
2. When a map source measurement arrives with a timestamp older than the current tick, the algorithm finds the latest snapshot before that timestamp.
3. The global EKF is restored to that snapshot.
4. All measurements from that point forward (including the delayed one) are sorted by time and replayed with predict steps between them.
5. A final predict brings the EKF to the current time.

This only applies to the global EKF. The local EKF fuses measurements immediately without delay handling.

### Measurement sources

Sources are configured as a unified `MeasurementSource` struct supporting two types via the `type` field:

- **`type: "odom"`** -- subscribes to a `nav_msgs/Odometry` topic. Provides 6-DOF pose and 6-DOF twist, each independently masked and with per-DOF noise.
- **`type: "imu"`** -- subscribes to a `sensor_msgs/Imu` topic. Provides angular velocity, orientation, and optionally linear acceleration with gravity compensation and accelerometer bias estimation.

Sources are assigned to EKFs by which list they appear in:

| List | Local EKF | Global EKF |
|---|---|---|
| `odom_sources` | Yes | No |
| `map_sources` | No | Yes |

### IMU source processing

When an IMU source is fused:

1. **Frame rotation:** The `imu_frame -> base_link` static TF is looked up once and cached. All IMU measurements are rotated into the base link frame before fusion.
2. **Angular velocity:** Fused as a twist update (rotation DOFs only) using `updateTwist`.
3. **Orientation:** The IMU quaternion is converted to a rotation matrix, combined with the IMU-to-base rotation, and fused as a pose update (rotation DOFs only) using `updatePose`. The current EKF translation is preserved so `Logmap` only sees rotation error.
4. **Linear acceleration** (optional): Gravity is removed using the EKF's current orientation estimate (unless `gravity_compensated` is true). The gravity-free acceleration is passed to `updateAcceleration`, which handles bias subtraction and velocity integration internally.

### Data flow

```
odom_sources (wheel odom, LiDAR odom, IMU) --> [Local EKF]  --> odom->base_link TF + /odom topic
odom_sources + map_sources (SLAM, GPS)      --> [Global EKF] --> map->odom TF
UTM transform                               --> [broadcast]  --> utm->map static TF
```

## Quick start

Standalone launch:

```bash
ros2 launch eidos_transform eidos_transform.launch.yaml
```

With a custom config:

```bash
ros2 launch eidos_transform eidos_transform.launch.yaml config_file:=/path/to/params.yaml
```

Launch arguments:

| Argument | Default | Description |
|---|---|---|
| `config_file` | `$(find-pkg-share eidos_transform)/config/params.yaml` | Path to parameter file |
| `use_sim_time` | `true` | Use `/clock` topic for time |
| `autostart` | `true` | Auto-configure and activate the lifecycle node |

The node runs under the `/world_modeling` namespace and is managed by `wato_lifecycle_manager`.

## Configuration

All parameters live under `/**/eidos_transform_node/ros__parameters`.

### Core parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `tick_rate` | double | `200.0` | Main loop frequency in Hz |
| `frames.odom` | string | `"odom"` | Odom frame ID |
| `frames.base_link` | string | `"base_footprint"` | Base link frame ID |
| `frames.map` | string | `"map"` | Map frame ID |
| `frames.utm` | string | `"utm"` | UTM frame ID |
| `topics.odom` | string | `"transform/odometry"` | Fused odometry output topic |
| `topics.utm_to_map` | string | `""` | `TransformStamped` topic for the static UTM-to-map transform. Set to `""` to disable. |
| `topics.predict_service` | string | `"transform/predict_relative"` | Service name for `PredictRelativeTransform` |

### EKF model plugin

| Parameter | Type | Default | Description |
|---|---|---|---|
| `ekf.plugin` | string | `"eidos_transform::HolonomicEKF"` | Fully qualified plugin class name |
| `ekf.name` | string | `"holonomic_ekf"` | Instance name (used to namespace plugin-specific parameters). The global EKF instance uses `<name>_global` automatically. |

The HolonomicEKF plugin reads process noise under `<ekf.name>.process_noise`:

| Parameter | Type | Default | Description |
|---|---|---|---|
| `holonomic_ekf.process_noise` | double[15] | `[1e-4]*6 + [1e-2]*6 + [1e-6]*3` | 15 diagonal process noise values: 6 for pose (rx, ry, rz, tx, ty, tz), 6 for velocity (angular_x, angular_y, angular_z, linear_x, linear_y, linear_z), 3 for accelerometer bias (bias_ax, bias_ay, bias_az) |
| `holonomic_ekf_global.process_noise` | double[15] | same | Process noise for the global EKF instance (tuned independently) |

### Odom sources (local EKF only)

Sources are listed by name in `odom_sources`, then each name gets its own parameter block. These are fed to the local EKF only.

#### Odom-type source

```yaml
odom_sources:
  - "liso_odom"

liso_odom:
  type: "odom"
  topic: "liso/odometry_incremental"
  pose_mask: [true, true, true, true, true, true]
  twist_mask: [false, false, false, false, false, false]
  pose_noise: [1.0e-3, 1.0e-3, 1.0e-3, 1.0e-3, 1.0e-3, 1.0e-3]
  twist_noise: [1.0e-2, 1.0e-2, 1.0e-2, 1.0e-2, 1.0e-2, 1.0e-2]
```

| Parameter | Type | Default | Description |
|---|---|---|---|
| `<name>.type` | string | `"odom"` | Source type. `"odom"` for `nav_msgs/Odometry` topics. |
| `<name>.topic` | string | `""` | Topic to subscribe to (SensorDataQoS). Required. |
| `<name>.pose_mask` | bool[6] | `[false]*6` | Which pose DOFs to fuse: `[rx, ry, rz, tx, ty, tz]` |
| `<name>.twist_mask` | bool[6] | `[false]*6` | Which twist DOFs to fuse: `[angular_x, angular_y, angular_z, linear_x, linear_y, linear_z]` |
| `<name>.pose_noise` | double[6] | `[1e-2]*6` | Standard deviations for each pose DOF |
| `<name>.twist_noise` | double[6] | `[1e-2]*6` | Standard deviations for each twist DOF |

#### IMU-type source

```yaml
odom_sources:
  - "imu_raw"

imu_raw:
  type: "imu"
  topic: "/novatel/oem7/imu/data"
  imu_frame: "imu_link"
  use_orientation: true
  use_angular_velocity: true
  use_linear_acceleration: false
  gravity: 9.80511
  gravity_compensated: false
  orientation_noise: [1.0e-2, 1.0e-2, 1.0e-2, 1.0, 1.0, 1.0]
  angular_velocity_noise: [1.0e-4, 1.0e-4, 1.0e-4, 1.0, 1.0, 1.0]
  linear_velocity_noise: [1.0, 1.0, 1.0, 1.0e-1, 1.0e-1, 1.0e-1]
```

| Parameter | Type | Default | Description |
|---|---|---|---|
| `<name>.type` | string | -- | Must be `"imu"` for `sensor_msgs/Imu` topics. |
| `<name>.topic` | string | `""` | Topic to subscribe to (SensorDataQoS). Required. |
| `<name>.imu_frame` | string | `"imu_link"` | TF frame of the IMU sensor (used for `imu_frame -> base_link` rotation lookup) |
| `<name>.use_orientation` | bool | `true` | Fuse IMU orientation as a rotation-only pose update |
| `<name>.use_angular_velocity` | bool | `true` | Fuse IMU angular velocity as a twist update (rotation DOFs) |
| `<name>.use_linear_acceleration` | bool | `false` | Fuse IMU linear acceleration (with gravity compensation and bias estimation) |
| `<name>.gravity` | double | `9.80511` | Gravity magnitude (m/s^2) for gravity removal |
| `<name>.gravity_compensated` | bool | `false` | If true, the sensor already removes gravity; skip gravity compensation |
| `<name>.orientation_noise` | double[6] | `[1e-2, 1e-2, 1e-2, 1.0, 1.0, 1.0]` | Noise for orientation pose update (only rotation DOFs [0..2] matter) |
| `<name>.angular_velocity_noise` | double[6] | `[1e-4, 1e-4, 1e-4, 1.0, 1.0, 1.0]` | Noise for angular velocity twist update (only rotation DOFs [0..2] matter) |
| `<name>.linear_velocity_noise` | double[6] | `[1.0, 1.0, 1.0, 1e-1, 1e-1, 1e-1]` | Noise for acceleration updates (only translation DOFs [3..5] are used as the 3D accel noise) |

### Map sources (global EKF only)

Sources listed in `map_sources` are fed to the global EKF only. They use the same parameter format as odom sources (both odom-type and imu-type are supported). Map sources support rewind-replay for delayed measurements.

```yaml
map_sources:
  - "slam"

slam:
  type: "odom"
  topic: "slam/odometry"
  pose_mask: [true, true, true, true, true, true]
  twist_mask: [false, false, false, false, false, false]
  pose_noise: [1.0e-2, 1.0e-2, 1.0e-2, 1.0e-2, 1.0e-2, 1.0e-2]
  twist_noise: [1.0e-2, 1.0e-2, 1.0e-2, 1.0e-2, 1.0e-2, 1.0e-2]
```

## TF tree

The node broadcasts the following transforms:

| Parent | Child | Type | Condition |
|---|---|---|---|
| `odom` | `base_footprint` | Dynamic (every tick) | Always |
| `map` | `odom` | Dynamic (every tick) | Only after a map source measurement is received |
| `utm` | `map` | Static (once) | Only if `topics.utm_to_map` is set and a message is received |

## Topics

### Subscriptions

| Topic | Type | QoS | Description |
|---|---|---|---|
| `<source>.topic` (odom type) | `nav_msgs/Odometry` | SensorDataQoS | Odometry source (one per configured odom-type source) |
| `<source>.topic` (imu type) | `sensor_msgs/Imu` | SensorDataQoS | IMU source (one per configured imu-type source) |
| `topics.utm_to_map` | `geometry_msgs/TransformStamped` | Reliable, transient local, depth 1 | Static UTM-to-map transform |

### Publications

| Topic | Type | QoS | Description |
|---|---|---|---|
| `topics.odom` | `nav_msgs/Odometry` | Default (depth 10) | Fused odometry from the local EKF (odom frame, base_footprint child). Includes pose and twist. |

## Services

### PredictRelativeTransform

**Name:** value of `topics.predict_service` (default: `transform/predict_relative`)

**Type:** `eidos_msgs/srv/PredictRelativeTransform`

**Request:**
- `float64 timestamp_from` -- start time (seconds)
- `float64 timestamp_to` -- end time (seconds)

**Response:**
- `geometry_msgs/Pose relative_pose` -- predicted relative motion
- `bool success` -- false if `timestamp_to - timestamp_from` is negative

The service reads the current local EKF velocity (under the sources mutex) and integrates forward by `dt = timestamp_to - timestamp_from` using the Lie group exponential map (`Pose3::Expmap(velocity * dt)`) to produce a relative transform. This is a constant-velocity dead-reckoning prediction using the velocity estimate at the time the service is called. Used for motion prediction between SLAM keyframes.

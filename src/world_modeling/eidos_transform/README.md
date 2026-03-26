# eidos_transform

EKF-based multi-source odometry fusion and TF broadcasting for ROS 2.

`eidos_transform` runs as a standalone lifecycle node, separate from eidos SLAM. It fuses one or more `nav_msgs/Odometry` sources through a pluggable EKF model, broadcasts the resulting TF tree, and publishes a fused odometry topic. It also accepts map-frame corrections (e.g. from SLAM) and an optional static UTM-to-map transform.

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
| `topics.map_source` | string | `"slam/pose"` | `PoseStamped` topic providing the robot pose in the map frame (e.g. SLAM output). Used to compute the map-to-odom correction. Set to `""` to disable. |
| `topics.utm_to_map` | string | `""` | `TransformStamped` topic for the static UTM-to-map transform. Set to `""` to disable. |
| `topics.predict_service` | string | `"transform/predict_relative"` | Service name for `PredictRelativeTransform` |

### EKF model plugin

| Parameter | Type | Default | Description |
|---|---|---|---|
| `ekf.plugin` | string | `"eidos_transform::HolonomicEKF"` | Fully qualified plugin class name |
| `ekf.name` | string | `"holonomic_ekf"` | Instance name (used to namespace plugin-specific parameters) |

The HolonomicEKF plugin reads process noise under `<ekf.name>.process_noise`:

| Parameter | Type | Default | Description |
|---|---|---|---|
| `holonomic_ekf.process_noise` | double[12] | `[1e-4]*6 + [1e-2]*6` | 12 diagonal process noise values: 6 for pose (rx, ry, rz, tx, ty, tz) then 6 for velocity (angular_x, angular_y, angular_z, linear_x, linear_y, linear_z) |

### Odometry sources

Sources are listed by name in `odom_sources`, then each name gets its own parameter block:

```yaml
odom_sources:
  - "liso"

liso:
  odom_topic: "liso/odometry"
  pose_mask: [true, true, true, true, true, true]
  twist_mask: [true, true, true, true, true, true]
  pose_noise: [1.0e-3, 1.0e-3, 1.0e-3, 1.0e-3, 1.0e-3, 1.0e-3]
  twist_noise: [1.0e-2, 1.0e-2, 1.0e-2, 1.0e-2, 1.0e-2, 1.0e-2]
```

| Parameter | Type | Default | Description |
|---|---|---|---|
| `<name>.odom_topic` | string | `""` | `nav_msgs/Odometry` topic to subscribe to (SensorDataQoS) |
| `<name>.pose_mask` | bool[6] | `[false]*6` | Which pose DOFs to fuse: `[rx, ry, rz, tx, ty, tz]` |
| `<name>.twist_mask` | bool[6] | `[false]*6` | Which twist DOFs to fuse: `[angular_x, angular_y, angular_z, linear_x, linear_y, linear_z]` |
| `<name>.pose_noise` | double[6] | `[1e-2]*6` | Standard deviations for each pose DOF |
| `<name>.twist_noise` | double[6] | `[1e-2]*6` | Standard deviations for each twist DOF |

## TF tree

The node broadcasts the following transforms:

| Parent | Child | Type | Condition |
|---|---|---|---|
| `odom` | `base_footprint` | Dynamic (every tick) | Always |
| `map` | `odom` | Dynamic (every tick) | Only after a map source message is received |
| `utm` | `map` | Static | Only if `topics.utm_to_map` is set and a message is received |

The `map -> odom` correction is computed as:

```
map_to_odom = map_to_base * inverse(odom_to_base)
```

where `map_to_base` comes from the map source topic and `odom_to_base` is the current EKF pose.

## Topics

### Subscriptions

| Topic | Type | QoS | Description |
|---|---|---|---|
| `<source>.odom_topic` | `nav_msgs/Odometry` | SensorDataQoS | Odometry source (one per configured source) |
| `topics.map_source` | `geometry_msgs/PoseStamped` | SensorDataQoS | Robot pose in map frame (e.g. from SLAM) |
| `topics.utm_to_map` | `geometry_msgs/TransformStamped` | Reliable, transient local, depth 1 | Static UTM-to-map transform |

### Publications

| Topic | Type | QoS | Description |
|---|---|---|---|
| `topics.odom` | `nav_msgs/Odometry` | Default (depth 10) | Fused odometry (odom frame, base_footprint child) |

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

The service integrates the current EKF velocity forward by `dt = timestamp_to - timestamp_from` using the Lie group exponential map to produce a relative transform. This is used for dead-reckoning queries (e.g. to predict motion between two SLAM keyframes).

## Architecture

`eidos_transform` is independent from eidos SLAM. SLAM (or any other localizer) feeds corrections into `eidos_transform` via the map source topic. The transform node owns the TF tree and the fused odometry; SLAM owns the map-frame pose estimate.

Typical data flow:

```
sensor odometry --> [eidos_transform EKF] --> odom->base_link TF + /odom topic
SLAM pose       --> [eidos_transform]     --> map->odom TF
GPS/UTM         --> [eidos_transform]     --> utm->map static TF
```

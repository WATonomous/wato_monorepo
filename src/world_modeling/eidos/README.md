# Eidos

Eidos is a plugin-based LiDAR-Inertial SLAM and localization system for ROS 2. It builds maps from sensor data (mapping mode) and localizes against previously built maps (localization mode). The plugin architecture lets you swap sensors and add constraints -- factor plugins, relocalization plugins, and visualization plugins -- without modifying core code. Under the hood, Eidos uses GTSAM's ISAM2 incremental optimizer to maintain a factor graph of vehicle poses, with each plugin contributing factors, latching to existing factors, or consuming the optimized state.

Eidos is pure SLAM: factor graph, ISAM2 optimization, plugin management, and map persistence. It does not broadcast TF or run an EKF. TF broadcasting (`map->odom`, `odom->base_link`, `utm->map`) and EKF-based odometry fusion are handled by the separate `eidos_transform` package. Eidos publishes `slam/pose` for `eidos_transform` to consume.

## Quick Start

Eidos ships example configs and a standalone launch file. In a typical deployment, both `eidos` and `eidos_transform` are launched together via `world_model.launch.yaml`, which handles this automatically.

**Mapping (live)**

```bash
# world_model.launch.yaml launches both eidos and eidos_transform
ros2 launch eidos eidos.launch.yaml
```

**Localization (live):**

```bash
# Specify a configuration for localization and a path to a map
ros2 launch eidos eidos.launch.yaml \
  eidos_config_file:=$(ros2 pkg prefix eidos)/share/eidos/config/example_localization.yaml \
  eidos_map_path:=/path/to/map.map \
```

**Mapping (from bag):**

```bash
# Specify to use a simulated clock (from bag)
ros2 launch eidos eidos.launch.yaml use_sim_time:=true

# In a separate terminal
ros2 bag play /path/to/bag --clock
```

**Localization (from bag):**

```bash
# Specify to use a simulated clock (from bag)
ros2 launch eidos eidos.launch.yaml \
  eidos_config_file:=$(ros2 pkg prefix eidos)/share/eidos/config/example_localization.yaml \
  eidos_map_path:=/path/to/map.map \
  use_sim_time:=true

# In a separate terminal
ros2 bag play /path/to/bag --clock
```

The example configs are in `config/example_slam.yaml` and `config/example_localization.yaml`. Copy and adjust topic names, sensor frames, and tuning parameters for your platform.

## Mapping vs Localization

Eidos does not have a global "mode" switch. Mapping and localization are different **configurations** of the same plugin set. Each plugin has its own parameters that control whether it adds factors to the graph, where it sources its submap from, and so on. The two example configs (`example_slam.yaml` and `example_localization.yaml`) set these per-plugin parameters to achieve the desired behavior.

**Mapping** configures plugins to build a new map. Factor plugins add constraints to the graph (`add_factors: true`), the optimizer runs each tick, loop closures are set to occur, and the result is saved to a `.map` file. Eidos publishes the optimized `slam/pose` each tick; `eidos_transform` subscribes to this and updates the `map->odom` transform accordingly.

**Localization** reconfigures the same plugins to track against a prior map. Factor plugins produce odometry only (`add_factors: false`) and match against stored data from the prior map rather than building new submaps. The `map->odom` transform is set once from relocalization by `eidos_transform` and stays fixed. We chose this behavior because large jumps only occur during loop closures, which only occur during active mapping.

## Configuration

All parameters live under the `/**/eidos_node/ros__parameters` namespace. The two config files (`example_slam.yaml` and `example_localization.yaml`) share the same parameter schema but differ in values.

### Frames

| Parameter | SLAM Default | Localization Default | Description |
|---|---|---|---|
| `frames.base_link` | `"base_footprint"` | `"base_footprint"` | Robot body frame |
| `frames.odometry` | `"odom"` | `"odom"` | Odometry frame |
| `frames.map` | `"map"` | `"map"` | Map frame (global) |

### Core

| Parameter | SLAM Default | Localization Default | Description |
|---|---|---|---|
| `slam_rate` | `10.0` | `10.0` | Main SLAM loop frequency (Hz) |
| `relocalization_timeout` | `30.0` | `30.0` | Seconds to wait for relocalization before giving up |
| `odom_pose_cov` | `[1e-3, 1e-3, 1e-3, 1e-4, 1e-4, 1e-4]` | `[1e-3, 1e-3, 1e-3, 1e-4, 1e-4, 1e-4]` | Covariance diagonal published on the odometry topic |

### ISAM2 Optimizer

| Parameter | SLAM Default | Localization Default | Description |
|---|---|---|---|
| `isam2.update_iterations` | `2` | (not set) | ISAM2 update iterations per optimization step |
| `isam2.correction_iterations` | `5` | (not set) | Extra iterations when a correction factor (e.g. GPS) is added |
| `isam2.loop_closure_iterations` | `5` | (not set) | Extra iterations when loop closure is detected |
| `isam2.relinearize_threshold` | `0.01` | `0.01` | Variable relinearization threshold |
| `isam2.relinearize_skip` | `1` | `1` | Steps between relinearization checks |

### Prior

| Parameter | SLAM Default | Localization Default | Description |
|---|---|---|---|
| `prior.pose_cov` | `[1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2]` | `[1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2]` | Prior factor covariance on the first state [x, y, z, roll, pitch, yaw] |

### Map

| Parameter | SLAM Default | Localization Default | Description |
|---|---|---|---|
| `map.load_path` | `""` | `"/opt/watonomous/maps/wrestrc.map"` | Path to load a prior map on activation. Empty = no load. Overridden by `eidos_map_path` launch arg in localization mode. |
| `map.save_path` | `"/opt/watonomous/maps/wrestrc.map"` | `""` | Default save path used by `save_map` service when no path is specified in the request. |

### GPS Factor (`gps_factor`)

| Parameter | SLAM Default | Localization Default | Description |
|---|---|---|---|
| `gps_factor.plugin` | `"eidos::GpsFactor"` | `"eidos::GpsFactor"` | Plugin class name |
| `gps_factor.add_factors` | `true` | `false` | Whether to inject GPS factors into the graph |
| `gps_factor.gps_topic` | `"/novatel/oem7/fix"` | `"/novatel/oem7/fix"` | GPS NavSatFix input topic |
| `gps_factor.imu_topic` | `"/novatel/oem7/imu/data"` | `"/novatel/oem7/imu/data"` | IMU topic (for heading initialization) |
| `gps_factor.max_cov` | `5.0` | `5.0` | Maximum acceptable GPS covariance; fixes above this are rejected |
| `gps_factor.use_elevation` | `true` | `true` | Use GPS elevation in the factor |
| `gps_factor.min_radius` | `150.0` | `200.0` | Minimum distance from origin before adding GPS factors |
| `gps_factor.pose_cov_threshold` | `1.0` | `1.0` | Graph pose covariance threshold for GPS factor injection |
| `gps_factor.gps_cov` | `[2.0, 2.0, 4.0]` | `[2.0, 2.0, 4.0]` | GPS factor noise covariance [x, y, z] |

### LISO Factor (`liso_factor`)

LiDAR-Inertial Scan-to-submap Odometry. Matches incoming scans against a local submap using GICP.

| Parameter | SLAM Default | Localization Default | Description |
|---|---|---|---|
| `liso_factor.plugin` | `"eidos::LisoFactor"` | `"eidos::LisoFactor"` | Plugin class name |
| `liso_factor.add_factors` | `true` | `false` | Whether to inject odometry factors into the graph |
| `liso_factor.submap_source` | `"recent_keyframes"` | `"prior_map"` | Where to build the submap from. `"recent_keyframes"` = live keyframes. `"prior_map"` = loaded map. |
| `liso_factor.lidar_topic` | `"/lidar_cc/velodyne_points"` | `"/lidar_cc/velodyne_points"` | LiDAR point cloud input topic |
| `liso_factor.odom_topic` | `"liso/odometry"` | `"liso/odometry"` | Published LISO odometry topic |
| `liso_factor.odometry_incremental_topic` | `"liso/odometry_incremental"` | `"liso/odometry_incremental"` | Published incremental odometry topic |
| `liso_factor.imu_topic` | `"/novatel/oem7/imu/data"` | `"/novatel/oem7/imu/data"` | IMU input topic |
| `liso_factor.lidar_frame` | `"lidar_cc"` | `"lidar_cc"` | LiDAR frame for extrinsic lookup |
| `liso_factor.imu_frame` | `"imu_link"` | `"imu_link"` | IMU frame for extrinsic lookup |
| `liso_factor.initialization.warmup_samples` | `200` | `200` | IMU samples before LISO can produce factors |
| `liso_factor.initialization.stationary_gyr_threshold` | `0.005` | `0.005` | Gyroscope threshold for stationary detection |
| `liso_factor.scan_ds_resolution` | `0.5` | `0.5` | Voxel downsampling resolution for incoming scans (m) |
| `liso_factor.submap_ds_resolution` | `0.5` | `0.5` | Voxel downsampling resolution for the submap (m) |
| `liso_factor.num_neighbors` | `10` | `10` | KNN neighbors for GICP covariance estimation |
| `liso_factor.submap_radius` | `20.0` | `30.0` | Radius around the current pose to build the submap (m) |
| `liso_factor.max_submap_states` | `10` | `15` | Maximum number of keyframe states in the submap |
| `liso_factor.max_correspondence_distance` | `2.0` | `2.0` | Maximum GICP correspondence distance (m) |
| `liso_factor.max_iterations` | `20` | `20` | Maximum GICP iterations |
| `liso_factor.num_threads` | `16` | `16` | Threads for GICP |
| `liso_factor.min_inliers` | `50` | `50` | Minimum inlier correspondences for a valid match |
| `liso_factor.min_noise` | `0.01` | `0.01` | Minimum noise floor for the odometry factor |
| `liso_factor.min_scan_distance` | `5.0` | `5.0` | Minimum travel distance between keyframes (m) |
| `liso_factor.odom_pose_cov` | `[0.01, 0.01, 0.005, 1e-6, 1e-6, 1e-3]` | `[0.01, 0.01, 0.005, 1e-6, 1e-6, 1e-3]` | Pose covariance on published odometry |
| `liso_factor.odom_twist_cov` | `[1e-4, 1e-4, 1e-8, 1e-10, 1e-10, 1e-6]` | `[1e-4, 1e-4, 1e-8, 1e-10, 1e-10, 1e-6]` | Twist covariance on published odometry |

### Motion Model Factor (`motion_model_factor`)

Bridges temporally consecutive states created by different factor plugins. Calls `eidos_transform`'s `PredictRelativeTransform` service and produces a `BetweenFactor<Pose3>`. See [MotionModelFactor docs](docs/plugins/factors/motion_model_factor.md).

| Parameter | Default | Description |
|---|---|---|
| `motion_model_factor.plugin` | `"eidos::MotionModelFactor"` | Plugin class name |
| `motion_model_factor.predict_service` | `"predict_relative_transform"` | Service name for `eidos_transform`'s PredictRelativeTransform |
| `motion_model_factor.noise_cov` | `[1e-2, 1e-2, 1e-2, 1e-4, 1e-4, 1e-4]` | Base noise covariance [rot3, trans3], scaled by dt |

### Euclidean Distance Loop Closure Factor (SLAM only)

Detects loop closures by searching for nearby keyframes that are far apart in time, then verifies with ICP.

| Parameter | Default | Description |
|---|---|---|
| `euclidean_distance_loop_closure_factor.plugin` | `"eidos::EuclideanDistanceLoopClosureFactor"` | Plugin class name |
| `euclidean_distance_loop_closure_factor.frequency` | `1.0` | Loop closure search frequency (Hz) |
| `euclidean_distance_loop_closure_factor.search_radius` | `20.0` | Spatial search radius around current pose (m) |
| `euclidean_distance_loop_closure_factor.search_time_diff` | `80.0` | Minimum time difference between candidate keyframes (s) |
| `euclidean_distance_loop_closure_factor.search_num` | `15` | Maximum number of candidates to evaluate |
| `euclidean_distance_loop_closure_factor.min_inlier_ratio` | `0.3` | Minimum ICP inlier ratio to accept a loop closure |
| `euclidean_distance_loop_closure_factor.submap_radius` | `25.0` | Radius for building the candidate submap (m) |
| `euclidean_distance_loop_closure_factor.submap_leaf_size` | `0.4` | Voxel leaf size for candidate submap downsampling (m) |
| `euclidean_distance_loop_closure_factor.max_correspondence_distance` | `2.0` | Maximum ICP correspondence distance (m) |
| `euclidean_distance_loop_closure_factor.max_iterations` | `100` | Maximum ICP iterations |
| `euclidean_distance_loop_closure_factor.num_threads` | `16` | Threads for ICP |
| `euclidean_distance_loop_closure_factor.num_neighbors` | `10` | KNN neighbors for GICP covariance estimation |
| `euclidean_distance_loop_closure_factor.loop_closure_cov` | `[1e-4, 1e-4, 1e-8, 1e-10, 1e-10, 1e-6]` | BetweenFactor covariance for accepted loop closures |
| `euclidean_distance_loop_closure_factor.gicp_pointcloud_from` | `"liso_factor/gicp_cloud"` | Source for the pre-processed GICP point cloud |

### GPS + ICP Relocalization (`gps_icp_relocalization`)

Used on startup to determine the initial pose. Matches GPS position candidates against the prior map with ICP.

| Parameter | Default | Description |
|---|---|---|
| `gps_icp_relocalization.plugin` | `"eidos::GpsIcpRelocalization"` | Plugin class name |
| `gps_icp_relocalization.gps_topic` | `"/novatel/oem7/fix"` | GPS NavSatFix input topic |
| `gps_icp_relocalization.lidar_topic` | `"/lidar_cc/velodyne_points"` | LiDAR input topic |
| `gps_icp_relocalization.imu_topic` | `"/novatel/oem7/imu/data"` | IMU input topic |
| `gps_icp_relocalization.lidar_frame` | `"lidar_cc"` | LiDAR frame |
| `gps_icp_relocalization.imu_frame` | `"imu_link"` | IMU frame |
| `gps_icp_relocalization.gps_candidate_radius` | `30.0` | Search radius around GPS position for map candidates (m) |
| `gps_icp_relocalization.pointcloud_from` | `"liso_factor/cloud"` | Source plugin for the current LiDAR cloud |
| `gps_icp_relocalization.gps_from` | `"gps_factor"` | Source plugin for GPS UTM coordinates |
| `gps_icp_relocalization.scan_ds_resolution` | `0.5` | Voxel downsampling for the scan (m) |
| `gps_icp_relocalization.submap_leaf_size` | `0.4` | Voxel leaf size for the candidate submap (m) |
| `gps_icp_relocalization.max_correspondence_distance` | `2.0` | Maximum ICP correspondence distance (m) |
| `gps_icp_relocalization.max_icp_iterations` | `100` | Maximum ICP iterations |
| `gps_icp_relocalization.num_threads` | `16` | Threads for ICP |
| `gps_icp_relocalization.num_neighbors` | `10` | KNN neighbors for covariance estimation |
| `gps_icp_relocalization.fitness_threshold` | `0.3` | Maximum ICP fitness score to accept a relocalization |

### Keyframe Map Visualization (`keyframe_map_visualization`)

Publishes the accumulated or windowed keyframe point cloud map for visualization in RViz.

| Parameter | Default | Description |
|---|---|---|
| `keyframe_map_visualization.plugin` | `"eidos::KeyframeMapVisualization"` | Plugin class name |
| `keyframe_map_visualization.topic` | `"slam/visualization/map"` | Published point cloud topic |
| `keyframe_map_visualization.pointcloud_from` | `"liso_factor/cloud"` | Source plugin for keyframe point clouds |
| `keyframe_map_visualization.voxel_leaf_size` | `0.4` | Voxel downsampling leaf size (m) |
| `keyframe_map_visualization.publish_rate` | `1.0` | Publish rate (Hz) |
| `keyframe_map_visualization.mode` | `"accumulate"` | Visualization mode: `"accumulate"` or `"windowed"` |
| `keyframe_map_visualization.accumulate.skip_factor` | `20` | Only include every Nth keyframe in accumulate mode |
| `keyframe_map_visualization.windowed.radius` | `50.0` | Radius around current pose for windowed mode (m) |

### Factor Graph Visualization (SLAM only)

Publishes the factor graph structure as RViz markers.

| Parameter | Default | Description |
|---|---|---|
| `factor_graph_visualization.plugin` | `"eidos::FactorGraphVisualization"` | Plugin class name |
| `factor_graph_visualization.topic` | `"slam/visualization/factor_graph"` | Published marker topic |
| `factor_graph_visualization.state_scale` | `1.0` | Scale of state (node) markers |
| `factor_graph_visualization.line_width` | `0.5` | Width of factor (edge) lines |
| `factor_graph_visualization.publish_rate` | `1.0` | Publish rate (Hz) |
| `factor_graph_visualization.mode` | `"full"` | Visualization mode: `"full"` or windowed |
| `factor_graph_visualization.window_radius` | `50.0` | Radius for windowed mode (m) |

## Map Files

Eidos stores maps as SQLite databases with the `.map` extension. The map file contains keyframe poses, associated point cloud data, graph adjacency, and metadata (such as the UTM-to-map offset used by the GPS factor).

**Saving a map:**

```bash
# Via service (uses map.save_path default if filepath is empty):
ros2 service call /world_modeling/eidos_node/slam/save_map eidos_msgs/srv/SaveMap "{filepath: '/opt/watonomous/maps/wrestrc.map'}"

# Or with an empty filepath to use the configured default:
ros2 service call /world_modeling/eidos_node/slam/save_map eidos_msgs/srv/SaveMap "{filepath: ''}"
```

**Loading a map:**

```bash
# Via service:
ros2 service call /world_modeling/eidos_node/slam/load_map eidos_msgs/srv/LoadMap "{filepath: '/opt/watonomous/maps/wrestrc.map'}"

# Or via config (loaded on node activation):
# Set map.load_path in the YAML config, or pass eidos_map_path:= at launch.
```

**SQLite WAL artifacts:** When a `.map` file is open, SQLite may create `-shm` (shared memory) and `-wal` (write-ahead log) companion files next to it. These are normal and will be merged back into the main file when the database is closed cleanly. Do not delete them while the node is running. If they persist after a crash, opening the map file again will replay the WAL and recover the data.

## Topics and Services

All topics and services are published under the node namespace (default: `/world_modeling/eidos_node/`). Topic names are configurable via the `topics.*` parameters.

### Published Topics

| Topic | Type | Description |
|---|---|---|
| `slam/pose` | `geometry_msgs/msg/PoseStamped` | Current optimized pose in the map frame. Published each SLAM tick. Consumed by `eidos_transform` for TF broadcasting. |
| `slam/odometry` | `nav_msgs/msg/Odometry` | Current pose as odometry (map frame, base_link child). Includes covariance from `odom_pose_cov`. Published each SLAM tick. |
| `slam/status` | `eidos_msgs/msg/SlamStatus` | System status: current state (INITIALIZING, WARMUP, RELOCALIZING, TRACKING), state index, keyframe count, factor count, active plugins. Published every tick. |
| `slam/visualization/map` | `sensor_msgs/msg/PointCloud2` | Keyframe map point cloud for RViz (from `keyframe_map_visualization` plugin). |
| `slam/visualization/factor_graph` | `visualization_msgs/msg/MarkerArray` | Factor graph structure for RViz (from `factor_graph_visualization` plugin, SLAM only). |
| `liso/odometry` | `nav_msgs/msg/Odometry` | LISO scan-matching odometry (from `liso_factor` plugin). |
| `liso/odometry_incremental` | `nav_msgs/msg/Odometry` | LISO incremental odometry (from `liso_factor` plugin). |

### Services

| Service | Type | Description |
|---|---|---|
| `slam/save_map` | `eidos_msgs/srv/SaveMap` | Save the current map to disk. If `filepath` in the request is empty, uses `map.save_path` from config. |
| `slam/load_map` | `eidos_msgs/srv/LoadMap` | Load a map from disk at the given `filepath`. |

### State Machine

The node progresses through these states (visible in the `slam/status` topic):

1. **INITIALIZING** -- Waiting for plugins to be loaded and configured.
2. **WARMUP** -- Plugins loaded. If no prior map, transitions immediately to TRACKING.
3. **RELOCALIZING** -- Prior map loaded. Attempting to determine initial pose via relocalization plugins. Times out after `relocalization_timeout` seconds.
4. **TRACKING** -- Normal operation. Factor plugins produce constraints, the optimizer runs, and poses are published.

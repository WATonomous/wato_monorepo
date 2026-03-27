# Eidos

![alt text](.img/EIDOS.gif)

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

### Plugins

Each plugin is configured under its own YAML namespace. Full parameter documentation lives in each plugin's doc page:

**Factor Plugins:**
- [LisoFactor](docs/plugins/factors/liso_factor.md) -- LiDAR-Inertial Submap Odometry (GICP scan-to-submap matching)
- [GpsFactor](docs/plugins/factors/gps_factor.md) -- Unary GPS position constraints
- [ImuFactor](docs/plugins/factors/imu_factor.md) -- IMU preintegration with warmup gating
- [EuclideanDistanceLoopClosureFactor](docs/plugins/factors/loop_closure_factor.md) -- KD-tree + GICP loop closure (SLAM only)
- [MotionModelFactor](docs/plugins/factors/motion_model_factor.md) -- Cross-plugin BetweenFactor bridging via eidos_transform

**Relocalization Plugins:**
- [GpsIcpRelocalization](docs/plugins/relocalization/gps_icp_relocalization.md) -- GPS coarse + GICP fine alignment against prior map

**Visualization Plugins:**
- [KeyframeMapVisualization](docs/plugins/visualization/keyframe_map_visualization.md) -- Accumulated/windowed point cloud map
- [FactorGraphVisualization](docs/plugins/visualization/factor_graph_visualization.md) -- RViz MarkerArray of the pose graph (SLAM only)

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
| `slam/odometry` | `nav_msgs/msg/Odometry` | Current pose as odometry (map frame, base_link child). Includes covariance from `topics.odom_pose_cov`. Published each SLAM tick. |
| `slam/status` | `eidos_msgs/msg/SlamStatus` | System status: current state (INITIALIZING, WARMUP, RELOCALIZING, TRACKING), state index, keyframe count, factor count, active plugins. Published every tick. |
| `slam/visualization/map` | `sensor_msgs/msg/PointCloud2` | Keyframe map point cloud for RViz (from `keyframe_map_visualization` plugin). |
| `slam/visualization/factor_graph` | `visualization_msgs/msg/MarkerArray` | Factor graph structure for RViz (from `factor_graph_visualization` plugin, SLAM only). |
| `liso/odometry` | `nav_msgs/msg/Odometry` | LISO scan-matching odometry (from `liso_factor` plugin). |
| `liso/odometry_incremental` | `nav_msgs/msg/Odometry` | LISO incremental odometry (from `liso_factor` plugin). |
| `imu_factor/odometry` | `nav_msgs/msg/Odometry` | IMU preintegrated odometry with body-frame twist (from `imu_factor` plugin, if loaded). |

### Services

| Service | Type | Description |
|---|---|---|
| `slam/save_map` | `eidos_msgs/srv/SaveMap` | Save the current map to disk. If `filepath` in the request is empty, uses `map.save_path` from config. |
| `slam/load_map` | `eidos_msgs/srv/LoadMap` | Load a map from disk at the given `filepath`. |

### State Machine

The node progresses through these states (visible in the `slam/status` topic):

1. **INITIALIZING** -- Waiting for plugins to be loaded and configured.
2. **WARMUP** -- Plugins loaded. The InitSequencer polls all factor plugins' `isReady()` each tick. It will not proceed until **every** factor plugin reports ready (e.g. ImuFactor requires stationary detection and gravity alignment to complete). Once all plugins are ready: if no prior map is loaded, transitions immediately to TRACKING; if a prior map is loaded, transitions to RELOCALIZING.
3. **RELOCALIZING** -- Prior map loaded. Attempting to determine initial pose via relocalization plugins. Times out after `relocalization_timeout` seconds.
4. **TRACKING** -- Normal operation. Factor plugins produce constraints, the optimizer runs, and poses are published.

## Related Packages

- **[eidos_transform](../eidos_transform/README.md)** -- EKF-based multi-source odometry fusion and TF broadcasting. Subscribes to eidos's `slam/pose` and factor plugin odometry topics. Owns the `map->odom->base_link` TF tree.
- **[eidos_msgs](../eidos_msgs/)** -- Shared message/service definitions (`SlamStatus`, `SaveMap`, `LoadMap`, `PredictRelativeTransform`).
- **[eidos_tools](../eidos_tools/README.md)** -- CLI (`eidos`) for inspecting and exporting `.map` files. Inspect metadata, dump poses, build combined PCD maps, view graph edges.

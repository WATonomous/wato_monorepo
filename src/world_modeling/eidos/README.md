# eidos

A plugin-based LiDAR-Inertial SLAM system for ROS 2. Runs as a single node (`eidos_node`) with a plugin architecture inspired by nav2 costmap layers.

## Architecture

```
eidos_node
  SlamCore (orchestrator)
    - Timer-driven SLAM loop at configurable rate
    - Adds new state when displacement/rotation exceeds threshold
    - Collects factors from plugins at each new state
    - On startup: waits for relocalization (timeout -> fresh SLAM)
    - Publishes: path, trajectory, SlamStatus

  PoseGraph (ISAM2 wrapper)
    - Wraps GTSAM's ISAM2 incremental optimizer
    - Accepts factors + initial values, returns optimized values

  MapManager (generic keyframe data store)
    - Type-agnostic keyframe data store with plugin type registry
    - Plugins register data types at init, store/retrieve per-keyframe data
    - Manages keyframe poses (3D + 6D)
    - Saves/loads maps to disk (GTSAM graph + per-keyframe plugin data)

  Factor Plugins (loaded via pluginlib):
    LidarKEPFactor  ImuIntegrationFactor  GpsFactor  EucDistLoopClosure
    (own subs/pubs)  (own subs/pubs)       (own sub)  (background thread)

  Relocalization Plugins (loaded via pluginlib):
    GpsIcpRelocalization  (future: visual, etc.)
```

### Plugin Types

There are two plugin base classes:

**FactorPlugin** (`eidos::FactorPlugin`) -- provides GTSAM factors from sensor data. Each plugin is fully self-contained with its own subscriptions, publishers, and internal state. SlamCore does NOT broker raw data between plugins. Plugins store their own keyframe data directly in MapManager via `core_->getMapManager().addKeyframeData()`.

**RelocalizationPlugin** (`eidos::RelocalizationPlugin`) -- provides relocalization strategies for resuming SLAM against a prior map. Multiple can be loaded; the first to succeed wins.

### SLAM Loop

**INITIALIZING**: Load plugins (each plugin's `onInitialize()` registers its types with MapManager), load prior map if configured. If relocalization plugins exist and a prior map is loaded, transition to RELOCALIZING. Otherwise, transition to TRACKING.

**RELOCALIZING**: For each relocalization plugin, call `tryRelocalize()`. If one returns a pose, anchor first state with a PriorFactor and transition to TRACKING. If timeout expires, fall back to fresh SLAM.

**TRACKING**: Each cycle:
1. Call `processFrame()` on all factor plugins (collect pose estimates, plugin order = priority)
2. Check displacement/rotation since last state; if exceeded, create new state
3. On new state:
   a. SlamCore calls `MapManager::addKeyframe(index, pose)` for the pose
   b. Call `getFactors()` on all plugins (plugins store their own keyframe data in MapManager here)
   c. Add all factors to ISAM2, optimize
   d. Call `onOptimizationComplete()` on all plugins with optimized values
   e. If loop closure detected: correct all poses via `MapManager::updatePoses()`
4. Publish path and status

---

## MapManager -- Generic Keyframe Data Store

MapManager is a **type-agnostic keyframe data store**. It owns keyframe poses and provides a registry for plugins to store/retrieve arbitrary per-keyframe data. It does **not** know about point clouds, images, or any sensor-specific types.

### Type Registry

Plugins register their data types at initialization time. Each type is identified by a string key (convention: `plugin_name/data_name`) and paired with serialize/deserialize functions.

```cpp
struct TypeHandler {
  std::function<void(const std::any& data, const std::string& path)> serialize;
  std::function<std::any(const std::string& path)> deserialize;
};

// Register during onInitialize():
core_->getMapManager().registerType("my_plugin/my_data", {
  [](const std::any& data, const std::string& path) {
    auto cloud = std::any_cast<pcl::PointCloud<PointType>::Ptr>(data);
    pcl::io::savePCDFileBinary(path, *cloud);
  },
  [](const std::string& path) -> std::any {
    auto cloud = pcl::make_shared<pcl::PointCloud<PointType>>();
    pcl::io::loadPCDFile(path, *cloud);
    return cloud;
  }
});
```

### Storing Data

Plugins store their keyframe data in `getFactors()` after producing their factors:

```cpp
auto& map_manager = core_->getMapManager();
map_manager.addKeyframeData(state_index, "my_plugin/my_data", my_cloud_ptr);
```

### Retrieving Data

Consumer plugins retrieve data by key:

```cpp
// Get a single key
auto data = map_manager.getKeyframeData(index, "lidar_kep_factor/corners");
if (data.has_value()) {
  auto cloud = std::any_cast<pcl::PointCloud<PointType>::Ptr>(data.value());
}

// Get all data from a specific producer plugin
auto plugin_data = map_manager.getKeyframeDataForPlugin(index, "lidar_kep_factor");
for (const auto& [key, data] : plugin_data) {
  auto cloud = std::any_cast<pcl::PointCloud<PointType>::Ptr>(data);
  // use cloud...
}
```

### Keyframe Data Key Registry

Every factor plugin that produces sensor data MUST register its keyframe data keys. Consumer plugins reference producers by plugin name via their own `pointcloud_from` / `gps_from` parameters.

| Plugin | Key | Type | Description |
|--------|-----|------|-------------|
| `lidar_kep_factor` | `lidar_kep_factor/corners` | `pcl::PointCloud<PointType>::Ptr` | Corner features in body frame |
| `lidar_kep_factor` | `lidar_kep_factor/surfaces` | `pcl::PointCloud<PointType>::Ptr` | Surface features in body frame |
| `imu_integration_factor` | `imu_integration_factor/bias` | `gtsam::imuBias::ConstantBias` | Estimated accelerometer + gyroscope bias at keyframe time |
| `imu_integration_factor` | `imu_integration_factor/velocity` | `gtsam::Vector3` | Estimated velocity at keyframe time |
| `gps_factor` | `gps_factor/position` | `gtsam::Point3` | GPS position at keyframe time (in map frame) |
| `euclidean_distance_loop_closure_factor` | *(none -- pure consumer)* | | |
| `gps_icp_relocalization` | *(none -- pure consumer)* | | |

**This table MUST be updated whenever a new factor plugin is added or an existing plugin's keys change.**

### Consumer -> Producer Wiring

Each consumer plugin specifies which producer to read from via its own config parameters. Multiple consumers can reference the same producer independently.

| Consumer Plugin | Parameter | Example Value | What it reads |
|----------------|-----------|---------------|---------------|
| `euclidean_distance_loop_closure_factor` | `pointcloud_from` | `"lidar_kep_factor"` | All `pcl::PointCloud<PointType>::Ptr` keys from that plugin |
| `gps_icp_relocalization` | `pointcloud_from` | `"lidar_kep_factor"` | All `pcl::PointCloud<PointType>::Ptr` keys from that plugin |
| `gps_icp_relocalization` | `gps_from` | `"gps_factor"` | All `gtsam::Point3` keys from that plugin |

### Map Format on Disk

```
<map_directory>/
  metadata.yaml               # version, num_keyframes, registered keys
  graph_factors.bin            # GTSAM NonlinearFactorGraph serialized
  graph_values.bin             # GTSAM Values serialized
  trajectory.pcd               # PointXYZI keyframe positions
  transformations.pcd          # PointXYZIRPYT 6-DOF poses + timestamps
  GlobalMap.pcd                # Downsampled global point cloud (if resolution > 0)
  keyframes/
    000000/                    # Per-keyframe directory
      lidar_kep_factor/        # Subdirectory per plugin
        corners.bin            # Serialized by the plugin's registered handler
        surfaces.bin
      gps_factor/
        position.bin
    000001/
      ...
```

`metadata.yaml` includes the list of registered keys so that `loadMap()` knows what to deserialize:
```yaml
version: 1
num_keyframes: 150
registered_keys:
  - "gps_factor/position"
  - "imu_integration_factor/bias"
  - "imu_integration_factor/velocity"
  - "lidar_kep_factor/corners"
  - "lidar_kep_factor/surfaces"
```

This format preserves the full GTSAM Bayes tree, enabling incremental re-optimization on reload.

### Services

- `slam/save_map` (`eidos_msgs/srv/SaveMap`) -- save the current map to a directory
- `slam/load_map` (`eidos_msgs/srv/LoadMap`) -- load a prior map from a directory

### Published Topics

- `slam/path` (`nav_msgs/Path`) -- global trajectory
- `slam/status` (`eidos_msgs/msg/SlamStatus`) -- SLAM state, keyframe count, loop closure flag

---

## Configuration

All parameters live under `eidos_node.ros__parameters` in the YAML config file. See `config/params.yaml` for the full reference.

### Core Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `slam_rate` | float | 10.0 | SLAM loop frequency (Hz) |
| `max_displacement` | float | 1.0 | Translation threshold (m) to create a new state |
| `max_rotation` | float | 0.2 | Rotation threshold (rad) to create a new state |
| `relocalization_timeout` | float | 30.0 | Seconds before falling back to fresh SLAM |
| `frames.lidar` | string | "lidar_link" | LiDAR frame name |
| `frames.base_link` | string | "base_link" | Base link frame name |
| `frames.odometry` | string | "odom" | Odometry frame name |
| `frames.map` | string | "map" | Map frame name |
| `keyframe.density` | float | 2.0 | Voxel size for downsampling keyframe positions |
| `keyframe.search_radius` | float | 50.0 | Radius for local map assembly |
| `map.load_directory` | string | "" | Load prior map on startup (empty = no load) |
| `map.save_directory` | string | "/tmp/eidos_maps/" | Default save directory |
| `performance.num_cores` | int | 4 | Number of cores for parallel scan matching |

### Plugin Lists

```yaml
factor_plugins:          # Order = processFrame priority
  - "imu_integration_factor"
  - "lidar_kep_factor"
  - "gps_factor"
  - "euclidean_distance_loop_closure_factor"

relocalization_plugins:  # First to succeed wins
  - "gps_icp_relocalization"
```

Each plugin name maps to a parameter namespace with at least a `plugin` field specifying the pluginlib class name.

---

## Factor Plugins

### LidarKEPFactor (`eidos::LidarKEPFactor`)

Keyframe-based Edge-Plane LiDAR odometry. Performs point cloud deskewing, range image projection, LOAM-style curvature-based feature extraction (corners + surfaces), and scan-to-map matching via Levenberg-Marquardt optimization. Provides `BetweenFactor<Pose3>` odometry constraints between consecutive states.

Uses IMU angular velocities for deskewing and IMU odometry from ImuIntegrationFactor as initial guess for scan matching.

Owns the local map assembly logic: retrieves its own keyframe data from MapManager, transforms to world frame, and assembles a local submap for scan matching.

**Registered keyframe data keys:**

| Key | Type | Description |
|-----|------|-------------|
| `lidar_kep_factor/corners` | `pcl::PointCloud<PointType>::Ptr` | Corner features in body frame |
| `lidar_kep_factor/surfaces` | `pcl::PointCloud<PointType>::Ptr` | Surface features in body frame |

**Subscriptions:**
- Point cloud (`sensor_msgs/PointCloud2`)
- IMU (`sensor_msgs/Imu`) -- for deskewing
- IMU odometry (`nav_msgs/Odometry`) -- from ImuIntegrationFactor, for initial guess

**Publications:**
- `<name>/odometry` (`nav_msgs/Odometry`) -- LiDAR odometry

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `plugin` | string | -- | `"eidos::LidarKEPFactor"` |
| `point_cloud_topic` | string | "points_raw" | Point cloud topic |
| `imu_topic` | string | "imu/data" | IMU topic for deskewing |
| `imu_odom_topic` | string | "imu_integration_factor/odometry/imu_incremental" | IMU odometry for initial guess |
| `sensor_type` | string | "velodyne" | Sensor type: "velodyne", "ouster", or "livox" |
| `n_scan` | int | 32 | Number of scan lines |
| `horizon_scan` | int | 1800 | Points per scan line |
| `min_range` | float | 1.0 | Minimum point range (m) |
| `max_range` | float | 100.0 | Maximum point range (m) |
| `edge_threshold` | float | 1.0 | Curvature threshold for corner features |
| `surf_threshold` | float | 0.1 | Curvature threshold for surface features |
| `odom_surf_leaf_size` | float | 0.4 | Surface voxel size for odometry |
| `mapping_corner_leaf_size` | float | 0.2 | Corner voxel size for mapping |
| `mapping_surf_leaf_size` | float | 0.4 | Surface voxel size for mapping |
| `ring_flag` | int | 0 | Whether the point cloud has ring info (1 = yes) |
| `deskew_flag` | int | 0 | Whether the point cloud has deskew time info (1 = yes) |
| `extrinsic_rot` | float[9] | identity | IMU-to-LiDAR rotation matrix (row-major) |
| `extrinsic_rpy` | float[9] | identity | IMU-to-LiDAR RPY rotation matrix (row-major) |
| `extrinsic_trans` | float[3] | [0,0,0] | IMU-to-LiDAR translation |

### ImuIntegrationFactor (`eidos::ImuIntegrationFactor`)

IMU preintegration with internal ISAM2 optimizer for bias estimation. Runs two parallel pipelines:
- **Optimization pipeline**: Integrates IMU data between lidar corrections and estimates biases using an internal ISAM2 graph (separate from the main pose graph). This two-loop architecture is intentional and matches LIO-SAM.
- **Real-time pipeline**: Produces high-rate IMU-predicted odometry and TF broadcasts

Publishes two odometry topics:
- **Incremental**: Raw IMU-predicted odometry (subscribed to by LidarKEPFactor for scan matching initial guess)
- **Fused**: Transform fusion of latest lidar correction + IMU increment (smooth, high-rate output)

**Registered keyframe data keys:**

| Key | Type | Description |
|-----|------|-------------|
| `imu_integration_factor/bias` | `gtsam::imuBias::ConstantBias` | Estimated accelerometer + gyroscope bias at keyframe time |
| `imu_integration_factor/velocity` | `gtsam::Vector3` | Estimated velocity at keyframe time |

**Subscriptions:**
- IMU (`sensor_msgs/Imu`)

**Publications:**
- `<name>/odometry/imu_incremental` (`nav_msgs/Odometry`) -- raw IMU prediction
- `<name>/odometry/imu` (`nav_msgs/Odometry`) -- fused (lidar-corrected + IMU increment)
- TF: `odom` -> `base_link`

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `plugin` | string | -- | `"eidos::ImuIntegrationFactor"` |
| `imu_topic` | string | "imu/data" | IMU topic |
| `acc_noise` | float | 3.99e-03 | Accelerometer white noise |
| `gyr_noise` | float | 1.56e-03 | Gyroscope white noise |
| `acc_bias_noise` | float | 6.44e-05 | Accelerometer bias random walk |
| `gyr_bias_noise` | float | 3.56e-05 | Gyroscope bias random walk |
| `gravity` | float | 9.80511 | Gravity magnitude (m/s^2) |
| `odom_frame` | string | "odom" | Odometry frame for TF |
| `base_link_frame` | string | "base_link" | Base link frame for TF |
| `lidar_frame` | string | "lidar_link" | LiDAR frame |
| `extrinsic_rot` | float[9] | identity | IMU extrinsic rotation |
| `extrinsic_rpy` | float[9] | identity | IMU extrinsic RPY rotation |
| `extrinsic_trans` | float[3] | [0,0,0] | IMU extrinsic translation |

### GpsFactor (`eidos::GpsFactor`)

Adds GPS constraints to the pose graph when GPS data has low covariance. Skips injection when too close to the last GPS constraint. Returns `GPSFactor` at each new state if GPS is available.

**Registered keyframe data keys:**

| Key | Type | Description |
|-----|------|-------------|
| `gps_factor/position` | `gtsam::Point3` | GPS position at keyframe time (in map frame) |

**Subscriptions:**
- GPS odometry (`nav_msgs/Odometry`)

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `plugin` | string | -- | `"eidos::GpsFactor"` |
| `gps_topic` | string | "gps/odometry" | GPS odometry topic |
| `cov_threshold` | float | 2.0 | Max position covariance to accept GPS |
| `use_elevation` | bool | false | Include elevation (z) in GPS factor |

### EuclideanDistanceLoopClosureFactor (`eidos::EuclideanDistanceLoopClosureFactor`)

Detects loop closures using Euclidean distance-based candidate search and ICP validation. Runs a background thread that periodically scans for candidate loops by performing KD-tree radius search on keyframe positions, requiring a minimum time difference. Validates candidates via ICP alignment of submaps assembled from the `pointcloud_from` producer plugin's keyframe data, and queues validated `BetweenFactor<Pose3>` constraints.

**Registered keyframe data keys:** None (pure consumer).

**Consumer parameters:**
- `pointcloud_from`: Which plugin's keyframe point clouds to use for ICP. Uses `MapManager::getKeyframeDataForPlugin(index, pointcloud_from)` to discover and retrieve all point cloud entries.

**No subscriptions.** Reads `MapManager` via `core_`.

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `plugin` | string | -- | `"eidos::EuclideanDistanceLoopClosureFactor"` |
| `pointcloud_from` | string | "lidar_kep_factor" | Producer plugin for point cloud data |
| `frequency` | float | 1.0 | Loop detection frequency (Hz) |
| `search_radius` | float | 15.0 | Radius for candidate search (m) |
| `search_time_diff` | float | 30.0 | Minimum time gap between current and candidate (s) |
| `search_num` | int | 25 | Number of neighbor keyframes for submap |
| `fitness_score` | float | 0.3 | ICP fitness threshold for validation |

---

## Relocalization Plugins

### GpsIcpRelocalization (`eidos::GpsIcpRelocalization`)

Relocalizes against a loaded prior map using GPS + ICP. During the RELOCALIZING state, uses the latest GPS fix to find candidate keyframes in the prior map via KD-tree radius search, assembles a local submap from the `pointcloud_from` producer plugin's keyframe data, runs ICP alignment, and returns the relocalized pose if the fitness score is below threshold.

**Registered keyframe data keys:** None (pure consumer).

**Consumer parameters:**
- `pointcloud_from`: Which plugin's keyframe point clouds to use for ICP submap assembly
- `gps_from`: Which plugin's stored GPS positions to use for candidate keyframe search

**Subscriptions:**
- GPS odometry (`nav_msgs/Odometry`)

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `plugin` | string | -- | `"eidos::GpsIcpRelocalization"` |
| `pointcloud_from` | string | "lidar_kep_factor" | Producer plugin for point cloud data |
| `gps_from` | string | "gps_factor" | Producer plugin for GPS positions |
| `gps_topic` | string | "gps/odometry" | GPS odometry topic |
| `gps_candidate_radius` | float | 30.0 | Radius for candidate keyframe search (m) |
| `fitness_threshold` | float | 0.3 | ICP fitness threshold |
| `max_icp_iterations` | int | 100 | Maximum ICP iterations |

---

## Adding New Factor Plugins

### Within the eidos package

1. Create header `include/eidos/plugins/my_factor.hpp` with a class inheriting from `eidos::FactorPlugin`
2. Create source `src/plugins/my_factor.cpp` implementing all pure virtual methods
3. In `onInitialize()`, register keyframe data types with MapManager:
   ```cpp
   auto& map_manager = core_->getMapManager();
   map_manager.registerType("my_factor/my_data", {
     [](const std::any& data, const std::string& path) {
       // serialize data to path
     },
     [](const std::string& path) -> std::any {
       // deserialize and return data from path
     }
   });
   ```
4. In `getFactors()`, store keyframe data:
   ```cpp
   core_->getMapManager().addKeyframeData(state_index, "my_factor/my_data", my_data);
   ```
5. At the bottom of the `.cpp`, register with pluginlib:
   ```cpp
   #include <pluginlib/class_list_macros.hpp>
   PLUGINLIB_EXPORT_CLASS(eidos::MyFactor, eidos::FactorPlugin)
   ```
6. Add the class to `factor_plugins.xml`:
   ```xml
   <class name="eidos::MyFactor"
          type="eidos::MyFactor"
          base_class_type="eidos::FactorPlugin">
     <description>My custom factor plugin.</description>
   </class>
   ```
7. Add the source file to `eidos_plugins` in `CMakeLists.txt`
8. Add plugin config in `params.yaml`:
   ```yaml
   factor_plugins:
     - "my_factor"
   my_factor:
     plugin: "eidos::MyFactor"
     # plugin-specific params...
   ```
9. Update the Keyframe Data Key Registry table in this README

### As a separate package

1. Create a new ROS 2 package with `ament_cmake` and depend on `eidos` (for the `FactorPlugin` base class) and `pluginlib`
2. Implement your plugin class inheriting from `eidos::FactorPlugin`
3. Register with `PLUGINLIB_EXPORT_CLASS`
4. Create your own `<plugin_name>_plugins.xml` and export it:
   ```cmake
   pluginlib_export_plugin_description_file(eidos my_plugins.xml)
   ```
5. Add to the `factor_plugins` list in the eidos config YAML

### How Consumer Plugins Resolve Keys

Consumer plugins (loop closure, relocalization) use the `pointcloud_from` / `gps_from` parameters to specify which producer plugin to read from. At runtime:

1. The consumer calls `MapManager::getKeyframeDataForPlugin(index, pointcloud_from_)` which returns all key-value pairs where the key starts with `pointcloud_from_ + "/"`.
2. The consumer iterates over the returned data and `std::any_cast`s each entry to the expected type (e.g., `pcl::PointCloud<PointType>::Ptr`).
3. Non-matching types are silently skipped via `try/catch` on `std::bad_any_cast`.

This allows a consumer to work with any producer plugin without hardcoding key names, as long as the producer stores point cloud data.

### FactorPlugin Interface

```cpp
class FactorPlugin {
public:
  virtual void onInitialize() = 0;           // Create subs/pubs, declare params, register types
  virtual void activate() = 0;               // Start processing
  virtual void deactivate() = 0;             // Stop processing
  virtual void reset() = 0;                  // Reset internal state

  // Called every SLAM cycle. Return a pose estimate if available.
  virtual std::optional<gtsam::Pose3> processFrame(double timestamp) = 0;

  // Called when a new state is created. Return GTSAM factors.
  // MUST also store keyframe data in MapManager via addKeyframeData().
  virtual std::vector<gtsam::NonlinearFactor::shared_ptr> getFactors(
      int state_index, const gtsam::Pose3& state_pose, double timestamp) = 0;

  // Called after ISAM2 optimization (optional override).
  virtual void onOptimizationComplete(
      const gtsam::Values& optimized_values, bool loop_closure_detected) {}

protected:
  SlamCore* core_;                           // Access to pose graph, map manager
  std::string name_;                         // Plugin instance name
  rclcpp::Node::SharedPtr node_;             // Shared node for subs/pubs/params
  tf2_ros::Buffer* tf_;                      // TF buffer
  rclcpp::CallbackGroup::SharedPtr callback_group_;
};
```

---

## Adding New Relocalization Plugins

### Within the eidos package

1. Create header `include/eidos/plugins/my_reloc.hpp` inheriting from `eidos::RelocalizationPlugin`
2. Create source `src/plugins/my_reloc.cpp` implementing all pure virtual methods
3. Register with pluginlib:
   ```cpp
   PLUGINLIB_EXPORT_CLASS(eidos::MyReloc, eidos::RelocalizationPlugin)
   ```
4. Add to `relocalization_plugins.xml`
5. Add source to `eidos_plugins` in `CMakeLists.txt`
6. Add plugin config in `params.yaml`

### As a separate package

Same approach as factor plugins -- depend on `eidos` for the base class, export your own plugin XML with `pluginlib_export_plugin_description_file(eidos ...)`.

### How to Use `pointcloud_from` / `gps_from`

Relocalization plugins that need to consume data from factor plugins should:

1. Declare and read the `pointcloud_from` / `gps_from` parameters in `onInitialize()`
2. In `tryRelocalize()`, call `map_manager.getKeyframeDataForPlugin(index, pointcloud_from_)` to get all data entries from the producer
3. `std::any_cast` each entry to the expected type
4. Handle `std::bad_any_cast` gracefully (skip non-matching types)

### RelocalizationPlugin Interface

```cpp
struct RelocalizationResult {
  gtsam::Pose3 pose;
  double fitness_score;
  int matched_keyframe_index;
};

class RelocalizationPlugin {
public:
  virtual void onInitialize() = 0;
  virtual void activate() = 0;
  virtual void deactivate() = 0;

  // Called repeatedly during RELOCALIZING state.
  // Return a pose if relocalization succeeds, std::nullopt if still searching.
  virtual std::optional<RelocalizationResult> tryRelocalize(double timestamp) = 0;

protected:
  SlamCore* core_;
  std::string name_;
  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer* tf_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
};
```

---

## Design Notes

1. **Plugins store data directly in MapManager** -- they call `core_->getMapManager().addKeyframeData()` in their `getFactors()` method. SlamCore does NOT relay, broker, or touch keyframe data. It only calls `MapManager::addKeyframe()` for the pose.

2. **Every factor plugin registers something** -- if a factor plugin processes sensor data, it should store that data (or a derived product) in MapManager. This ensures the map is self-contained and can be reloaded/used by any consumer plugin combination.

3. **Consumer parameters are per-plugin** -- each consumer specifies its own `pointcloud_from` / `gps_from` independently. If two consumers both need point clouds, they each have their own `pointcloud_from` parameter (they can point to the same or different producers).

4. **The two ISAM2 loops are intentional** -- ImuIntegrationFactor has its own internal ISAM2 optimizer for bias estimation, separate from the main SLAM pose graph. This matches LIO-SAM's architecture where IMU preintegration has its own optimization loop.

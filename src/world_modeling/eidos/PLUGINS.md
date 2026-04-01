# Eidos Plugins

Eidos uses a pluginlib-based architecture where each plugin is self-contained: it manages
its own subscriptions, publishers, and internal state. Plugins are loaded from XML
descriptors and instantiated at runtime based on YAML configuration.

## Plugin Types

| Type | Base Class | Purpose |
|---|---|---|
| Factor | `eidos::FactorPlugin` | Produce GTSAM factors from sensor data for the SLAM graph. |
| Relocalization | `eidos::RelocalizationPlugin` | Determine initial pose against a prior map. First plugin to succeed wins. |
| Visualization | `eidos::VisualizationPlugin` | Read-only rendering of optimized state to RViz topics on an independent timer. |

## FactorPlugin API

Base class: `eidos::FactorPlugin` (`plugins/base_factor_plugin.hpp`).

### Lifecycle

| Method | When Called | Purpose |
|---|---|---|
| `onInitialize()` | Once, during plugin loading | Declare params, create subs/pubs, register map formats. |
| `activate()` | Node transitions to ACTIVE | Enable subscriptions and processing. |
| `deactivate()` | Node transitions to INACTIVE | Stop publishing and processing. |

### SLAM Loop Hooks

| Method | Signature | Purpose |
|---|---|---|
| `produceFactor()` | `(Key key, double timestamp) -> StampedFactorResult` | Return factors to create a new state (set `result.timestamp`) or empty if no new data. Only state-creating plugins (e.g. LISO) override this. |
| `latchFactor()` | `(Key key, double timestamp) -> StampedFactorResult` | Attach additional factors to a newly created state (e.g. GPS unary, loop closure). Return empty if nothing to attach. |
| `isReady()` | `() const -> bool` | Whether the plugin has warmed up. Default: true. Plugins with sensor warmup (e.g. LISO gates on IMU stationarity detection) override to return false until ready. InitSequencer blocks in WARMING_UP until all plugins report ready. |
| `onTrackingBegin()` | `(const Pose3 & pose)` | Called when transitioning to TRACKING. Use to initialize against a prior map (e.g. build initial submap). |
| `onOptimizationComplete()` | `(const Values & values, bool graph_corrected)` | Called after ISAM2 optimization. `graph_corrected` is true when a loop closure or correction factor was present. Use to re-anchor internal state (e.g. LISO applies a pending correction to `prev_incremental_pose_` and triggers submap rebuild). |

### Lock-Free Pose Outputs

Factor plugins expose `getMapPose()` and `getOdomPose()` (read from any thread). Sensor
callbacks write via `setMapPose()` / `setOdomPose()`. These use a seqlock (`LockFreePose`)
internally -- no mutex contention.

## Plugin Documentation

### [Factor Plugins](docs/plugins/factors/README.md)

- [LisoFactor](docs/plugins/factors/liso_factor.md) -- LiDAR-Inertial Submap Odometry (GICP scan-to-submap matching)
- [GpsFactor](docs/plugins/factors/gps_factor.md) -- Unary GPS position constraints
- [EuclideanDistanceLoopClosureFactor](docs/plugins/factors/loop_closure_factor.md) -- KD-tree + GICP loop closure detection
- [ImuFactor](docs/plugins/factors/imu_factor.md) -- IMU preintegration factor
- [MotionModelFactor](docs/plugins/factors/motion_model_factor.md) -- Cross-plugin BetweenFactor bridging via eidos_transform predict service

### [Relocalization Plugins](docs/plugins/relocalization/README.md)

- [GpsIcpRelocalization](docs/plugins/relocalization/gps_icp_relocalization.md) -- GPS coarse + GICP fine alignment against prior map

### [Visualization Plugins](docs/plugins/visualization/README.md)

- [KeyframeMapVisualization](docs/plugins/visualization/keyframe_map_visualization.md) -- Accumulated/windowed point cloud map
- [FactorGraphVisualization](docs/plugins/visualization/factor_graph_visualization.md) -- RViz MarkerArray of the pose graph

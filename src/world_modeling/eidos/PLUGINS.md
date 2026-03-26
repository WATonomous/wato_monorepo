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

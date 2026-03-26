# LisoFactor

**Class:** `eidos::LisoFactor`
**Type:** Factor (state-creating via `produceFactor`)
**XML:** `factor_plugins.xml`

LiDAR-Inertial Submap Odometry. Subscribes to LiDAR and IMU, performs GICP scan-to-submap matching to produce `BetweenFactor<Pose3>` entries for the SLAM graph. Writes both map-frame and odom-frame poses lock-free. Publishes odometry on dedicated topics. Registers `liso_factor/cloud` (PCL PCD) and `liso_factor/gicp_cloud` (small_gicp binary) with MapManager for persistence.

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `lidar_topic` | string | `"/lidar/points"` | Input LiDAR PointCloud2 topic. |
| `odom_topic` | string | `"liso/odometry"` | Published map-frame odometry topic. |
| `odometry_incremental_topic` | string | `"liso/odometry_incremental"` | Published odom-frame incremental odometry topic. |
| `add_factors` | bool | `true` | Whether to produce factors for the SLAM graph. |
| `submap_source` | string | `"recent_keyframes"` | Submap assembly mode: `"recent_keyframes"` or `"prior_map"`. |
| `scan_ds_resolution` | double | `0.5` | Voxel downsample resolution for incoming scans (meters). |
| `submap_ds_resolution` | double | `0.5` | Voxel downsample resolution for submap assembly (meters). |
| `num_neighbors` | int | `10` | Number of neighbors for normal/covariance estimation. |
| `submap_radius` | double | `20.0` | Spatial radius for collecting keyframes into the submap (meters). |
| `max_submap_states` | int | `10` | Maximum number of keyframe states in the submap. |
| `max_correspondence_distance` | double | `2.0` | GICP max correspondence distance (meters). |
| `max_iterations` | int | `20` | GICP max iterations. |
| `num_threads` | int | `16` | Thread count for GICP and preprocessing. |
| `min_inliers` | int | `50` | Minimum GICP inlier count to accept a match. |
| `min_noise` | double | `0.01` | Minimum noise floor for the between-factor covariance. |
| `min_scan_distance` | double | `5.0` | Minimum travel distance (meters) before creating a new factor. |
| `odom_pose_cov` | double[6] | `[0.01, 0.01, 0.005, 1e-6, 1e-6, 1e-3]` | Diagonal pose covariance for published odometry and BetweenFactor noise. |
| `odom_twist_cov` | double[6] | `[1e-4, 1e-4, 1e-8, 1e-10, 1e-10, 1e-6]` | Diagonal twist covariance for published odometry. |
| `lidar_frame` | string | `"lidar"` | TF frame of the LiDAR sensor. |
| `imu_topic` | string | `"/imu/data"` | IMU topic for gyro-based initial guess and warmup detection. |
| `imu_frame` | string | `"imu_link"` | TF frame of the IMU sensor. |
| `initialization.warmup_samples` | int | `200` | Number of IMU samples required for warmup. |
| `initialization.stationary_gyr_threshold` | double | `0.005` | Gyroscope RMS threshold for stationary detection during warmup. |

## Notes

- LISO is the only built-in plugin that creates new states via `produceFactor()`.
- Implements `onTrackingBegin()` to build an initial submap from the prior map at the relocalized position.
- Implements `onOptimizationComplete()` to re-anchor the GICP initial guess and trigger submap rebuilds after graph corrections (especially loop closures).
- IMU is used only for gyro integration (initial guess rotation) and warmup detection, not for preintegration factors.

## Mapping vs Localization

| Parameter | Mapping | Localization |
|---|---|---|
| `add_factors` | `true` | `false` |
| `submap_source` | `"recent_keyframes"` | `"prior_map"` |
| `submap_radius` | `20.0` | `30.0` |
| `max_submap_states` | `10` | `15` |

In **mapping**, LISO builds submaps from recent keyframes in the current session and produces `BetweenFactor` constraints that grow the pose graph.

In **localization**, LISO matches scans against point clouds stored in the prior `.map` file. It does not produce factors -- it only provides odom-frame and map-frame pose estimates via lock-free pose slots. The submap radius and state count are larger to provide a broader matching context against the prior map. On `onTrackingBegin()`, an initial submap is built at the relocalized position from the prior map's stored clouds.

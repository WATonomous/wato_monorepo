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
| `min_range` | double | `0.0` | Drop points closer than this to the LiDAR (meters), e.g. returns from the ego vehicle. `0` keeps every point. |
| `keyframe_cloud_ds_resolution` | double | `0.0` | Voxel size (meters) of the `liso_factor/cloud` stored per keyframe. `0` stores the full scan; set it for dense LiDARs to bound RAM and map size. |
| `odom_pose_cov` | double[6] | `[0.01, 0.01, 0.005, 1e-6, 1e-6, 1e-3]` | Diagonal pose covariance for published odometry and BetweenFactor noise. |
| `odom_twist_cov` | double[6] | `[1e-4, 1e-4, 1e-8, 1e-10, 1e-10, 1e-6]` | Diagonal twist covariance for published odometry. |
| `lidar_frame` | string | `"lidar"` | TF frame of the LiDAR sensor. |
| `imu_topic` | string | `"/imu/data"` | IMU topic for gyro-based initial guess and warmup detection. |
| `imu_frame` | string | `"imu_link"` | TF frame of the IMU sensor. |
| `initialization.warmup_samples` | int | `200` | Number of IMU samples required for warmup. |
| `initialization.stationary_gyr_threshold` | double | `0.005` | Gyroscope RMS threshold for stationary detection during warmup. |

## `isReady()` and IMU Warmup

LisoFactor overrides `isReady()` to return `false` until IMU warmup is complete (`imu_warmup_complete_` is set). This gates the `InitSequencer` state machine alongside any other plugins that require warmup (e.g. ImuFactor). IMU warmup performs stationary detection and gravity alignment identically to ImuFactor's warmup: after `warmup_samples` consecutive stationary IMU samples, a gravity-aligned initial orientation (roll + pitch, yaw = 0) is computed and used to seed the first scan's pose.

## Notes

- LISO is the only built-in plugin that creates new states via `produceFactor()`.
- Implements `onTrackingBegin()` to build an initial submap from the prior map at the relocalized position.
- Implements `onOptimizationComplete(values, graph_corrected)` to re-anchor the GICP initial guess (see below) and trigger submap rebuilds when `graph_corrected` is true (loop closure or GPS correction). A pending incremental correction is queued for the sensor thread to apply to `prev_incremental_pose_` on the next LiDAR callback, avoiding a data race between the SLAM and sensor threads.
- Published twist is in body frame, derived from the `between()` delta of consecutive matched poses divided by dt.
- IMU is used only for gyro integration (initial guess rotation) and warmup detection, not for preintegration factors.
- The IMU subscription has its own callback group and a 1000-message queue, so IMU messages keep arriving while a scan is being matched. The guess rotation integrates every buffered gyro sample between the last match and the new scan. Before this, the IMU shared the plugin's mutually exclusive group with a 5-message queue, so any scan that took longer than five IMU periods to match dropped the rest of the gyro data for that interval.
- The throttled `GICP match` log and every `GICP failed` log report GICP iterations and the scan's processing time (ms), so throughput can be read from `launch.log`. Failures and keyframes also log how far GICP moved from the initial guess and the eigenvalues of the translation block of GICP's information matrix (a small minimum, with its body-frame direction, marks a direction the scan barely constrains).

## Mapping vs Localization

| Parameter | Mapping | Localization |
|---|---|---|
| `add_factors` | `true` | `false` |
| `submap_source` | `"recent_keyframes"` | `"prior_map"` |
| `submap_radius` | `20.0` | `30.0` |
| `max_submap_states` | `10` | `15` |

In **mapping**, LISO builds submaps from recent keyframes in the current session and produces `BetweenFactor` constraints that grow the pose graph.

In **localization**, LISO matches scans against point clouds stored in the prior `.map` file. It does not produce factors -- it only provides odom-frame and map-frame pose estimates via lock-free pose slots. The submap radius and state count are larger to provide a broader matching context against the prior map. On `onTrackingBegin()`, an initial submap is built at the relocalized position from the prior map's stored clouds.

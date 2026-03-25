# GpsIcpRelocalization

**Class:** `eidos::GpsIcpRelocalization`
**XML:** `relocalization_plugins.xml`

Combines GPS and GICP to relocalize against a prior map. Subscribes to live GPS, LiDAR, and IMU. Uses the GPS fix to find candidate keyframes near the estimated position (using the saved `utm_to_map` offset from the prior map), assembles a world-frame submap from their stored point clouds, then aligns the live LiDAR scan against that submap via GICP. The initial guess is constructed from GPS position (translation), IMU roll/pitch (transformed to body frame), and the nearest prior keyframe's yaw.

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `gps_topic` | string | `"gps/fix"` | Input NavSatFix topic. |
| `lidar_topic` | string | `"velodyne_points"` | Input LiDAR PointCloud2 topic. |
| `imu_topic` | string | `"/imu/data"` | Input IMU topic (for roll/pitch). |
| `lidar_frame` | string | `"velodyne"` | TF frame of the LiDAR sensor. |
| `imu_frame` | string | `"imu_link"` | TF frame of the IMU sensor. |
| `gps_candidate_radius` | double | `30.0` | KD-tree search radius (meters) for finding prior map keyframes near the GPS position. |
| `fitness_threshold` | double | `0.3` | GICP fitness score threshold (logged but inlier ratio is the primary acceptance criterion). |
| `max_icp_iterations` | int | `100` | GICP max iterations. |
| `scan_ds_resolution` | double | `0.5` | Voxel downsample resolution for the live LiDAR scan (meters). |
| `submap_leaf_size` | double | `0.4` | Voxel downsample leaf size for the assembled prior map submap (meters). |
| `max_correspondence_distance` | double | `2.0` | GICP max correspondence distance (meters). |
| `num_threads` | int | `4` | Thread count for GICP and preprocessing. |
| `num_neighbors` | int | `10` | Number of neighbors for normal/covariance estimation. |
| `pointcloud_from` | string | `"liso_factor"` | MapManager data key prefix for retrieving prior map keyframe point clouds. |
| `gps_from` | string | `"gps_factor"` | MapManager data key prefix for retrieving the saved `utm_to_map` offset from the prior map. |

## Notes

- Requires a prior map with both saved point clouds and a `gps_factor/utm_to_map` global entry.
- LiDAR scans are transformed to body frame using the `base_link` to `lidar_frame` TF (resolved once on first scan).
- IMU orientation is transformed to body frame using the `base_link` to `imu_frame` TF (resolved once on first message).
- The inlier ratio must exceed 0.3 for the relocalization to be accepted.
- On successful relocalization, the `utm_to_map` offset is refined using the precise GICP pose and re-persisted to MapManager.

## Mapping vs Localization

This plugin is loaded in both modes. In mapping mode (when no prior map is loaded), `tryRelocalize()` returns immediately because `hasPriorMap()` is false — the system skips relocalization and starts from the origin. In localization mode, it actively relocalizes against the loaded prior map.

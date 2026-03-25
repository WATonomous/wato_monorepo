# EuclideanDistanceLoopClosureFactor

**Class:** `eidos::EuclideanDistanceLoopClosureFactor`
**Type:** Factor (latching via `latchFactor`)
**XML:** `factor_plugins.xml`

Detects loop closures by searching for spatially close but temporally distant keyframes using a KD-tree radius search, then verifying with GICP scan alignment. GICP runs on a background thread; the resulting `BetweenFactor<Pose3>` is delivered on the next `latchFactor()` call. Submaps are assembled in the body frame of the center keyframe, and the ISAM2-estimated relative pose is used as the GICP initial guess.

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `search_radius` | double | `15.0` | KD-tree radius for finding loop closure candidates (meters). |
| `search_time_diff` | double | `30.0` | Minimum time difference (seconds) between source and candidate keyframes. |
| `search_num` | int | `25` | Maximum number of keyframe states to collect for the candidate submap. |
| `min_inlier_ratio` | double | `0.3` | Minimum GICP inlier ratio to accept a loop closure. |
| `submap_leaf_size` | double | `0.4` | Voxel downsample leaf size for submap preprocessing (meters). |
| `submap_radius` | double | `25.0` | Spatial radius for collecting neighbor keyframes into the candidate submap (meters). |
| `max_correspondence_distance` | double | `2.0` | GICP max correspondence distance (meters). |
| `max_iterations` | int | `100` | GICP max iterations. |
| `num_threads` | int | `4` | Thread count for GICP and preprocessing. |
| `num_neighbors` | int | `10` | Number of neighbors for normal/covariance estimation. |
| `loop_closure_cov` | double[6] | `[0.01, 0.01, 0.01, 0.01, 0.01, 0.01]` | Diagonal noise variances [tx, ty, tz, rx, ry, rz] for the BetweenFactor. |
| `pointcloud_from` | string | `""` | MapManager data key for PCL point clouds (fallback source). |
| `gicp_pointcloud_from` | string | `""` | MapManager data key for small_gicp point clouds (preferred source). |
| `frequency` | double | `1.0` | Legacy parameter, no longer used. Declared to avoid YAML errors. |

## Notes

- Only runs while in TRACKING state.
- Only one GICP alignment runs at a time (background thread). If GICP is in progress, new candidates are not dispatched.
- Source cloud is the single body-frame cloud of the latest keyframe (not a submap) to avoid confirming the current ISAM2 estimate instead of detecting drift.
- Prior map keys are skipped as candidates since they are not in ISAM2.

## Mapping vs Localization

This plugin is **only loaded in mapping mode**. It is not listed in the localization config's `factor_plugins` list. In localization mode, no new factors are added to the graph, so loop closure detection is not needed.

# lidar_aggregator

Synchronizes, motion-compensates, and merges the three vehicle LiDAR point clouds (center, NE, NW) into a single output cloud.

## Overview

The vehicle carries three Velodyne LiDARs that spin asynchronously and have small timing offsets relative to each other. Before merging them, two problems must be solved:

1. **Synchronization** — the three scan streams must be paired so they correspond to the same moment in time.
2. **Deskewing (motion compensation)** — each scan takes ~100 ms to complete. If the vehicle is turning, points at the start and end of a scan are captured at different vehicle poses. Without correction, the merged cloud would be smeared.

`lidar_aggregator` handles both using IMU orientation data. An optional online timing offset estimator can refine the initial timing offsets using voxel overlap scoring.

## Architecture

```
/lidar_cc/velodyne_points ─┐
/lidar_ne/velodyne_points ─┤──► ApproximateTime sync ──► deskew each cloud ──► merge ──► /lidar/merged
/lidar_nw/velodyne_points ─┘         ▲
                                      │
/novatel/oem7/imu/data_raw ──────► IMU buffer (SLERP interpolation)
/novatel/oem7/bestpos ──────────► clock offset estimation (GPS↔IMU)
```

**Deskew** works per-point using the `time` field in each `PointCloud2` message (offset from scan start). Each point is rotated back to the scan-start pose using interpolated IMU orientation deltas. If no per-point time field is present, scan-level deskew is applied as a fallback.

**Online timing offset estimation** is optional. When enabled, the node searches over candidate timing offsets for NE/NW clouds, scores each candidate by voxel overlap with the center cloud, and updates the active offsets using exponential moving average. Only activates when yaw rate exceeds a minimum threshold (rotation provides signal for overlap scoring).

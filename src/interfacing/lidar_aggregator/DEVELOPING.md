# Developing lidar_aggregator

## Topics

### Subscribed

| Topic | Type | Default | Description |
|-------|------|---------|-------------|
| `topics.gps` | `novatel_oem7_msgs/BESTPOS` | `/novatel/oem7/bestpos` | GPS for clock offset estimation |
| `topics.imu` | `sensor_msgs/Imu` | `/novatel/oem7/imu/data_raw` | IMU for motion compensation |
| `topics.center` | `sensor_msgs/PointCloud2` | `/lidar_cc/velodyne_points` | Center LiDAR |
| `topics.ne` | `sensor_msgs/PointCloud2` | `/lidar_ne/velodyne_points` | NE side LiDAR |
| `topics.nw` | `sensor_msgs/PointCloud2` | `/lidar_nw/velodyne_points` | NW side LiDAR |

### Published

| Topic | Type | Default | Description |
|-------|------|---------|-------------|
| `outputs.ne_deskewed_cc` | `sensor_msgs/PointCloud2` | — | NE cloud motion-compensated into center frame |
| `outputs.nw_deskewed_cc` | `sensor_msgs/PointCloud2` | — | NW cloud motion-compensated into center frame |
| `outputs.merged` | `sensor_msgs/PointCloud2` | — | Merged deskewed cloud |
| `outputs.estimated_offsets` | `geometry_msgs/Vector3Stamped` | — | Current NE/NW timing offsets (diagnostics) |
| `outputs.offset_scores` | `geometry_msgs/Vector3Stamped` | — | Voxel-overlap scores for current offsets (diagnostics) |

## Parameters

### topics / outputs / frames

| Parameter | Type | Description |
|-----------|------|-------------|
| `topics.gps/imu/center/ne/nw` | string | Input topic overrides |
| `outputs.*` | string | Output topic overrides |
| `frames.center/ne/nw` | string | TF frame IDs for each LiDAR (default: `lidar_cc`, `lidar_ne`, `lidar_nw`) |

### runtime

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `runtime.qos_depth` | int | `20` | Publisher QoS history depth |
| `runtime.max_pair_dt_sec` | double | `0.05` | Max time gap between synced scan pairs |
| `runtime.max_imu_buffer_sec` | double | `20.0` | IMU buffer duration |
| `runtime.max_imu_interp_gap_sec` | double | `0.5` | Max gap for IMU interpolation |
| `runtime.scan_period_sec` | double | `0.1` | Nominal scan period (used when per-point time field is absent) |

### timing

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `timing.ne_time_offset_sec` | double | `0.0` | Manual timing offset for NE cloud (seconds) |
| `timing.nw_time_offset_sec` | double | `0.0` | Manual timing offset for NW cloud (seconds) |

### estimation (online offset tuning)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `estimation.enable_online_offset` | bool | `false` | Enable runtime offset estimation |
| `estimation.write_back_to_params` | bool | `true` | Write optimized offsets back to ROS params |
| `estimation.search_half_window_sec` | double | `0.03` | Search range ±N seconds around current offset |
| `estimation.search_step_sec` | double | `0.003` | Step size for candidate offset search |
| `estimation.ema_alpha` | double | `0.2` | EMA smoothing factor for offset updates (0 = no update, 1 = no smoothing) |
| `estimation.min_yaw_rate_rad_s` | double | `0.05` | Min yaw rate before estimation is active |
| `estimation.min_score_gain` | double | `0.01` | Min voxel overlap improvement to accept new offset |
| `estimation.voxel_size_m` | double | `0.5` | Voxel size for overlap scoring |
| `estimation.min_points` | int | `300` | Min points required to score a candidate |
| `estimation.min_offset_sec` | double | `-0.1` | Lower bound on estimated offset |
| `estimation.max_offset_sec` | double | `0.1` | Upper bound on estimated offset |

## Build & Launch

```bash
colcon build --packages-select lidar_aggregator
# Launched by interfacing_bringup/interfacing_sensors.launch.yaml
```

## After Launching

1. **Verify startup** — check the node log for these lines before proceeding:

   ```
   LidarAggregatorNode running with message_filters sync. Inputs: cc=... ne=... nw=...
   Loaded extrinsics from TF: lidar_ne->lidar_cc and lidar_nw->lidar_cc
   GPS clock offset computed: X.XXX s
   ```

   If extrinsics fail to load, `eve_description` TF is not running — launch it first.

2. **Verify output** — confirm the merged cloud is publishing:

   ```bash
   ros2 topic hz /lidar/all/points_merged   # should be ~10 Hz
   ros2 topic echo /lidar/all/points_merged --once | grep width  # expect ~3x the points of one lidar
   ```

3. **Visualize in Foxglove** — open a 3D panel, click the panel settings, and add `/lidar/all/points_merged` as a topic. The cloud should cover a full 360° ring around the vehicle with no visible gaps between the three LiDAR sectors.

## Definition of Good Result

**Basic operation (no online offset estimation):**
- Merged cloud publishes at ~10 Hz
- Point count ≥ 2× that of the center lidar alone (all three contributing)
- No repeated/doubled surfaces visible in Foxglove at the NE/NW seams — if surfaces appear doubled, the extrinsic calibration in `eve_description` needs updating

**With online offset estimation enabled:**

Monitor the offset score topics during a turn:

```bash
ros2 topic echo /lidar/sync/offset_scores
```

`vector.x` = NE overlap score, `vector.y` = NW overlap score. Score is `hits / total_points` (voxel overlap fraction, range 0–1).

- **Good:** scores stabilize at ≥ 0.10 (≥ 10% overlap) during turns — NE/NW timing is well-aligned with the center cloud
- **Poor:** scores below 0.05 consistently — increase `search_half_window_sec` or adjust `timing.ne_time_offset_sec` / `timing.nw_time_offset_sec`

Log lines during active estimation look like:

```
Offset update [ne]: old=0.0000 best=0.0120 new=0.0024 score=0.143 gyro_z=0.082
```

## Internal Architecture

**Message synchronization:** Uses `message_filters::Synchronizer` with `ApproximateTime` policy across the three lidar streams. When a synchronized triplet arrives, the callback deskews and merges.

**IMU buffering:** A deque stores `ImuSample` structs (timestamp, orientation quaternion, gyro_z) with mutex protection. `get_orientation_at_time()` does SLERP interpolation between bracketing samples. `get_rotation_delta()` returns the rotation between two timestamps for deskew.

**Deskew pipeline:**
1. Look up TF extrinsics (NE/NW → center) at configure time.
2. For each side cloud, compute per-point rotation delta using the point's `time` offset field and interpolated IMU.
3. Rotate each point by its delta to bring it to the scan-start pose.
4. Transform into the center frame using TF extrinsics.
5. Concatenate with the deskewed center cloud.

**Online offset estimation:** Iterates candidate offsets in `[current ± search_half_window]` steps. For each candidate, re-synchronizes the side cloud header timestamp and scores voxel overlap against the center cloud. The best candidate updates the active offset via EMA if the improvement exceeds `min_score_gain`. Only runs when yaw rate is high enough that spatial overlap is informative.

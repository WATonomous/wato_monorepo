# MultiViewOdometryFactor

**Class:** `eidos::MultiViewOdometryFactor`
**Type:** Factor (state-creating via `produceFactor`)
**XML:** `factor_plugins.xml`

Keyframes and odometry from camera images alone. The motion between keyframes is measured by a multi-view model (MapAnything, served by `src/perception/map_anything`) from the images of all configured cameras, which also returns each keyframe's dense coloured point cloud. It needs no lidar: with `ImuFactor` and `GpsFactor` it makes a camera + IMU + GPS graph.

This is an **experiment**: MapAnything's odometry is about 5–7 % per step, against 1.5 % for LISO. See `src/perception/map_anything/README.md`.

## How It Works

1. **Choose.** A worker thread picks keyframes at camera frame times:
   - the first one as soon as tracking begins;
   - then the first time the GPS (`gps_topic`) shows `keyframe_distance` of travel since the previous keyframe, or `keyframe_max_interval` after it, whichever comes first.

   The GPS only times the keyframes; it does not enter this plugin's factors. A keyframe is skipped if a camera has no image within `frame_tolerance` of it; the next frame is tried.
2. **Measure.** The pose service returns the rig's motion from the previous keyframe's images to the new one's, and with `cloud.enabled` the new keyframe's points in the rig frame (one per `cloud.voxel`, up to `cloud.max_range` from their camera). One request takes about 0.7 s for two sets of 8 images on an RTX 4090, 3.5 s with the points.
3. **Check.** A measurement is rejected when fewer than `min_cameras` cameras contributed, or when they disagree by more than `max_trans_spread` / `max_rot_spread`.
4. **Create.** `produceFactor()` creates the state at the images' time with:
   - a `BetweenFactor<Pose3>` from the previous keyframe, `Diagonal::Sigmas([rot_sigma ×3, σt ×3])`, `σt = trans_sigma + trans_sigma_per_m · distance`, in a robust kernel;
   - the previous keyframe's optimized pose composed with the measured motion as the initial value.

   The points are stored under `<name>/cloud` (format `pcl_xyzrgb_binary`); point `KeyframeMapVisualization.pointcloud_from` at it to see a map that grows and follows the graph's corrections.

**Lidar depth (optional, `depth.enabled`).** MapAnything also accepts metric depth per image. With it, the plugin keeps the recent scans of a spinning lidar (`depth.lidar_topic`) and the vehicle's body twist from an odometry topic of eidos' own (`depth.odometry_topic`, e.g. `ImuFactor`'s). For each image, the points fired within `depth.window` of the image's time are moved to that time and into the camera's frame, and sent with the images; the server turns them into sparse depth maps.
- The image's time is its stamp + `time_offset`.
- Each row of an organized scan is one firing, in time order over `depth.sweep_period`, with the scan stamped at the end of the revolution (velodyne driver).
- Motion between firing and image is a constant twist over at most a few tens of ms.
- Extrinsics come from TF: `base_link` <- lidar and camera <- `base_link`. Projected depth is only as good as both; see `tools/camera_lidar_calibrate.py`.

On the July 7 2026 drive, lidar depth halved MapAnything's per-step translation error (6.5 % to 3.7 %) but not its rotation error; see the map_anything README.

**Bias states (optional, `bias.enabled`).** MapAnything's errors are not zero-mean. On the July 7 2026 drive with lidar depth:
- distances were 3.7 % short, drifting from 2 % to 5 % over the drive;
- yaw leaned left by 0.027° per metre travelled (none while stopped);
- turns were reported about 4 % too small.

A plain BetweenFactor treats such errors as noise, and the graph bends its other states to fit them; with the IMU, the yaw lean ended up in the gyro bias. With `bias.enabled`, every state i also gets a bias vector `c<i>` (translation scale, yaw per metre, rotation scale), and the measurement becomes a `BiasedRelativePoseFactor` (`include/eidos/utils/biased_relative_pose_factor.hpp`):

`measured = Pose3(Rz(yaw_per_m * |t|) * Exp((1 + rotation_scale) * Log(R)), (1 + scale) * t)` for the true relative pose (R, t).

The first bias has a zero-mean prior (`bias.prior_sigma`); each later one follows a random walk from the previous (`bias.random_walk` per √s). The biases are only observable with other sensors: the gyro sees rotation errors, and the IMU and GPS see the scale. The estimate is published on `<name>/bias`.

A measurement that failed or was rejected adds no factor:
- with `create_without_measurement: true`, the state is still created, and other plugins (e.g. `ImuFactor`) must constrain it;
- otherwise the keyframe is dropped and the next one is measured from the previous keyframe.

With `add_factors: false` (shadow mode) every keyframe becomes a state without a visual factor, so other plugins can be scored on the same keyframes, and the measurements are still published.

States are created about one pose-service call after their time. `GpsFactor` and `ImuFactor` match states by their timestamp, so this delay does not change what they attach.

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `cameras` | string[] | `[]` | Camera names. A keyframe's time is its first camera's image stamp. |
| `image_topic` | string | `"/{camera}/image_rect_compressed"` | `sensor_msgs/CompressedImage` per camera; `header.frame_id` must be the camera's optical frame. |
| `frame_tolerance` | double | `0.06` | Largest time between a keyframe and each camera's image (s). |
| `buffer_duration` | double | `2.0` | Seconds of images kept per camera. Must cover the time one measurement takes. |
| `pose_service` | string | `"/perception/map_anything/estimate_relative_pose"` | `eidos_msgs/srv/EstimateRelativePose`. |
| `service_timeout` | double | `10.0` | Seconds to wait for a response. |
| `time_offset` | double | `0.0` | An image was taken at its stamp + this (s). States are created at that time, and lidar depth is moved to it. |
| `gps_topic` | string | `"/novatel/oem7/fix"` | `sensor_msgs/NavSatFix` timing the keyframes. Empty: by time only. |
| `keyframe_distance` | double | `5.0` | GPS travel between keyframes (m). |
| `keyframe_max_interval` | double | `1.0` | Longest time between keyframes (s), e.g. when stopped. `0`: none (needs `gps_topic`). |
| `add_factors` | bool | `true` | `false` = shadow mode: create the states, measure and publish, add no factors. |
| `create_without_measurement` | bool | `false` | Create the state when its measurement failed or was rejected. Only safe if another plugin constrains every state. |
| `cloud.enabled` | bool | `true` | Ask for and store each keyframe's dense points. |
| `cloud.voxel` | double | `0.2` | One point per voxel of this size (m). |
| `cloud.max_range` | double | `40.0` | Drop points farther than this from their camera (m). |
| `cloud.every_n` | int | `1` | Ask for the points on every n-th keyframe only. A request with points takes about 3× as long (3.5 s against 0.7 s for 2 × 8 images). |
| `depth.enabled` | bool | `false` | Send lidar points as metric depth with the images. |
| `depth.lidar_topic` | string | `"/lidar_cc/velodyne_points"` | Organized `sensor_msgs/PointCloud2` of a spinning lidar. |
| `depth.odometry_topic` | string | `"imu_factor/odometry"` | `nav_msgs/Odometry` whose twist (body frame) moves points to the image's time. |
| `depth.sweep_period` | double | `0.05` | Seconds per revolution; rows of a scan are its firings in time order, stamped at the end. `0`: all points at the stamp. |
| `depth.window` | double | `0.03` | Use points fired this close to the image's time (s); with a 50 ms revolution each direction is fired once. |
| `depth.min_range` | double | `1.0` | Closest point used (m). |
| `depth.max_range` | double | `60.0` | Farthest point used (m). |
| `depth.buffer_duration` | double | `2.0` | Seconds of scans and twist kept. |
| `min_cameras` | int | `2` | Reject a measurement averaged over fewer cameras. |
| `max_trans_spread` | double | `1.0` | Reject if the cameras' estimates disagree by more (m, RMS). |
| `max_rot_spread` | double | `0.05` | Same for rotation (rad). |
| `rot_sigma` | double | `0.008` | Rotation sigma (rad). |
| `trans_sigma` | double | `0.05` | Translation sigma floor (m). |
| `trans_sigma_per_m` | double | `0.07` | Added translation sigma per metre measured. |
| `robust_kernel` | string | `"huber"` | `none`, `huber` or `cauchy`. |
| `bias.enabled` | bool | `false` | Estimate the measurement biases as graph states (needs other plugins that observe them, e.g. `ImuFactor`, `GpsFactor`). |
| `bias.prior_sigma` | double[3] | `[0.1, 0.002, 0.1]` | First state's bias prior: translation scale, yaw per metre (rad/m), rotation scale. |
| `bias.random_walk` | double[3] | `[0.002, 2e-5, 0.002]` | Bias change per √s between states, same order. |
| `robust_k` | double | `1.345` | Robust kernel parameter. |
| `max_queue` | int | `30` | Measured keyframes waiting for the SLAM loop before the worker pauses. |

Reads the global `frames.base_link`; the pose service is asked for the motion of that frame, so TF must connect it to the camera frames.

## Topics and Services

| Name | Type | Description |
|---|---|---|
| `<image_topic>` (sub) | `sensor_msgs/CompressedImage` | One per camera. |
| `<gps_topic>` (sub) | `sensor_msgs/NavSatFix` | Keyframe timing. |
| `<depth.lidar_topic>`, `<depth.odometry_topic>` (sub) | `sensor_msgs/PointCloud2`, `nav_msgs/Odometry` | Lidar depth, if enabled. |
| `<name>/measurement` (pub) | `eidos_msgs/LoopClosureMeasurement` | Every measurement (`candidate_source: consecutive`), with the graph's relative pose of the same two states once optimized. |
| `<name>/bias` (pub) | `geometry_msgs/Vector3Stamped` | With `bias.enabled`: the newest state's bias estimate (x = translation scale, y = yaw per metre in rad/m, z = rotation scale). |
| `<pose_service>` (client) | `eidos_msgs/srv/EstimateRelativePose` | Rig motion between two image sets, and the second set's points. |

## Map Data

| Data key | Format | Content |
|---|---|---|
| `<name>/cloud` | `pcl_xyzrgb_binary` | The keyframe's dense points in `base_link` (x, y, z, rgb). |

## Example

```yaml
factor_plugins: ["multiview_odometry_factor", "imu_factor", "gps_factor"]

multiview_odometry_factor:
  plugin: "eidos::MultiViewOdometryFactor"
  cameras: ["camera_pano_nn", "camera_pano_ne", "camera_pano_ee", "camera_pano_se",
            "camera_pano_ss", "camera_pano_sw", "camera_pano_ww", "camera_pano_nw"]
  buffer_duration: 4.0
  create_without_measurement: true   # ImuFactor constrains every state

keyframe_map_visualization:
  pointcloud_from: "multiview_odometry_factor/cloud"
```

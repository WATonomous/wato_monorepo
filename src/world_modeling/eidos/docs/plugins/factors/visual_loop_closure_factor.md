# VisualLoopClosureFactor

**Class:** `eidos::VisualLoopClosureFactor`
**Type:** Factor (latching via `latchFactor`)
**XML:** `factor_plugins.xml`

Loop closure from camera images. The relative pose between two keyframes is measured by a multi-view model (MapAnything, served by `src/perception/map_anything`) from the camera images nearest to each keyframe, and added as a `BetweenFactor<Pose3>`.

eidos itself stays free of GPU and image libraries: the plugin passes the images on still compressed and gets a pose back over a ROS service. This is an **experiment**; see the accuracy it reached in `src/perception/map_anything/README.md` before adding its factors to a graph.

## How It Works

1. **Record.** For every new keyframe, a thread of the plugin waits until every camera has delivered an image newer than the keyframe, then stores each camera's image nearest to the keyframe under `<name>/frames` in `MapManager`.
   - The images are saved with the map, so `VisualRelocalization` can later relocalize against it.
   - A keyframe with no image within `frame_tolerance` of it for some camera is skipped.
2. **Describe.** With `candidate_mode` `place_recognition` or `both`, the keyframe's images get place descriptors from the descriptor service, stored under `<name>/descriptors`.
3. **Find candidates.** At most one of each kind per keyframe, both at least `search_time_diff` seconds older:
   - `distance`: the nearest keyframe within `search_radius` whose heading differs by at most `max_heading_diff_deg`. This is the search `EuclideanDistanceLoopClosureFactor` does, so it only finds loops the drifted poses still put close together.
   - `place_recognition`: the keyframe whose images look most alike, if the similarity reaches `min_similarity`. This also finds loops after large drift.
4. **Measure.** The pose service returns the rig's motion between the two image sets.
   - Camera images are not taken at the keyframe's (lidar) time. The motion in between is taken as constant-velocity along the neighbouring keyframes and removed at both ends.
5. **Check.** A measurement is rejected when fewer than `min_cameras` cameras contributed, when the cameras disagree by more than `max_trans_spread` / `max_rot_spread`, or (distance candidates only, unless `gate_place_recognition`) when it is further than `gate_trans` / `gate_rot` from the graph's current relative pose.
6. **Factor.** An accepted measurement becomes a `BetweenFactor<Pose3>` on the next `latchFactor()` call, with `Diagonal::Sigmas([rot_sigma ×3, σt ×3])`, `σt = trans_sigma + trans_sigma_per_m · distance`, wrapped in a robust kernel.

Every measurement is published on `<name>/measurement`, accepted or not. With `add_factors: false` (shadow mode) nothing is added to the graph.

With `shadow_odometry: true` the plugin also measures every consecutive keyframe pair. Those are only published (`candidate_source: consecutive`), never added: they are a baseline to score against the odometry already in the graph.

The pose service takes roughly 0.3 s for two sets of three images on an RTX 4090. If keyframes arrive faster than they can be measured, the oldest waiting ones are skipped (`max_queue`); their images are still recorded.

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `cameras` | string[] | `[]` | Camera names. The frame set's stamp is the first camera's. |
| `image_topic` | string | `"/{camera}/image_rect_compressed"` | `sensor_msgs/CompressedImage` topic per camera. `header.frame_id` must be the camera's optical frame. |
| `frame_tolerance` | double | `0.06` | Largest accepted time between a keyframe and each camera's image (s). |
| `buffer_duration` | double | `2.0` | Seconds of images kept per camera while waiting for keyframes. |
| `max_frame_wait` | double | `0.5` | A camera silent for this long (s) after a keyframe no longer delays recording it. |
| `pose_service` | string | `"/perception/map_anything/estimate_relative_pose"` | `eidos_msgs/srv/EstimateRelativePose`. |
| `descriptor_service` | string | `"/perception/map_anything/compute_place_descriptors"` | `eidos_msgs/srv/ComputePlaceDescriptors`. |
| `service_timeout` | double | `10.0` | Seconds to wait for a service response. |
| `add_factors` | bool | `true` | `false` = shadow mode: measure and publish, but add no factors. |
| `shadow_odometry` | bool | `false` | Also measure every consecutive keyframe pair, for publishing only. |
| `candidate_mode` | string | `"distance"` | `distance`, `place_recognition` or `both`. |
| `search_radius` | double | `20.0` | Distance candidates: radius (m). |
| `search_time_diff` | double | `40.0` | Minimum time between the two keyframes (s), both candidate kinds. |
| `max_heading_diff_deg` | double | `60.0` | Distance candidates: largest heading difference. With the server's `per_camera` fit the cameras must see the same scene at both keyframes. |
| `min_similarity` | double | `0.6` | Place recognition: cosine similarity needed. |
| `max_queue` | int | `30` | Keyframes waiting for the pose service before the oldest is skipped. |
| `min_cameras` | int | `1` | Reject a measurement averaged over fewer cameras. |
| `max_trans_spread` | double | `1.0` | Reject if the cameras' estimates disagree by more (m, RMS). |
| `max_rot_spread` | double | `0.05` | Same for rotation (rad). |
| `gate_trans` | double | `5.0` | Reject if further than this from the graph's relative pose (m). `0` disables. |
| `gate_rot` | double | `0.3` | Same for rotation (rad). `0` disables. |
| `gate_place_recognition` | bool | `false` | Also apply the graph gate to place-recognition candidates. Off by default: those candidates exist to find loops the drifted graph no longer agrees with. |
| `rot_sigma` | double | `0.02` | Rotation sigma (rad). |
| `trans_sigma` | double | `0.3` | Translation sigma floor (m). |
| `trans_sigma_per_m` | double | `0.08` | Added translation sigma per metre between the keyframes. |
| `robust_kernel` | string | `"huber"` | `none`, `huber` or `cauchy`. |
| `robust_k` | double | `1.345` | Robust kernel parameter. |

Reads the global `frames.base_link`; the pose service is asked for the motion of that frame, so TF must connect it to the camera frames.

## Topics and Services

| Name | Type | Description |
|---|---|---|
| `<image_topic>` (sub) | `sensor_msgs/CompressedImage` | One per camera. |
| `<name>/measurement` (pub) | `eidos_msgs/LoopClosureMeasurement` | Every measurement, with the graph's relative pose at that moment and the quality values. |
| `<pose_service>` (client) | `eidos_msgs/srv/EstimateRelativePose` | Rig motion between two image sets. |
| `<descriptor_service>` (client) | `eidos_msgs/srv/ComputePlaceDescriptors` | Place descriptors of an image set. |

## Map Data

| Data key | Format | Content |
|---|---|---|
| `<name>/frames` | `visual_frame_set` | The compressed images of the keyframe (`eidos::VisualFrameSet`). About 0.5 MB per camera. |
| `<name>/descriptors` | `raw_float_vector` | One place descriptor per camera, concatenated. |

## Limitations

- **Memory and map size.** Images are kept as received. Three cameras cost about 1.5 MB per keyframe in memory and in the saved map.
- **Unsynchronized cameras.** Every camera contributes the image nearest to the keyframe; the time correction uses the first camera's stamp.
- **Opposite-direction revisits** need cameras that see the same scene from both directions and the server's `joint` fit; with forward cameras only they cannot be measured, and `max_heading_diff_deg` keeps them from being tried.
- **Prior-map keyframes** are never candidates: they are not in the graph.

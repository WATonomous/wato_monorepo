# VisualRelocalization

**Class:** `eidos::VisualRelocalization`
**Type:** Relocalization
**XML:** `relocalization_plugins.xml`

Relocalization from camera images, without GPS. It needs a prior map whose keyframes carry camera images and place descriptors, which is what a map saved with `VisualLoopClosureFactor` loaded contains, and the model services of `src/perception/map_anything`.

This is an **experiment**; see the accuracy it reached in `src/perception/map_anything/README.md`.

## How It Works

Attempts run on a thread of the plugin. `tryRelocalize()` starts one if none is running and returns nothing until one has succeeded.

1. **Look.** The newest image set that all cameras have reached is described by the descriptor service.
2. **Retrieve.** The `top_k` prior-map keyframes whose descriptors are most alike (at least `min_similarity`) are the candidates. Fewer than two candidates ends the attempt.
3. **Locate.** For each candidate, the pose service measures the live rig's pose relative to the keyframe's images, which gives the vehicle's pose in the map. Measurements where the cameras disagree by more than `max_trans_spread` / `max_rot_spread` are dropped.
4. **Agree.** Two candidates must put the vehicle within `agree_trans` / `agree_rot` of each other; otherwise the attempt ends. Their better one (smaller camera spread) is kept.
5. **Velocity.** A newer image set, at least `min_interval` later, is measured against the same keyframe. The two poses give the vehicle's velocity. An implausible jump (`max_speed`) ends the attempt.
6. **Hand over.** The pose is carried forward at that velocity to the time eidos asks for it. An estimate older than `max_age` is discarded and a new attempt starts.

Every accepted estimate is published on `<name>/estimate`, stamped with the time of its images.

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `cameras`, `image_topic`, `frame_tolerance`, `buffer_duration`, `pose_service`, `descriptor_service`, `service_timeout` | | | As for [VisualLoopClosureFactor](../factors/visual_loop_closure_factor.md). The cameras must be the ones the prior map was recorded with. |
| `keyframes_from` | string | `"visual_loop_closure_factor"` | Plugin instance whose `frames` and `descriptors` the prior map holds. |
| `top_k` | int | `3` | Prior keyframes measured against per attempt. |
| `min_similarity` | double | `0.5` | Cosine similarity a prior keyframe needs to be a candidate. |
| `agree_trans` | double | `1.5` | Two candidates must put the vehicle this close together (m). |
| `agree_rot` | double | `0.1` | Same for rotation (rad). |
| `max_trans_spread` | double | `1.0` | Drop a measurement whose cameras disagree by more (m, RMS). |
| `max_rot_spread` | double | `0.05` | Same for rotation (rad). |
| `min_interval` | double | `0.2` | Seconds between the two image sets the velocity is taken from. |
| `max_speed` | double | `40.0` | Plausibility bound on that velocity (m/s). |
| `max_age` | double | `3.0` | Seconds an estimate may be carried forward. |

Reads the global `frames.base_link` and `frames.map`.

## Topics and Services

| Name | Type | Description |
|---|---|---|
| `<image_topic>` (sub) | `sensor_msgs/CompressedImage` | One per camera. |
| `<name>/estimate` (pub) | `geometry_msgs/PoseStamped` | The vehicle's pose in the map at the time of the images it was measured from. |
| `<pose_service>`, `<descriptor_service>` (clients) | | As for VisualLoopClosureFactor. |

The pose eidos starts tracking from, whichever plugin supplied it, is published by the node on `slam/initial_pose`.

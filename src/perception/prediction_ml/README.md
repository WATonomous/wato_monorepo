# prediction_ml

ML-based trajectory prediction for tracked agents using Motion Transformer (MTR), with a
constant-velocity straight-line fallback that is always safe to publish.

## Overview

`prediction_ml` replaces the deployed `simple_prediction` constant-velocity stub. It runs a
synchronous fallback every tick and can asynchronously replace fallback trajectories with fresh,
valid MTR results. If MTR is disabled or unavailable, the node continues publishing fallback
predictions.

## ROS Interface

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `tracks_3d` | `vision_msgs/Detection3DArray` | Tracked agents from perception |
| `odometry_topic` param | `nav_msgs/Odometry` | Ego pose (in `map_frame`) and body-frame twist; MTR runs only with a sample near the track stamp (fallback until then) |
| `lanelet_ahead` | `lanelet_msgs/LaneletAhead` | Optional lanes; fresh ones run the map engine (see Map-optional) |

`perception_bringup` launches the node under `/perception` with these remaps (as does
`launch/prediction_ml.launch.yaml`):

| Node topic | Remapped to |
|------------|-------------|
| `tracks_3d` | `/perception/detections_3D_tracked` |
| `lanelet_ahead` | `/world_modeling/lanelet/lanelet_ahead` (absent without world_modeling) |
| `world_object_seeds` | `/world_modeling/world_object_seeds` |
| `mtr_predictions` | `/world_modeling/mtr_predictions` |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `world_object_seeds` | `world_model_msgs/WorldObjectArray` | Predicted objects consumed by world model pre-enrichment |
| `mtr_predictions` | `world_model_msgs/WorldObjectArray` | Evaluation-only pinned MTR batches, independent of fallback filtering |
| `prediction_markers` | `visualization_msgs/MarkerArray` | One line per published path, in the seeds' frame (best path per object thick green, other modes thin yellow); built only while subscribed. Not remapped: `/perception/prediction_markers` |

Each `WorldObject` carries the original `Detection3D` plus one or more `Prediction` entries. When
MTR is disabled or not ready, each detection receives one straight-line prediction with
`conf=1.0`.

`mtr_predictions` is emitted only when `mtr.pinned_target_ids` is nonempty. It publishes every
accepted pinned-target inference batch exactly once, including the synthetic `__ego__` target.
Only `detection.id` and the prediction/header fields are populated; consumers must not treat this
diagnostic stream as production world-model input. Production `world_object_seeds` behavior is
unchanged. In this evaluation mode, odometry (ego pose and twist), lanelet, and tracks are joined by
exact source stamp before MTR tensor construction so replay results do not depend on cross-topic
callback order.

## Launch

On the car it runs in the perception module (`./watod -m perception up`), managed by
`perception_lifecycle_manager` after tracking. It loads `config/params.yaml`, then the
`/**/prediction_ml_node` block of `perception_bringup/config/perception_bringup.yaml`, which only
sets the car-specific keys (`mtr.mode: "tensorrt"`, Novatel odometry).

Standalone (tracking or a bag supplies `/perception/detections_3D_tracked`), configured like the car
and brought up by its own `prediction_lifecycle_manager`:

```bash
ros2 launch prediction_ml prediction_ml.launch.yaml
```

| Argument | Default | Notes |
|----------|---------|-------|
| `mtr_mode` | `tensorrt` | `disabled` = fallback only; tensorrt builds engines from `models/prediction/*.onnx` on first run |
| `odometry_topic` | `/novatel/oem7/odom` | `/interfacing/can_state_estimator/odom` for CAN odometry |
| `ego_velocity_child_frame` | `base_link` | `base_footprint` with CAN odometry |
| `autostart` | `true` | `false` leaves it unconfigured for `ros2 lifecycle set` |
| `param_file` | `config/params.yaml` | Everything else; the arguments above override it |

## Configuration

Parameters in `config/params.yaml`. The perception bringup block mirrors them, except that on the car
it sets `mtr.mode: "tensorrt"` and reads Novatel odometry (`/novatel/oem7/odom`, child `base_link`).
Without the ONNX files in `models/prediction/` the node logs an ERROR and publishes the fallback only:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `map_frame` | `"odom"` | Frame of the ego pose and MTR inputs; tracks in another frame are moved here, seeds go back to the tracks' frame |
| `odometry_topic` | `"/interfacing/can_state_estimator/odom"` | Ego `nav_msgs/Odometry` (`/novatel/oem7/odom` is the alternative) |
| `odometry_max_gap_s` | `0.2` | Tracks further than this from every odometry sample get fallback only |
| `prediction_horizon` | `3.0` | Prediction window in seconds |
| `prediction_time_step` | `0.2` | Waypoint spacing in seconds |
| `mtr.mode` | `"disabled"` | `disabled` or `tensorrt`; anything else logs an ERROR and runs fallback-only |
| `mtr.onnx_path` | `.../prediction/mtr_av2_nomap_r2.onnx` | Built into `mtr.engine_path` at configure when that is missing (tensorrt only) |
| `mtr.engine_path` | `.../prediction/build/mtr_av2_nomap_r2.plan` | TensorRT engine path; reused when it exists |
| `mtr.map_onnx_path` | `.../prediction/mtr_av2_r2.onnx` | Map-capable model, built like `mtr.onnx_path` |
| `mtr.map_engine_path` | `.../prediction/build/mtr_av2_r2.plan` | Map-capable engine; empty disables it |
| `mtr.metadata_path` | `contract_8_target_av2.yaml` | MTR model metadata path (both engines share it) |
| `mtr.cache_ttl_s` | `0.5` | Per-object MTR result TTL in seconds |
| `mtr.selected_target_agent_limit` | `8` | Maximum target agents per MTR batch |
| `mtr.target_forward_half_angle_deg` | `90.0` | Ego-heading half-angle used to prioritize forward targets |
| `mtr.pinned_target_ids` | `[]` | Evaluation only: pin MTR targets to these detection IDs in order, bypassing the selection heuristic |
| `mtr.model_time_step_s` | `0.1` | Native MTR future-sample interval used when downsampling output |
| `mtr.ego_length` | `4.0936` | Ego vehicle length in metres |
| `mtr.ego_width` | `1.3579` | Ego vehicle width in metres |
| `mtr.ego_height` | `1.4090` | Ego vehicle height in metres |
| `mtr.ego_velocity_child_frame` | `"base_footprint"` | Required odometry child frame (body-frame twist, static TF to the tracks frame) |
| `mtr.history_steps` | `11` | History steps fed to the scene builder |
| `mtr.history_rate_hz` | `10.0` | Expected history sample rate |
| `mtr.velocity_from_history` | `true` | MTR inputs use velocity measured from each track's positions instead of the tracker's `linear_velocity` |
| `mtr.planar_inputs` | `true` | Zero heights and box sizes before inference (the Argoverse 2 model) |

## CV Fallback Behavior

When `mtr.mode` is `disabled`, or the MTR path is not ready, the node publishes a
constant-velocity straight-line trajectory per detection, using the velocity measured from that
track's own positions over the history window. Speeds under 1 m/s (box jitter), tracks with less
than 0.3 s of history, and non-agents that never enter the history (barriers, cones) get a
zero-length path. Only tracks that cannot be moved into `map_frame` (no odometry or mount TF) use the tracker's
`linear_velocity` (m/s) instead, or a zero-length path when it is missing.

## Runtime Selection

MTR inference runs on a latest-frame-only background worker. A newer submission replaces queued
work; an in-flight result is still cached unless a newer frame could not be built (invalid
tensors), so MTR keeps refreshing when inference is slower than the track period. Successful
predictions are cached by detection ID
and replace only the matching fallback trajectory while their source frame is no older than
`mtr.cache_ttl_s`. Invalid tensors, backend errors, malformed outputs, and stale results leave the
synchronous fallback unchanged.

## Map-free frames

No prebuilt map, SLAM or dynamic TF is needed. The node keeps about 1 s of odometry and takes the
ego pose and twist at each track stamp from it (interpolated between samples, or the nearest sample
within `odometry_max_gap_s`). Tracks in another frame (tracking publishes `lidar_cc`) are moved into
`map_frame` by composing that pose with the static TF from the odometry `child_frame_id` to the
tracks frame. The odometry callback accepts only finite odometry whose `frame_id` is `map_frame`
and whose `child_frame_id` is `mtr.ego_velocity_child_frame`; anything else empties the buffer, so
the tracks get the fallback only. Seeds, MTR or fallback, are published back in the tracks' own
frame: world_model resolves seeds by header frame through TF, and `map_frame` may not be in the TF
tree (`can_state_estimator` publishes `odom` -> `base_footprint` as a topic only).

## Map-optional

Two engines are built and loaded at configure: the no-map one (`mtr.engine_path`) and, when
`mtr.map_engine_path` is set, a map-capable one. A frame runs the map engine when that loaded and
`lanelet_ahead` is within `mtr.cache_ttl_s` of the tracks and can be moved into `map_frame`: TF
from the lanes' frame to the odometry child frame at the track stamp (eidos for `map`, static for
BEV lanes in `lidar_cc`), then the ego odometry, as for the tracks. Every other frame runs the
no-map engine with no lanes. A map engine that fails to load only logs a warning. The node logs
the engine in use every 10 s (`MTR engine: map (fresh lanes)` or `no-map`).

Build, architecture, backend status, model validation, testing, and contribution details are in
[DEVELOPING.md](DEVELOPING.md).

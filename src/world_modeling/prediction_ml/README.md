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
| `ego_pose` | `geometry_msgs/PoseStamped` | Ego pose in `map_frame`; MTR runs only after one arrives (fallback until then) |
| `ego_odometry` | `nav_msgs/Odometry` | Optional ego body-frame twist used by the MTR path |
| `lanelet_ahead` | `lanelet_msgs/LaneletAhead` | Optional reachable lanelets used by the MTR path |

Default remaps in `launch/prediction_ml.launch.yaml`:

| Node topic | Remapped to |
|------------|-------------|
| `tracks_3d` | `/perception/detections_3D_tracked` |
| `ego_pose` | `/world_modeling/slam/pose` (eidos) |
| `ego_odometry` | `/world_modeling/transform/odometry` |
| `lanelet_ahead` | `lanelet/lanelet_ahead` |
| `world_object_seeds` | `/world_modeling/world_object_seeds` |
| `mtr_predictions` | `/world_modeling/mtr_predictions` |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `world_object_seeds` | `world_model_msgs/WorldObjectArray` | Predicted objects consumed by world model pre-enrichment |
| `mtr_predictions` | `world_model_msgs/WorldObjectArray` | Evaluation-only pinned MTR batches, independent of fallback filtering |

Each `WorldObject` carries the original `Detection3D` plus one or more `Prediction` entries. When
MTR is disabled or not ready, each detection receives one straight-line prediction with
`conf=1.0`.

`mtr_predictions` is emitted only when `mtr.pinned_target_ids` is nonempty. It publishes every
accepted pinned-target inference batch exactly once, including the synthetic `__ego__` target.
Only `detection.id` and the prediction/header fields are populated; consumers must not treat this
diagnostic stream as production world-model input. Production `world_object_seeds` behavior is
unchanged. In this evaluation mode, pose, odometry, lanelet, and tracks are joined by exact source
stamp before MTR tensor construction so replay results do not depend on cross-topic callback order.

## Launch

```bash
ros2 launch prediction_ml prediction_ml.launch.yaml

# Supply model assets when using a build that includes a working TensorRT backend.
ros2 launch prediction_ml prediction_ml.launch.yaml \
  mtr_mode:=tensorrt \
  engine_path:=/path/to/mtr.engine \
  metadata_path:=/path/to/mtr_metadata.yaml
```

The node is a lifecycle node. Configure and activate it after launch:

```bash
ros2 lifecycle set /world_modeling/prediction_ml_node configure
ros2 lifecycle set /world_modeling/prediction_ml_node activate
```

## Configuration

Parameters in `config/params.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `prediction_horizon` | `3.0` | Prediction window in seconds |
| `prediction_time_step` | `0.2` | Waypoint spacing in seconds |
| `mtr.mode` | `"disabled"` | `disabled`, `null`, or `tensorrt`; anything else logs an ERROR and runs fallback-only |
| `mtr.engine_path` | `""` | TensorRT engine path |
| `mtr.metadata_path` | `""` | MTR model metadata path |
| `mtr.cache_ttl_s` | `0.5` | Per-object MTR result TTL in seconds; an ego pose further than this from the tracks skips MTR for that frame |
| `mtr.selected_target_agent_limit` | `8` | Maximum target agents per MTR batch |
| `mtr.target_forward_half_angle_deg` | `90.0` | Ego-heading half-angle used to prioritize forward targets |
| `mtr.pinned_target_ids` | `[]` | Evaluation only: pin MTR targets to these detection IDs in order, bypassing the selection heuristic |
| `mtr.model_time_step_s` | `0.1` | Native MTR future-sample interval used when downsampling output |
| `mtr.ego_length` | `4.0936` | Ego vehicle length in metres |
| `mtr.ego_width` | `1.3579` | Ego vehicle width in metres |
| `mtr.ego_height` | `1.4090` | Ego vehicle height in metres |
| `mtr.ego_velocity_child_frame` | `"base_footprint"` | Required odometry child frame for body-frame twist |
| `mtr.history_steps` | `11` | History steps fed to the scene builder |
| `mtr.history_rate_hz` | `10.0` | Expected history sample rate |
| `mtr.velocity_from_history` | `true` | MTR inputs use velocity measured from each track's positions instead of the tracker's `linear_velocity` |

## CV Fallback Behavior

When `mtr.mode` is `disabled`, or the MTR path is not ready, the node publishes a
constant-velocity straight-line trajectory per detection, using the velocity measured from that
track's own positions over the history window. Speeds under 1 m/s (box jitter), tracks with less
than 0.3 s of history, and non-agents that never enter the history (barriers, cones) get a
zero-length path. Only tracks that cannot be moved into `map_frame` (no TF) use the tracker's
`linear_velocity` (m/s) instead, or a zero-length path when it is missing.

## Runtime Selection

MTR inference runs on a latest-frame-only background worker. A newer submission replaces queued
work; an in-flight result is still cached unless a newer frame could not be built (invalid
tensors), so MTR keeps refreshing when inference is slower than the track period. Successful
predictions are cached by detection ID
and replace only the matching fallback trajectory while their source frame is no older than
`mtr.cache_ttl_s`. Invalid tensors, backend errors, malformed outputs, and stale results leave the
synchronous fallback unchanged.

The ego odometry callback accepts only finite twist whose `child_frame_id` matches
`mtr.ego_velocity_child_frame`. Eidos publishes `/world_modeling/transform/odometry` with
body-frame twist and defaults that child frame to `base_footprint`. The scene builder rotates the
linear velocity into map coordinates once; until valid odometry arrives, it falls back to pose
finite differences.

Build, architecture, backend status, model validation, testing, and contribution details are in
[DEVELOPING.md](DEVELOPING.md).

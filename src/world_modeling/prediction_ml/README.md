# prediction_ml

`prediction_ml` converts tracked detections into `world_model_msgs/WorldObjectArray` messages. It
always computes a synchronous constant-velocity fallback and can optionally bridge semantic scene
data to `deep_ros/deep_mtr`.

## Current status

The cross-repository migration is complete: WATO-specific adaptation remains here, while the
model-specific inference path lives in `deep_ros/deep_mtr`. The Deep ROS node is still a
non-inferencing skeleton, so constant-velocity fallback is currently the only deployable output
and MTR remains disabled by default.

Official MTR requires multiple named inputs and outputs, while the pinned Deep ROS inference API
currently supports one input and one output tensor. The next inference milestone is therefore a
verified ONNX export plus backward-compatible named multi-tensor execution in Deep ROS.

## ROS interface

Subscribed inputs:

| Topic | Type |
|---|---|
| `tracks_3d` | `vision_msgs/Detection3DArray` |
| `ego_pose` | `geometry_msgs/PoseStamped` |
| `lanelet_ahead` | `lanelet_msgs/LaneletAhead` |
| configured `mtr.result_topic` | `deep_msgs/MtrPredictionArray` |

Published outputs:

| Topic | Type | Condition |
|---|---|---|
| `world_object_seeds` | `world_model_msgs/WorldObjectArray` | Always after a tracked-detection input while active |
| configured `mtr.request_topic` | `deep_msgs/MtrScene` | Only when `mtr.enabled` is true |

Accepted MTR results must match a known request ID, source timestamp, frame, and track ID. Empty,
stale, malformed, or non-finite trajectories are ignored. A fresh accepted trajectory replaces
fallback only for the corresponding object until the cache TTL expires.

## Launch

Fallback-only behavior is the default:

```bash
ros2 launch prediction_ml prediction_ml.launch.yaml
ros2 lifecycle set /world_modeling/prediction_ml_node configure
ros2 lifecycle set /world_modeling/prediction_ml_node activate
```

The bridge and `deep_mtr` skeleton are opt-in:

```bash
ros2 launch prediction_ml prediction_ml.launch.yaml enable_mtr:=true
```

No model path is configured in `prediction_ml`. The eventual inference owner supplies an ONNX
model and selects the TensorRT execution provider through the standard Deep ROS backend parameters.

## Configuration

| Parameter | Default | Description |
|---|---:|---|
| `prediction_horizon` | `3.0` | Fallback horizon in seconds |
| `prediction_time_step` | `0.2` | Fallback waypoint interval in seconds |
| `fallback.vehicle_size_threshold_m` | `3.5` | Bounding-box threshold for selecting fallback speed |
| `fallback.vehicle_speed_mps` | `5.0` | Vehicle fallback speed |
| `fallback.vru_speed_mps` | `1.4` | VRU fallback speed |
| `mtr.enabled` | `false` | Enable the neutral Deep ROS request/result bridge |
| `mtr.request_topic` | `/mtr/scenes` | Semantic scene request topic |
| `mtr.result_topic` | `/mtr/predictions` | Neutral prediction result topic |
| `mtr.cache_ttl_s` | `0.5` | Maximum result age in seconds |

## Ownership and next work

| Owner | Area | Next deliverable |
|---|---|---|
| Person A | Scene history and official-MTR input preparation | History plus agent/map tensor packers in `deep_ros/deep_mtr` |
| Person B | Model export, Deep ROS inference, and decoding | Verified ONNX contract, named multi-tensor execution, and world-frame results |
| Person C | WATO runtime, fallback, and deployment integration | WATO horizon adaptation, end-to-end validation, and safe enablement |

See [DEVELOPING.md](DEVELOPING.md) for exact file ownership, implementation gates, testing, and
cross-repository handoffs.

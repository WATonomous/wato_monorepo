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
| `ego_pose` | `geometry_msgs/PoseStamped` | Optional ego vehicle pose used by the MTR path |
| `lanelet_ahead` | `lanelet_msgs/LaneletAhead` | Optional reachable lanelets used by the MTR path |

Default remaps in `launch/prediction_ml.launch.yaml`:

| Node topic | Remapped to |
|------------|-------------|
| `tracks_3d` | `/perception/detections_3D_tracked` |
| `ego_pose` | `/localization/pose` |
| `lanelet_ahead` | `lanelet/lanelet_ahead` |
| `world_object_seeds` | `/world_modeling/world_object_seeds` |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `world_object_seeds` | `world_model_msgs/WorldObjectArray` | Predicted objects consumed by world model pre-enrichment |

Each `WorldObject` carries the original `Detection3D` plus one or more `Prediction` entries. When
MTR is disabled or not ready, each detection receives one straight-line prediction with
`conf=1.0`.

## Launch

```bash
ros2 launch prediction_ml prediction_ml.launch.yaml

# Supply model assets when using a build that includes a working TensorRT backend.
ros2 launch prediction_ml prediction_ml.launch.yaml \
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
| `fallback.vehicle_size_threshold_m` | `3.5` | Bounding-box X size above which the fallback uses vehicle speed |
| `fallback.vehicle_speed_mps` | `5.0` | Vehicle fallback speed in metres per second |
| `fallback.vru_speed_mps` | `1.4` | Pedestrian/cyclist fallback speed in metres per second |
| `mtr.mode` | `"disabled"` | `disabled`, `null`, or `tensorrt` |
| `mtr.engine_path` | `""` | TensorRT engine path |
| `mtr.metadata_path` | `""` | MTR model metadata path |
| `mtr.cache_ttl_s` | `0.5` | Per-object MTR result TTL in seconds |
| `mtr.selected_target_agent_limit` | `8` | Maximum target agents per MTR batch |
| `mtr.history_steps` | `11` | History steps fed to the scene builder |
| `mtr.history_rate_hz` | `10.0` | Expected history sample rate |

## CV Fallback Behavior

When `mtr.mode` is `disabled`, or the MTR path is not ready, the node publishes a
constant-velocity straight-line trajectory per detection. By default, detections with
`bbox.size.x > 3.5 m` use `5.0 m/s`; all other detections use `1.4 m/s`. The parameters above can
adjust this heuristic without recompiling the package.

Build, architecture, backend status, model validation, testing, and contribution details are in
[DEVELOPING.md](DEVELOPING.md).

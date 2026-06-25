# prediction_ml

ML-based trajectory prediction for tracked agents using Motion Transformer (MTR), with a
constant-velocity straight-line fallback that is always safe to publish.

## Overview

`prediction_ml` is a new ROS 2 package that replaces the deployed `simple_prediction`
constant-velocity stub with a skeleton that supports:

1. A synchronous CV fallback (ported from `simple_prediction`) that runs every tick and is
   always publishable.
2. An asynchronous MTR inference path that caches results per-object and replaces fallback
   predictions only when a fresh, valid result is available.
3. A CPU-only build path (TensorRT OFF) that compiles and runs using the null backend — the
   null engine always returns an invalid result, so the node behaves identically to the CV
   fallback.

The three collaborators (Person A: scene builder, Person B: inference backend, Person C:
runtime + node + converter) work in disjoint files. Only `mtr_types.hpp` and
`mtr_inference_engine.hpp` are shared and frozen.

## ROS Interface

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `tracks_3d` | `vision_msgs/Detection3DArray` | Tracked agents from perception |
| `ego_pose` | `geometry_msgs/PoseStamped` | Ego vehicle pose (optional; used by MTR path) |
| `lanelet_ahead` | `lanelet_msgs/LaneletAhead` | Reachable lanelets ahead (optional; used by MTR path) |

Default remaps (set in `launch/prediction_ml.launch.yaml`):

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

Each `WorldObject` carries the original `Detection3D` plus one or more `Prediction` entries
(`header` + `conf` + `poses[]`). When MTR is disabled or not ready, a single CV straight-line
prediction with `conf=1.0` is emitted per detection.

## Architecture

```
tracks_3d ─┐
ego_pose ──┼─▶ SceneBuilder ──MtrInputTensors──▶ MtrRuntime ──▶ IMtrInferenceEngine
lanelet ───┘   (Person A)        + sidecar       (Person C)        (Person B)
                                                     │  ▲ MtrOutputTensors
                                                     ▼  │
                                              OutputConverter (Person C)
                                                     │
                              selectOutput(fallback, cached_mtr, now)
                                                     │
                                                     ▼
                                          world_object_seeds
```

### File ownership

| Owner | Files |
|-------|-------|
| Person A (scene builder) | `include/prediction_ml/scene_builder.hpp`, `src/scene_builder.cpp`, `test/test_scene_builder.cpp` |
| Person B (inference backend) | `include/prediction_ml/mtr_backend.hpp`, `src/null_backend.cpp`, `src/tensorrt_backend.cpp`, `test/test_backend.cpp` |
| Person C (runtime + node + converter) | `include/prediction_ml/mtr_runtime.hpp`, `include/prediction_ml/output_converter.hpp`, `include/prediction_ml/prediction_ml_node.hpp`, `src/mtr_runtime.cpp`, `src/output_converter.cpp`, `src/prediction_ml_node.cpp`, `config/params.yaml`, `launch/prediction_ml.launch.yaml`, `CMakeLists.txt`, `test/test_runtime.cpp` |
| Frozen shared headers | `include/prediction_ml/mtr_types.hpp`, `include/prediction_ml/mtr_inference_engine.hpp` |

## Build

```bash
# CPU-only (default) — CV fallback only, null MTR backend
colcon build --packages-select prediction_ml

# With TensorRT backend (requires CUDA/TensorRT in the dev container)
colcon build --packages-select prediction_ml --cmake-args -DPREDICTION_ML_ENABLE_TENSORRT=ON
```

The `PREDICTION_ML_ENABLE_TENSORRT` flag is **OFF by default**. When OFF, `tensorrt_backend.cpp`
is excluded from the build and `null_backend.cpp` provides a stub
`createTensorRtMtrInferenceEngine` that returns the null engine. The node compiles and runs
with pure CV fallback on any CPU-only machine (CI, laptops, dev container without GPU).

## Launch

```bash
ros2 launch prediction_ml prediction_ml.launch.yaml

# With a TensorRT engine asset (deploy)
ros2 launch prediction_ml prediction_ml.launch.yaml \
  engine_path:=/path/to/mtr.engine \
  metadata_path:=/path/to/mtr_metadata.yaml
```

The node is a lifecycle node; after launch, configure and activate it:

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
| `mtr.mode` | `"disabled"` | `disabled` \| `null` \| `tensorrt` |
| `mtr.engine_path` | `""` | TensorRT `.engine` file path (injected at launch) |
| `mtr.metadata_path` | `""` | MTR model metadata YAML path (injected at launch) |
| `mtr.cache_ttl_s` | `0.5` | Per-object MTR result TTL in seconds |
| `mtr.selected_target_agent_limit` | `8` | Max target agents per MTR batch |
| `mtr.history_steps` | `11` | History steps fed to scene builder |
| `mtr.history_rate_hz` | `10.0` | Expected history sample rate |

## CV Fallback Behavior

When `mtr.mode` is `disabled` (or the MTR path is not ready), the node publishes a single
constant-velocity straight-line trajectory per detection. The speed heuristic is:

- `bbox.size.x > 3.5 m` → vehicle speed (5.0 m/s)
- otherwise → pedestrian/cyclist speed (1.4 m/s)

This matches the behavior of the deployed `simple_prediction` package.

## Testing

```bash
colcon test --packages-select prediction_ml
colcon test-result --verbose
```

Tests: `test_contract_compiles`, `test_backend` (Person B), `test_scene_builder` (Person A),
`test_runtime` (Person C).

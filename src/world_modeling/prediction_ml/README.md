# prediction_ml

ML-based trajectory prediction for tracked agents using Motion Transformer (MTR), with a
constant-velocity fallback that is always safe to publish.

This package is the completed MTR skeleton. It is meant to compile before the next round of
real MTR implementation work starts, while still running as a safe constant-velocity fallback.

## Skeleton Status

The skeleton includes:

- Package scaffold and CMake wiring.
- Shared MTR contract headers.
- Null backend and TensorRT-gated backend stub.
- Scene builder, output converter, and runtime stubs.
- Lifecycle node with constant-velocity fallback.
- Config, launch file, and per-owner tests.

The real MTR predictor is not implemented yet. The TODOs in the owner files are intentional
handoff points for the next phase. Source-level MTR I/O verification found these contract
updates and they are already reflected in `mtr_types.hpp`:

- `obj_trajs_mask` and `map_polylines_mask` use `std::vector<uint8_t>` for bool masks.
- `center_objects_type` uses `std::vector<std::string>` for MTR object-type tokens.

Official MTR forward-pass validation has now been exercised in a WSL2 GPU environment. TensorRT
engine binding sign-off still needs to happen once the deployment engine artifact exists.

Latest local package verification was run on Windows through Ubuntu 22.04 WSL with ROS Humble.

Verification commands:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to prediction_ml --event-handlers console_direct+
source install/setup.bash
colcon test --packages-select prediction_ml --event-handlers console_direct+
colcon test-result --verbose
```

Last verified result:

```text
colcon build --packages-up-to prediction_ml: 3 packages finished
colcon test: 100% tests passed, 0 tests failed out of 4
colcon test-result: 10 tests, 0 errors, 0 failures, 0 skipped
```

The build covers:

- `lanelet_msgs`
- `world_model_msgs`
- `prediction_ml`

The verified local path is to run colcon in a sourced ROS workspace:

```bash
colcon build --packages-select prediction_ml
colcon test --packages-select prediction_ml
colcon test-result --verbose
```

If you are using the normal monorepo Docker/DevContainer flow, enter the world_modeling
DevContainer first and run the same colcon commands from `/ws`. The `watod` wrapper requires a
working Docker setup and Linux shell line endings, so it was not used for the Windows WSL
verification above.

For CI-style module testing on a machine with Docker working, the monorepo wrapper command is:

```bash
./watod -m world_modeling test world_modeling prediction_ml
```

If local dependencies are not built yet, use `--packages-up-to prediction_ml` instead of
`--packages-select prediction_ml`.

## MTR Contract Sign-Off

WSL2 real-data forward validation was run with:

- Official MTR clone at `sshaoshuai/MTR@a5ba7bd`.
- NVIDIA RTX 3060 Ti through WSL2.
- CUDA Toolkit 12.8 and PyTorch `2.11.0+cu128`.
- Waymo Motion validation shard `validation.tfrecord-00000-of-00150`.
- Local compatibility fixes for the current Waymo API and v1.3.1 `driveway` map features.

Validation result:

```text
processed validation scenarios: 286
official MTR dataloader dataset_len: 286
full-shard forward batches: 286
target agents: 1268
bad batches: 0
first output pred_scores: (2, 6)
first output pred_trajs: (2, 6, 80, 7)
```

This proves the official MTR CUDA forward path accepts real preprocessed Waymo data and produces
finite outputs. It is still random-weight validation because no trained checkpoint was available.
The official Waymo metric step failed after inference due a `metrics_ops.so` ABI mismatch in the
installed Waymo wheel, so scored validation still needs a clean Waymo metrics environment and a
trained checkpoint.

The remaining TensorRT sign-off should be done where the TensorRT engine artifact is available.

1. Export or build the TensorRT engine using the intended deployment path.
2. Print every input key shape and dtype: `obj_trajs`, `obj_trajs_mask`,
   `obj_trajs_last_pos`, `track_index_to_predict`, `center_objects_type`,
   `map_polylines`, `map_polylines_mask`, and `map_polylines_center`.
3. Print output shape and dtype for `pred_scores` and `pred_trajs`.
4. Dump TensorRT binding names, dtypes, static/dynamic dimensions, and input/output order.
5. Compare those bindings against `MtrInputTensors`, `MtrOutputTensors`, and
   `MtrModelContract`.
6. If there is a mismatch, update the shared contract before Person A/B/C build on top of it.

The sign-off artifact can live outside the repo if it contains machine-specific paths, model
assets, or large output logs. The repo only needs the final contract changes.

## Runtime Smoke Check

After building and sourcing the workspace:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch prediction_ml prediction_ml.launch.yaml
ros2 lifecycle set /world_modeling/prediction_ml_node configure
ros2 lifecycle set /world_modeling/prediction_ml_node activate
```

With a `tracks_3d` source active, verify fallback output:

```bash
ros2 topic hz /world_modeling/world_object_seeds
```

The node only publishes after detections arrive. With `mtr.mode=disabled`, output should be
constant-velocity fallback predictions.

## Goal

`prediction_ml` replaces the deployed `simple_prediction` constant-velocity stub with a
skeleton that supports:

1. A synchronous constant-velocity fallback that runs every tick and is always publishable.
2. An async MTR inference path that can later cache per-object results.
3. A CPU-only build path where TensorRT is off by default and the null backend forces fallback
   behavior.

In this skeleton, the MTR path is intentionally stubbed. The important guarantee is that the
package builds, links, launches as a lifecycle node, and has tests for each owner area while
the fallback remains publishable.

## Team Roles

Each teammate should stay inside their owned files unless a shared interface change is agreed
by the group.

| Role | Area | Owned files | Tests to update |
|------|------|-------------|-----------------|
| Person A | Scene builder | `include/prediction_ml/scene_builder.hpp`, `src/scene_builder.cpp` | `test/test_scene_builder.cpp` |
| Person B | Inference backend | `include/prediction_ml/mtr_backend.hpp`, `src/null_backend.cpp`, `src/tensorrt_backend.cpp` | `test/test_backend.cpp` |
| Person C | Runtime, output conversion, node integration | `include/prediction_ml/mtr_runtime.hpp`, `include/prediction_ml/output_converter.hpp`, `include/prediction_ml/prediction_ml_node.hpp`, `src/mtr_runtime.cpp`, `src/output_converter.cpp`, `src/prediction_ml_node.cpp`, `config/params.yaml`, `launch/prediction_ml.launch.yaml`, `CMakeLists.txt` | `test/test_runtime.cpp` |
| Shared | Frozen contracts | `include/prediction_ml/mtr_types.hpp`, `include/prediction_ml/mtr_inference_engine.hpp` | `test/test_contract_compiles.cpp` |

Shared headers should be treated as frozen for follow-up owner branches. If a teammate needs
to change one, call it out before merging because it can affect all three roles.

## Role Checklists

### Person A: Scene Builder

Owns converting ROS scene context into MTR input tensors.

Start here:

- `include/prediction_ml/scene_builder.hpp`
- `src/scene_builder.cpp`
- `test/test_scene_builder.cpp`

Expected work:

- Keep track history per detection or track id.
- Pack object history tensors.
- Select target agents up to `mtr.selected_target_agent_limit`.
- Add lanelet/map context when available.
- Fill `MtrBatchSidecar` so outputs can be mapped back to source detections.

Before handing off:

```bash
colcon test --packages-select prediction_ml --ctest-args -R test_scene_builder
```

### Person B: Inference Backend

Owns the null backend, TensorRT backend, and model contract validation.

Start here:

- `include/prediction_ml/mtr_backend.hpp`
- `src/null_backend.cpp`
- `src/tensorrt_backend.cpp`
- `test/test_backend.cpp`

Expected work:

- Keep the null backend fallback-safe.
- Implement TensorRT engine loading behind `PREDICTION_ML_ENABLE_TENSORRT`.
- Parse model metadata.
- Validate expected and actual engine bindings.
- Return invalid output instead of publishing unsafe predictions when inference is not ready.

Before handing off:

```bash
colcon build --packages-select prediction_ml --cmake-args -DPREDICTION_ML_ENABLE_TENSORRT=OFF
colcon test --packages-select prediction_ml --ctest-args -R test_backend
```

TensorRT-specific checks require a CUDA/TensorRT environment:

```bash
colcon build --packages-select prediction_ml --cmake-args -DPREDICTION_ML_ENABLE_TENSORRT=ON
```

### Person C: Runtime, Converter, Node

Owns node lifecycle behavior, fallback publishing, async runtime flow, and converting MTR output
into `world_model_msgs`.

Start here:

- `include/prediction_ml/mtr_runtime.hpp`
- `include/prediction_ml/output_converter.hpp`
- `include/prediction_ml/prediction_ml_node.hpp`
- `src/mtr_runtime.cpp`
- `src/output_converter.cpp`
- `src/prediction_ml_node.cpp`
- `config/params.yaml`
- `launch/prediction_ml.launch.yaml`
- `test/test_runtime.cpp`

Expected work:

- Keep constant-velocity fallback publishable at every tick.
- Add async latest-frame inference without blocking the ROS callback.
- Cache valid MTR predictions with TTL.
- Replace fallback predictions only when cached MTR output is fresh and valid.
- Keep lifecycle transitions safe: configure, activate, deactivate, cleanup, shutdown.

Before handing off:

```bash
colcon test --packages-select prediction_ml --ctest-args -R test_runtime
```

## ROS Interface

### Subscribed Topics

| Node topic | Type | Default remap | Description |
|------------|------|---------------|-------------|
| `tracks_3d` | `vision_msgs/Detection3DArray` | `/perception/detections_3D_tracked` | Tracked agents from perception |
| `ego_pose` | `geometry_msgs/PoseStamped` | `/localization/pose` | Ego vehicle pose, used by MTR path |
| `lanelet_ahead` | `lanelet_msgs/LaneletAhead` | `lanelet/lanelet_ahead` | Reachable lanelets ahead, used by MTR path |

### Published Topics

| Node topic | Type | Default remap | Description |
|------------|------|---------------|-------------|
| `world_object_seeds` | `world_model_msgs/WorldObjectArray` | `/world_modeling/world_object_seeds` | Predicted objects consumed by world model pre-enrichment |

Each `WorldObject` carries the original `Detection3D` plus one or more `Prediction` entries.
When MTR is disabled or not ready, the node emits one constant-velocity prediction with
`conf=1.0` per detection.

## Architecture

```text
tracks_3d
ego_pose       -> SceneBuilder -> MtrInputTensors -> MtrRuntime -> IMtrInferenceEngine
lanelet_ahead                       + sidecar             |
                                                          v
                                                   MtrOutputTensors
                                                          |
                                                          v
                                                  OutputConverter
                                                          |
fallback predictions ---------------------> selectOutput(fallback, cached_mtr, now)
                                                          |
                                                          v
                                                world_object_seeds
```

## Build

CPU-only fallback build, TensorRT off:

```bash
colcon build --packages-select prediction_ml
```

Build local dependencies too:

```bash
colcon build --packages-up-to prediction_ml
```

TensorRT build:

```bash
colcon build --packages-select prediction_ml --cmake-args -DPREDICTION_ML_ENABLE_TENSORRT=ON
```

The `PREDICTION_ML_ENABLE_TENSORRT` flag is **OFF by default**. When OFF, `tensorrt_backend.cpp`
is excluded from the build and `createMtrInferenceEngine` selects the null engine for every
mode. When ON, the same factory delegates `MtrMode::TensorRt` to the TensorRT backend. The node
compiles and runs with pure CV fallback on any CPU-only machine (CI, laptops, dev container
without GPU).

## Launch

```bash
ros2 launch prediction_ml prediction_ml.launch.yaml
```

With TensorRT assets:

```bash
ros2 launch prediction_ml prediction_ml.launch.yaml \
  engine_path:=/path/to/mtr.engine \
  metadata_path:=/path/to/mtr_metadata.yaml
```

The node is a lifecycle node. After launch:

```bash
ros2 lifecycle set /world_modeling/prediction_ml_node configure
ros2 lifecycle set /world_modeling/prediction_ml_node activate
```

## Configuration

Parameters live in `config/params.yaml`.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `prediction_horizon` | `3.0` | Prediction window in seconds |
| `prediction_time_step` | `0.2` | Waypoint spacing in seconds |
| `mtr.mode` | `disabled` | `disabled`, `null`, or `tensorrt` |
| `mtr.engine_path` | empty | TensorRT engine path |
| `mtr.metadata_path` | empty | MTR metadata path |
| `mtr.cache_ttl_s` | `0.5` | Per-object MTR result TTL |
| `mtr.selected_target_agent_limit` | `8` | Max target agents per MTR batch |
| `mtr.history_steps` | `11` | History steps fed to scene builder |
| `mtr.history_rate_hz` | `10.0` | Expected history sample rate |

## Fallback Behavior

When `mtr.mode` is `disabled` or MTR output is not ready, the node publishes a straight-line
constant-velocity trajectory per detection.

Speed heuristic:

- `bbox.size.x > 3.5 m`: vehicle speed, `5.0 m/s`
- otherwise: pedestrian/cyclist speed, `1.4 m/s`

This matches the deployed `simple_prediction` fallback behavior.

## Full Test Command

Run this before merging changes to any owned area:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to prediction_ml --event-handlers console_direct+
source install/setup.bash
colcon test --packages-select prediction_ml --event-handlers console_direct+
colcon test-result --verbose
```

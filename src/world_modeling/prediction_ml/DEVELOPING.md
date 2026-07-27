# Developing prediction_ml

## Skeleton and Model Status

The package currently provides the MTR contracts, null and TensorRT-gated inference backends,
scene builder, asynchronous latest-frame runtime with a per-object TTL cache, validated output
conversion, lifecycle node, and constant-velocity fallback. The TensorRT backend is implemented;
deployment-engine and GPU validation remain outstanding.

This is the **active working branch**: MTR inference scaffolding lives inside the monorepo. The
alternative deep_ros direction — moving inference ownership into `WATonomous/deep_ros` (`deep_msgs`
scene/result interfaces + a `deep_mtr` lifecycle node) while `prediction_ml` keeps only the WATO
adapter, result selection, and CV fallback — is **preserved for reference** on the
`ryan/prediction-ml-skeleton` branch (`feat(prediction_ml): integrate deep ROS MTR bridge`). Refer
to that branch for the deep_ros migration design and ownership split.

Source-level MTR I/O verification established that:

- `obj_trajs_mask` and `map_polylines_mask` use `std::vector<uint8_t>` for boolean masks.
- `center_type_ids` uses `std::vector<int64_t>` for MTR object-type codes
  (`TYPE_VEHICLE=0`, `TYPE_PEDESTRIAN=1`, `TYPE_CYCLIST=2`).
- `track_index_to_predict` also uses an `int64_t` engine binding.
- Ego dimensions come from configuration, and body-frame odometry twist is rotated into the map
  frame by the scene builder before target-relative packing.

### Real-data MTR forward validation

The official MTR CUDA forward path was validated in WSL2 using:

- Official MTR clone at `sshaoshuai/MTR@a5ba7bd`.
- NVIDIA RTX 3060 Ti, CUDA Toolkit 12.8, and PyTorch `2.11.0+cu128`.
- Waymo Motion validation shard `validation.tfrecord-00000-of-00150`.
- Compatibility fixes for the current Waymo API and v1.3.1 `driveway` map features.

```text
processed validation scenarios: 286
official MTR dataloader dataset_len: 286
full-shard forward batches: 286
target agents: 1268
bad batches: 0
first output pred_scores: (2, 6)
first output pred_trajs: (2, 6, 80, 7)
```

This validates finite real-data outputs from the official CUDA forward path. It used random
weights because no trained checkpoint was available. Scored validation still requires a trained
checkpoint and a Waymo metrics environment without the observed `metrics_ops.so` ABI mismatch.

### TensorRT sign-off

The static ONNX graph and an FP16 TensorRT 10.13.2.6 engine have now been built
on the development RTX 3060 Ti. The real C++ backend deserialized that engine,
created an execution context, and matched all eight inputs and two outputs
against `contract_8_target.yaml`. Engines remain machine-local build products:
copy `mtr.onnx` and `engine_build.yaml` to the vehicle and rebuild natively.

The remaining sign-off is the `sm_89` vehicle build, numerical gate, latency
and peak-memory measurement, followed by rung-4 replay. Do not report
development-machine performance numbers.

## Runtime Smoke Check

After building and sourcing the workspace:

```bash
ros2 launch prediction_ml prediction_ml.launch.yaml
ros2 lifecycle set /world_modeling/prediction_ml_node configure
ros2 lifecycle set /world_modeling/prediction_ml_node activate
ros2 topic hz /world_modeling/world_object_seeds
```

The node publishes only after detections arrive. With `mtr.mode: "disabled"`, output should contain
constant-velocity fallback predictions.

Quality-harness replays set `mtr.pinned_target_ids` before configuration and record both
`/world_modeling/mtr_predictions` and `/world_modeling/world_object_seeds`. The former is a
one-shot diagnostic stream containing all accepted pinned MTR targets, including `__ego__`; the
latter retains production detection-list filtering and fallback takeover. Never connect the
diagnostic topic to a production world-model consumer. Pinned mode also exact-joins the four input
messages by source stamp; without that join, DDS callback scheduling can mix current tracks with
stale auxiliary state and invalidate the rung-3-to-rung-4 comparison.

## Architecture

```text
tracks_3d ─────────┐
ego_pose/odometry ─┼─▶ SceneBuilder ──MtrInputTensors──▶ MtrRuntime ──▶ IMtrInferenceEngine
lanelet ───────────┘                      + sidecar                         │
                                                                    ▼
                                                            MtrOutputTensors
                                                                    │
                                                                    ▼
                                                            OutputConverter
                                                                    │
                                      selectOutput(fallback, cached_mtr, now)
                                                                    │
                                                                    ▼
                                                        world_object_seeds
```

The lifecycle node always creates a synchronous constant-velocity fallback. `SceneBuilder`
prepares MTR inputs, `MtrRuntime` owns asynchronous inference and its result cache, and
`OutputConverter` converts valid model output into ROS predictions.

## Collaboration Boundaries

### Ownership by pipeline stage

- **Person A — Input / Scene → Tensor** (`scene_builder.*`, `test_scene_builder.cpp`):
  maintain MTR-only per-object history keyed by detection id (pose, dims, heading,
  velocity, type, timestamp, validity); resample to a fixed step and mask missing
  samples; select target agents up to a limit; pack target + context agents into
  `obj_trajs*` and `track_index_to_predict` / `center_type_ids`; convert lanelet
  context into `map_polylines*`; populate the sidecar. Depends only on `mtr_types.hpp`.

- **Person B — Inference backend** (`null_backend.cpp`, `tensorrt_backend.cpp`,
  `mtr_backend.hpp`, `test_backend.cpp`): implement `IMtrInferenceEngine`. Null engine
  is always built and returns an empty/invalid result so the runtime falls back.
  TensorRT engine loads the `.engine` from `engine_path`, validates bindings against
  `MtrModelContract`, runs `infer`. Compiled only when `PREDICTION_ML_ENABLE_TENSORRT`
  is set. Depends only on the two shared headers — can develop against the null engine
  with zero input from A or C.

- **Person C — Orchestration / Output** (`prediction_ml_node.*`, `mtr_runtime.*`,
  `output_converter.*`, `config`, `launch`, `CMakeLists.txt`, `package.xml`,
  `test_runtime.cpp`): the lifecycle node (subs/pubs/params/wiring); the CV fallback
  predictor (ported from `simple_prediction`); `MtrRuntime` (latest-only async worker,
  per-object TTL cache, `submitFrame`, `selectOutput`, `ready`/`lastError`);
  `OutputConverter` (validate `pred_scores`/`pred_trajs`, rotate/translate target frame
  → map frame, infer yaw from consecutive points, emit `world_model_msgs/Prediction`).
  Owns the build/config/launch integration spine.

Treat the shared contract headers as frozen on parallel owner branches. Coordinate required
contract changes before editing them.

## Build

CPU-only development and CI build:

```bash
colcon build --packages-select prediction_ml
```

Build with the TensorRT translation unit included:

```bash
colcon build --packages-select prediction_ml \
  --cmake-args -DPREDICTION_ML_ENABLE_TENSORRT=ON
```

The validated development environment uses TensorRT 10.13.2.6. Install the
matching CUDA-12 Python builder in the model venv:

```bash
python -m pip install "tensorrt-cu12==10.13.2.6"
```

The C++ build additionally needs matching TensorRT headers and
`libnvinfer.so.10`, normally supplied by NVIDIA's `tensorrt-dev` Debian
package. A source-header checkout can be used explicitly:

```bash
colcon build --packages-select prediction_ml --cmake-args \
  -DPREDICTION_ML_ENABLE_TENSORRT=ON \
  -DTENSORRT_INCLUDE_DIR="$HOME/TensorRT-10.13/include" \
  -DTENSORRT_NVINFER_LIBRARY="/path/to/libnvinfer.so.10"
```

`PREDICTION_ML_ENABLE_TENSORRT` remains off by default. The integrated backend implements engine
loading, binding validation, CUDA buffer management, and enqueue/copy logic, but still needs a
TensorRT-enabled GPU sign-off with the deployment engine. The repository deploy image runs
`colcon build` without this CMake option, so deployment must explicitly enable the option.

Runtime selection is separate from compile-time inclusion. `mtr.mode: "tensorrt"` requests the
TensorRT engine only in a build that includes it; otherwise the factory returns the null engine and
the node publishes CV fallback output.

Launch the compiled TensorRT backend with:

```bash
ros2 launch prediction_ml prediction_ml.launch.yaml \
  mtr_mode:=tensorrt \
  engine_path:=/absolute/path/to/mtr.engine \
  metadata_path:=/absolute/path/to/contract_8_target.yaml
ros2 lifecycle set /world_modeling/prediction_ml_node configure
```

## Testing

```bash
colcon test --packages-select prediction_ml
colcon test-result --verbose
```

The package test targets cover shared contracts, inference backend selection, scene building,
runtime selection and TTL behavior, output conversion, and fallback speed selection.

When a real engine is available, enable the optional binding gate:

```bash
MTR_ENGINE_PATH=/absolute/path/to/mtr.engine \
MTR_METADATA_PATH=/absolute/path/to/contract_8_target.yaml \
colcon test --packages-select prediction_ml --ctest-args -R test_backend
```

# Developing prediction_ml

## Skeleton and Model Status

The package currently provides the MTR contracts, null backend, TensorRT-gated backend stub,
scene builder/runtime/output-converter stubs, lifecycle node, and constant-velocity fallback. The
real TensorRT predictor is not implemented yet. Intentional TODOs in the owner files are handoff
points for that work.

Source-level MTR I/O verification established that:

- `obj_trajs_mask` and `map_polylines_mask` use `std::vector<uint8_t>` for boolean masks.
- `center_objects_type` uses `std::vector<std::string>` for MTR object-type tokens.

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

### Remaining TensorRT sign-off

When the deployment engine artifact is available:

1. Build or export the engine through the intended deployment path.
2. Record every input/output binding name, dtype, dimension, and order.
3. Compare the bindings with `MtrInputTensors`, `MtrOutputTensors`, and `MtrModelContract`.
4. Update the shared contract before backend/scene/runtime implementation if any binding differs.

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

## Architecture

```text
tracks_3d ─┐
ego_pose ──┼─▶ SceneBuilder ──MtrInputTensors──▶ MtrRuntime ──▶ IMtrInferenceEngine
lanelet ───┘                      + sidecar                         │
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

The package was initially divided across three parallel work areas. Keep changes within an owned
area unless a shared interface change is agreed by the group.

| Area | Owned files | Tests to update |
|------|-------------|-----------------|
| Scene builder | `include/prediction_ml/scene_builder.hpp`, `src/scene_builder.cpp` | `test/test_scene_builder.cpp` |
| Inference backend | `include/prediction_ml/mtr_backend.hpp`, `src/null_backend.cpp`, `src/tensorrt_backend.cpp` | `test/test_backend.cpp` |
| Runtime and node integration | `include/prediction_ml/mtr_runtime.hpp`, `include/prediction_ml/output_converter.hpp`, `include/prediction_ml/prediction_ml_node.hpp`, `src/mtr_runtime.cpp`, `src/output_converter.cpp`, `src/prediction_ml_node.cpp`, `config/params.yaml`, `launch/prediction_ml.launch.yaml`, `CMakeLists.txt` | `test/test_runtime.cpp`, `test/test_fallback_prediction.cpp` |
| Shared contracts | `include/prediction_ml/mtr_types.hpp`, `include/prediction_ml/mtr_inference_engine.hpp` | `test/test_contract_compiles.cpp` |

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

`PREDICTION_ML_ENABLE_TENSORRT` remains off by default because `tensorrt_backend.cpp` is currently
a placeholder that returns the null engine; it does not perform TensorRT inference yet. The
repository deploy image runs `colcon build` without this CMake option, so the current deploy build
excludes that placeholder translation unit. When the real backend and its dependencies are added,
the deploy build must explicitly enable the option or the project must revisit the default.

Runtime selection is separate from compile-time inclusion. `mtr.mode: "tensorrt"` requests the
TensorRT engine only in a build that includes it; otherwise the factory returns the null engine and
the node publishes CV fallback output.

## Testing

```bash
colcon test --packages-select prediction_ml
colcon test-result --verbose
```

The package test targets cover shared contracts, inference backend selection, scene building,
runtime selection, and fallback speed selection.

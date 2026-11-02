# Developing prediction_ml

## Status

The package provides the MTR contracts, null and TensorRT-gated inference backends, scene builder,
asynchronous latest-frame runtime with a per-object TTL cache, validated output conversion,
lifecycle node, and constant-velocity fallback.

Masks (`obj_trajs_mask`, `map_polylines_mask`) are `uint8`; `center_type_ids` and
`track_index_to_predict` are `int64` engine bindings (`TYPE_VEHICLE=0`, `TYPE_PEDESTRIAN=1`,
`TYPE_CYCLIST=2`). Ego dimensions come from configuration, and body-frame odometry twist is rotated
into the map frame by the scene builder before target-relative packing.

### TensorRT sign-off

The static ONNX graph and a TensorRT 10.13.2.6 engine were built on a development RTX 3060 Ti.
`engine_build.yaml` is strict FP32 with TensorFloat-32 disabled: it matched PyTorch on all 9 fixture
targets (below 1e-3 m minFDE delta), while TF32 and FP16 builds did not.
Engines are machine-local: copy `mtr.onnx` and `engine_build.yaml` to the vehicle and rebuild natively, then
measure latency and memory there.

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
messages by source stamp, so DDS callback order cannot mix current tracks with stale auxiliary state.

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

`PREDICTION_ML_ENABLE_TENSORRT` is off by default and the repository deploy image builds without
it, so deployment must enable it explicitly.

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

The Gate B parity test needs the Waymo-derived replay fixtures, which are not
committed (WOD terms). They skip until you build them with `eval/regenerate_fixtures.sh <shard>`.

The package test targets cover shared contracts, inference backend selection, scene building,
runtime selection and TTL behavior, output conversion, and the fallback's tracker velocity.

When a real engine is available, enable the optional binding gate:

```bash
MTR_ENGINE_PATH=/absolute/path/to/mtr.engine \
MTR_METADATA_PATH=/absolute/path/to/contract_8_target.yaml \
colcon test --packages-select prediction_ml --ctest-args -R test_backend
```

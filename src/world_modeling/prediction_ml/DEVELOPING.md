# Developing prediction_ml

## Status and constraints

The repository migration is complete, but learned inference is not. `prediction_ml` owns the WATO
bridge and always produces constant-velocity fallback output. The pinned `deep_ros/deep_mtr` node
accepts scenes but deliberately publishes no predictions.

This guide was checked against official MTR commit
`a5ba7bdafa09a1a355cc34f8a895499a2b14ddb3` and pinned Deep ROS commit
`c7e40a1c8c51e7d1cac48fb59a285d3af4915e38`.

Official MTR prepares centered agent and map inputs separately. Its encoder consumes agent
trajectories/masks/last positions, map polylines/masks/centers, target indices, and center-object
types. The reference configuration uses 29 agent attributes, nine map attributes, 20 points per
polyline, up to 768 polylines, six modes, and 80 future frames. The exported ONNX contract remains
authoritative for deployment names, dtypes, dimensions, and limits.

Working inference has two gates:

1. Person B must prove the official model's custom CUDA attention/KNN operations export and execute
   with ONNX Runtime CUDA and its TensorRT execution provider.
2. Pinned Deep ROS binds only input 0 and output 0. Person B must add backward-compatible named
   multi-input/multi-output execution.

Do not use one anonymous packed tensor, a custom MTR backend, or direct TensorRT `.engine` loading.
Use an ONNX model through `onnxruntime_gpu` with `Backend.execution_provider: "tensorrt"`.

`prediction_ml` owns WATO adaptation, correlation, fallback, validation, horizon selection, and
`world_model_msgs` output. `deep_mtr` owns history, tensor preparation, inference, and decoding.
`deep_msgs` stays independent of `lanelet_msgs` and `world_model_msgs`.

## File ownership

Paths are relative to the named repository. Each file has one primary owner.

### Person A: history and input preparation

Current `wato_monorepo` files:

- `src/world_modeling/prediction_ml/include/prediction_ml/mtr_scene_adapter.hpp`
- `src/world_modeling/prediction_ml/src/mtr_scene_adapter.cpp`
- `src/world_modeling/prediction_ml/test/test_mtr_scene_adapter.cpp`

Current `deep_ros` input contracts:

- `deep_msgs/msg/MapPolyline.msg`
- `deep_msgs/msg/MtrScene.msg`

New `deep_ros` files:

- History: `deep_mtr/include/deep_mtr/mtr_scene_history.hpp`,
  `deep_mtr/src/mtr_scene_history.cpp`, `deep_mtr/test/test_mtr_scene_history.cpp`.
- Agent packing: `deep_mtr/include/deep_mtr/mtr_agent_input_packer.hpp`,
  `deep_mtr/src/mtr_agent_input_packer.cpp`,
  `deep_mtr/test/test_mtr_agent_input_packer.cpp`.
- Map packing: `deep_mtr/include/deep_mtr/mtr_map_input_packer.hpp`,
  `deep_mtr/src/mtr_map_input_packer.cpp`, `deep_mtr/test/test_mtr_map_input_packer.cpp`.

Person A derives motion from timestamped history, maps WATO classes to numeric MTR types,
represents ego as the SDC input, selects targets deterministically, and maps available lanelet
semantics without claiming full Waymo roadgraph coverage.

### Person B: export, inference, and decoding

Current `deep_ros` output contracts:

- `deep_msgs/msg/MtrTrajectory.msg`
- `deep_msgs/msg/MtrObjectPrediction.msg`
- `deep_msgs/msg/MtrPredictionArray.msg`

Current `deep_mtr` package:

- Node: `deep_mtr/include/deep_mtr/deep_mtr_node.hpp`, `deep_mtr/src/deep_mtr_node.cpp`,
  `deep_mtr/test/test_deep_mtr_node.cpp`.
- Runtime files: `deep_mtr/config/deep_mtr.yaml`, `deep_mtr/launch/deep_mtr.launch.yaml`.
- Package/docs: `deep_mtr/CMakeLists.txt`, `deep_mtr/package.xml`, `deep_mtr/README.md`,
  `deep_mtr/DEVELOPING.md`.

New model files:

- Contract: `deep_mtr/include/deep_mtr/mtr_model_contract.hpp`,
  `deep_mtr/test/test_mtr_model_contract.cpp`.
- Decoder: `deep_mtr/include/deep_mtr/mtr_output_decoder.hpp`,
  `deep_mtr/src/mtr_output_decoder.cpp`, `deep_mtr/test/test_mtr_output_decoder.cpp`.
- Integration/export: `deep_mtr/test/test_mtr_inference.cpp`,
  `deep_mtr/tools/export_mtr_onnx.py`.

Named-tensor Deep ROS changes:

- Core: `deep_core/include/deep_core/types/tensor_map.hpp` (new),
  `deep_core/include/deep_core/plugin_interfaces/backend_inference_executor.hpp`,
  `deep_core/src/backend_inference_executor.cpp`,
  `deep_core/include/deep_core/deep_node_base.hpp`, `deep_core/src/deep_node_base.cpp`.
- CPU ORT: `deep_ort_backend_plugin/include/deep_ort_backend_plugin/ort_backend_executor.hpp`,
  `deep_ort_backend_plugin/src/ort_backend_executor.cpp`,
  `deep_ort_backend_plugin/test/test_ort_backend.cpp`.
- GPU ORT: `deep_ort_gpu_backend_plugin/include/deep_ort_gpu_backend_plugin/ort_gpu_backend_executor.hpp`,
  `deep_ort_gpu_backend_plugin/src/ort_gpu_backend_executor.cpp`,
  `deep_ort_gpu_backend_plugin/test/test_ort_gpu_backend.cpp`.
- Test support: `deep_test/include/test_fixtures/test_executor_fixture.hpp`,
  `deep_test/src/test_executor_fixture.cpp`, `deep_test/test/deep_core/test_integration.cpp`.

Current `wato_monorepo` dependency files:

- `config/deep_ros.ref`
- `docker/world_modeling.Dockerfile`

The export spike comes first. It records every binding and coordinate convention, replaces string
object types with numeric IDs, and proves execution with the deployment provider. The decoder then
converts centered MTR trajectories to world-frame `MtrPredictionArray` results.

### Person C: WATO runtime and integration

Current files under `src/world_modeling/prediction_ml/`:

- Fallback: `include/prediction_ml/fallback_prediction.hpp`, `src/fallback_prediction.cpp`,
  `test/test_fallback_prediction.cpp`.
- Result cache: `include/prediction_ml/mtr_result_cache.hpp`, `src/mtr_result_cache.cpp`,
  `test/test_mtr_result_cache.cpp`.
- Node: `include/prediction_ml/prediction_ml_node.hpp`, `src/prediction_ml_node.cpp`,
  `src/prediction_ml_main.cpp`, `test/test_prediction_ml_node.cpp`.
- Package: `config/params.yaml`, `launch/prediction_ml.launch.yaml`, `CMakeLists.txt`,
  `package.xml`, `README.md`, `DEVELOPING.md`.

Other current `wato_monorepo` files:

- `src/world_modeling/world_modeling_bringup/launch/world_modeling.launch.yaml`
- `modules/docker-compose.yaml`
- `watod-config.sh`

New WATO result adapter:

- `src/world_modeling/prediction_ml/include/prediction_ml/mtr_result_adapter.hpp`
- `src/world_modeling/prediction_ml/src/mtr_result_adapter.cpp`
- `src/world_modeling/prediction_ml/test/test_mtr_result_adapter.cpp`

Shared-message verification in `deep_ros`:

- `deep_msgs/CMakeLists.txt`
- `deep_msgs/package.xml`
- `deep_msgs/test/test_mtr_messages.cpp`

Person C keeps MTR disabled by default, preserves fallback, rejects invalid results, and converts
the native MTR horizon to `prediction_horizon` and `prediction_time_step`.

## Handoff order

1. Person B exports MTR and records the complete contract.
2. Person B adds and tests named multi-tensor execution.
3. Person A implements history and input packing against that contract.
4. Person B implements inference and world-frame decoding.
5. Person B merges Deep ROS changes and updates the monorepo pin/image.
6. Person C adds horizon adaptation and validates fallback, correlation, expiry, and deployment.

Person A owns input messages, Person B owns output messages, and Person C reviews both for WATO
compatibility. Correlation changes also require review from the owner on the other side.

## Data flow

```text
detections + ego + lanelets --> WATO adapter --> MtrScene --> history/input packing
detections --------------------> fallback                         |
                                                                  v
                                                        ONNX Runtime / TensorRT
                                                                  |
                                                                  v
fallback <------------------------------- validate MtrPredictionArray
  |
  +--> select/resample --> world_object_seeds
```

Until the inference gates pass, `deep_mtr` publishes no result and selection always uses fallback.

## Build and test

Use the world-modeling image so pinned Deep ROS packages are present:

```bash
colcon build --packages-select prediction_ml --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select prediction_ml
colcon test-result --verbose
```

Focused tests are `test_mtr_scene_adapter`, `test_mtr_result_cache`,
`test_prediction_ml_node`, and `test_fallback_prediction`. Run the complete module check with:

```bash
./watod test world_modeling prediction_ml
```

## Runtime smoke checks

Fallback-only:

```bash
ros2 launch prediction_ml prediction_ml.launch.yaml
ros2 lifecycle set /world_modeling/prediction_ml_node configure
ros2 lifecycle set /world_modeling/prediction_ml_node activate
ros2 param get /world_modeling/prediction_ml_node mtr.enabled
ros2 topic echo /world_modeling/world_object_seeds --once
```

Opt-in skeleton:

```bash
ros2 launch prediction_ml prediction_ml.launch.yaml enable_mtr:=true
```

Scenes may appear on `/mtr/scenes`, but `/mtr/predictions` must remain silent until inference is
implemented. Missing models, lifecycle failures, and expired results must never suppress fallback.

# Developing prediction_ml

## Current migration state

The repository-boundary migration is complete, but learned MTR inference is not. `prediction_ml`
owns the WATO bridge and always produces synchronous constant-velocity fallback output. The pinned
`deep_ros/deep_mtr` package accepts semantic scenes but deliberately publishes no predictions.

This guide is based on:

- official `sshaoshuai/MTR` commit `a5ba7bdafa09a1a355cc34f8a895499a2b14ddb3`;
- pinned `WATonomous/deep_ros` commit `c7e40a1c8c51e7d1cac48fb59a285d3af4915e38`;
  and
- the current post-migration `prediction_ml` source tree.

The world-modeling image pins Deep ROS through `config/deep_ros.ref`. The pin changes only after a
reviewed Deep ROS revision has merged and passed its contract and backend tests.

## Verified implementation constraints

Official MTR does not accept a single image-style tensor. Its Waymo preprocessing builds centered
agent-history and map-polyline inputs separately. The context encoder consumes:

- `obj_trajs`, `obj_trajs_mask`, and `obj_trajs_last_pos`;
- `map_polylines`, `map_polylines_mask`, and `map_polylines_center`;
- `track_index_to_predict`; and
- center-object type metadata used to select decoder intention points.

The reference configuration uses 29 agent attributes, nine map attributes, 20 points per
polyline, and up to 768 source polylines. It returns six modes over 80 future frames with seven
trajectory features. These values describe the official reference; the verified exported ONNX
contract remains authoritative for deployment names, dtypes, dimensions, and limits.

The official decoder returns mode scores and trajectories in each target object's centered frame.
Deep MTR must rotate and translate them into the source world frame before publishing
`MtrPredictionArray`. The output contract must also define numeric object-type IDs and how velocity
or path tangents determine `PoseStamped` orientation.

Two gates block working inference today:

1. Official MTR uses custom CUDA attention and KNN operations. Person B must prove that the
   inference graph exports and executes with ONNX Runtime CUDA and its TensorRT execution provider.
2. Pinned Deep ROS exposes `Tensor run_inference(Tensor&)`; its CPU and GPU ONNX Runtime executors
   bind only input 0 and output 0. Person B must add backward-compatible named multi-input and
   multi-output execution before integrating the exported MTR graph.

Do not pack unrelated inputs into one anonymous tensor, add a custom MTR backend, or load a
serialized TensorRT `.engine` directly. The intended path is an ONNX model executed through
`onnxruntime_gpu` with `Backend.execution_provider: "tensorrt"`.

## Repository boundary

`prediction_ml` owns WATO-specific message adaptation, request correlation, fallback generation,
result validation, WATO horizon selection, and final `world_model_msgs` output. `deep_mtr` owns
temporal history, official-MTR tensor preparation, model execution, and model-output decoding.
`deep_msgs` remains semantic and independent of `lanelet_msgs` and `world_model_msgs`.

## File ownership

Paths are relative to the named repository. Each file has one primary owner; required reviews do
not transfer ownership.

### Person A: scene history and input preparation

Current files in `wato_monorepo`:

| File | Responsibility |
|---|---|
| `src/world_modeling/prediction_ml/include/prediction_ml/mtr_scene_adapter.hpp` | WATO-to-neutral adapter contract |
| `src/world_modeling/prediction_ml/src/mtr_scene_adapter.cpp` | Detection, ego-pose, and lanelet conversion |
| `src/world_modeling/prediction_ml/test/test_mtr_scene_adapter.cpp` | Adapter and semantic-map tests |

Current files in `deep_ros`:

| File | Responsibility |
|---|---|
| `deep_msgs/msg/MapPolyline.msg` | Neutral map representation |
| `deep_msgs/msg/MtrScene.msg` | MTR request and input contract |

Files Person A creates in `deep_ros`:

| File | Responsibility |
|---|---|
| `deep_mtr/include/deep_mtr/mtr_scene_history.hpp` | Per-track and ego history contract |
| `deep_mtr/src/mtr_scene_history.cpp` | Timestamped history, expiry, and reset behavior |
| `deep_mtr/test/test_mtr_scene_history.cpp` | History and derived-motion tests |
| `deep_mtr/include/deep_mtr/mtr_agent_input_packer.hpp` | Official centered-agent input contract |
| `deep_mtr/src/mtr_agent_input_packer.cpp` | Agent features, masks, target indices, and numeric type IDs |
| `deep_mtr/test/test_mtr_agent_input_packer.cpp` | Agent shape, dtype, ordering, mask, and centering tests |
| `deep_mtr/include/deep_mtr/mtr_map_input_packer.hpp` | Official centered-map input contract |
| `deep_mtr/src/mtr_map_input_packer.cpp` | Polyline selection, direction, previous-point, mask, and center features |
| `deep_mtr/test/test_mtr_map_input_packer.cpp` | Map shape, dtype, ordering, mask, and centering tests |

Person A must derive velocity and acceleration from timestamped history when detections do not
provide them, map WATO classes to numeric vehicle/pedestrian/cyclist IDs, represent ego as the MTR
SDC input, and select target agents deterministically. Lanelet center/left/right polylines must map
to documented exported feature IDs without claiming that WATO supplies every Waymo roadgraph type.

### Person B: model export, inference, and decoding

Current files in `deep_ros`:

| File | Responsibility |
|---|---|
| `deep_msgs/msg/MtrTrajectory.msg` | One modal trajectory |
| `deep_msgs/msg/MtrObjectPrediction.msg` | Per-object prediction result |
| `deep_msgs/msg/MtrPredictionArray.msg` | Correlated scene result |
| `deep_mtr/include/deep_mtr/deep_mtr_node.hpp` | Lifecycle and inference orchestration contract |
| `deep_mtr/src/deep_mtr_node.cpp` | Scene subscription, orchestration, and result publication |
| `deep_mtr/config/deep_mtr.yaml` | Model and backend parameters |
| `deep_mtr/launch/deep_mtr.launch.yaml` | Standalone launch |
| `deep_mtr/CMakeLists.txt` | Package targets and tests |
| `deep_mtr/package.xml` | Package dependencies |
| `deep_mtr/test/test_deep_mtr_node.cpp` | Lifecycle and no-fabricated-output behavior |
| `deep_mtr/README.md` | User-facing package status |
| `deep_mtr/DEVELOPING.md` | Deep MTR implementation guidance |

Files Person B creates for MTR in `deep_ros`:

| File | Responsibility |
|---|---|
| `deep_mtr/include/deep_mtr/mtr_model_contract.hpp` | Verified tensor names, dtypes, dimensions, and type IDs |
| `deep_mtr/test/test_mtr_model_contract.cpp` | Exported-model contract checks |
| `deep_mtr/include/deep_mtr/mtr_output_decoder.hpp` | Model-output decoder contract |
| `deep_mtr/src/mtr_output_decoder.cpp` | Scores/trajectories decoding and centered-to-world conversion |
| `deep_mtr/test/test_mtr_output_decoder.cpp` | Confidence, trajectory, frame, orientation, and invalid-output tests |
| `deep_mtr/test/test_mtr_inference.cpp` | Real-model correctness and backend integration tests |
| `deep_mtr/tools/export_mtr_onnx.py` | Reproducible official-MTR checkpoint export |

Files Person B owns for backward-compatible named tensors in `deep_ros`:

| File | Change |
|---|---|
| `deep_core/include/deep_core/types/tensor_map.hpp` (new) | Named tensor collection with per-tensor dtype and shape |
| `deep_core/include/deep_core/plugin_interfaces/backend_inference_executor.hpp` | Add named multi-tensor execution |
| `deep_core/src/backend_inference_executor.cpp` | Validate and dispatch named tensors |
| `deep_core/include/deep_core/deep_node_base.hpp` | Expose the named-tensor overload |
| `deep_core/src/deep_node_base.cpp` | Forward named tensors to the backend |
| `deep_ort_backend_plugin/include/deep_ort_backend_plugin/ort_backend_executor.hpp` | CPU ONNX Runtime contract |
| `deep_ort_backend_plugin/src/ort_backend_executor.cpp` | Bind all named CPU inputs and outputs |
| `deep_ort_backend_plugin/test/test_ort_backend.cpp` | Single- and multi-tensor CPU tests |
| `deep_ort_gpu_backend_plugin/include/deep_ort_gpu_backend_plugin/ort_gpu_backend_executor.hpp` | GPU/TensorRT contract |
| `deep_ort_gpu_backend_plugin/src/ort_gpu_backend_executor.cpp` | Bind all named GPU inputs and outputs |
| `deep_ort_gpu_backend_plugin/test/test_ort_gpu_backend.cpp` | CUDA/TensorRT named-tensor tests |
| `deep_test/include/test_fixtures/test_executor_fixture.hpp` | Reusable multi-tensor fixture contract |
| `deep_test/src/test_executor_fixture.cpp` | Multi-tensor fixture implementation |
| `deep_test/test/deep_core/test_integration.cpp` | Core-to-plugin multi-tensor integration coverage |

Person B also owns these dependency files in `wato_monorepo`:

| File | Responsibility |
|---|---|
| `config/deep_ros.ref` | Reviewed immutable Deep ROS revision |
| `docker/world_modeling.Dockerfile` | Deep ROS checkout and inference runtime dependencies |

The export spike comes first. It must replace official MTR string object types with documented
numeric IDs, record every binding and coordinate convention, and execute successfully with the
deployment provider before the node claims working inference.

### Person C: WATO runtime, fallback, and integration

Current files under `wato_monorepo/src/world_modeling/prediction_ml/`:

| Area | Files |
|---|---|
| Fallback | `include/prediction_ml/fallback_prediction.hpp`, `src/fallback_prediction.cpp`, `test/test_fallback_prediction.cpp` |
| Result validation/cache | `include/prediction_ml/mtr_result_cache.hpp`, `src/mtr_result_cache.cpp`, `test/test_mtr_result_cache.cpp` |
| Lifecycle node | `include/prediction_ml/prediction_ml_node.hpp`, `src/prediction_ml_node.cpp`, `src/prediction_ml_main.cpp`, `test/test_prediction_ml_node.cpp` |
| Package wiring | `config/params.yaml`, `launch/prediction_ml.launch.yaml`, `CMakeLists.txt`, `package.xml` |
| Package documentation | `README.md`, `DEVELOPING.md` |

Other current files in `wato_monorepo`:

| File | Responsibility |
|---|---|
| `src/world_modeling/world_modeling_bringup/launch/world_modeling.launch.yaml` | World-modeling launch integration |
| `modules/docker-compose.yaml` | GPU and device service configuration |
| `watod-config.sh` | Deep ROS build/ref argument wiring |

Files Person C creates in `wato_monorepo`:

| File | Responsibility |
|---|---|
| `src/world_modeling/prediction_ml/include/prediction_ml/mtr_result_adapter.hpp` | Native-MTR-to-WATO horizon contract |
| `src/world_modeling/prediction_ml/src/mtr_result_adapter.cpp` | Timestamp-aware truncation/resampling |
| `src/world_modeling/prediction_ml/test/test_mtr_result_adapter.cpp` | Horizon, interpolation, frame, and timestamp tests |

Person C also owns shared-message build verification in `deep_ros`:

| File | Responsibility |
|---|---|
| `deep_msgs/CMakeLists.txt` | MTR interface generation and test registration |
| `deep_msgs/package.xml` | Shared message dependencies |
| `deep_msgs/test/test_mtr_messages.cpp` | Request/result correlation contract |

Person C must keep MTR disabled by default, preserve synchronous fallback output, reject stale or
malformed results, and convert the model-native horizon to `prediction_horizon` and
`prediction_time_step` without changing the reusable Deep ROS output contract.

## Handoff and change control

Implement in this order:

1. Person B exports the official MTR model and records its complete input/output contract.
2. Person B adds backward-compatible named multi-tensor support and verifies CPU, CUDA, and
   TensorRT-provider behavior.
3. Person A implements history and agent/map packing against the verified export.
4. Person B implements inference and world-frame decoding, then proves real-model behavior.
5. Person B updates the reviewed Deep ROS pin and image dependencies after the Deep ROS changes
   merge.
6. Person C implements WATO horizon adaptation and verifies fallback, correlation, expiry, launch,
   and deployment behavior.

Person A owns input messages, Person B owns output messages, and Person C verifies that both remain
consumable by the WATO bridge. Changes to MTR messages require Person C's review; correlation
changes also require review from the owner on the other side of the request/result boundary.

## Data flow

```text
tracked detections --+
ego pose ------------+--> WATO scene adapter --> MtrScene --> Deep MTR history/input packing
lanelet context ------+                                      |
                                                             v
tracked detections --> constant-velocity fallback       ONNX Runtime / TensorRT EP
        |                                                    |
        |                                                    v
        +------------------------------ MtrPredictionArray <-+
                                        |
                                        v
                              validate, cache, and resample
                                        |
                                        v
                               world_object_seeds
```

Until all inference gates pass, `deep_mtr` publishes no result and the lower path always resolves
to fallback output.

## Build and test

Use the world-modeling image so the pinned Deep ROS packages are present:

```bash
colcon build --packages-select prediction_ml --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select prediction_ml
colcon test-result --verbose
```

Focused targets cover:

- lanelet and optional-context conversion in `test_mtr_scene_adapter`;
- correlation, validation, expiry, and replacement in `test_mtr_result_cache`;
- fallback-only lifecycle behavior in `test_prediction_ml_node`; and
- fallback speed selection in `test_fallback_prediction`.

The complete module check is:

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

After configuring and activating both lifecycle nodes, scenes may appear on `/mtr/scenes`, but
`/mtr/predictions` must remain silent until inference is implemented. A missing model, failed Deep
ROS lifecycle transition, or expired result must never suppress fallback output.

# Prediction ML Post-Migration Ownership Design

## Goal

Replace the obsolete pre-migration collaboration split with a post-migration Person A/B/C
ownership model that matches the current `wato_monorepo` and pinned `deep_ros` repositories.

## Current State

The migration is complete at the repository-boundary level, but learned MTR inference is not.
`prediction_ml` publishes constant-velocity fallback trajectories and optionally exchanges neutral
`MtrScene`/`MtrPredictionArray` messages with `deep_ros`. At pinned Deep ROS revision
`c7e40a1c8c51e7d1cac48fb59a285d3af4915e38`, `deep_mtr` is a lifecycle skeleton that deliberately
does not run inference or publish predictions.

The pre-migration files for local scene tensor construction, custom inference backends, runtime,
and output conversion no longer exist. Ownership must follow the new pipeline and repository
boundary instead of referring to deleted files.

## Official MTR Validation

The ownership split is based on the official `sshaoshuai/MTR` implementation at commit
`a5ba7bdafa09a1a355cc34f8a895499a2b14ddb3`, not only on the migration scaffold.

- `mtr/datasets/waymo/waymo_dataset.py` separately builds centered agent history and map-polyline
  features. This supports assigning agent/map preprocessing to Person A.
- `mtr/models/context_encoder/mtr_encoder.py` consumes `obj_trajs`, `obj_trajs_mask`,
  `obj_trajs_last_pos`, `map_polylines`, `map_polylines_mask`, `map_polylines_center`, and
  `track_index_to_predict`. The decoder also requires encoded `center_objects_type` metadata.
- The official Waymo configuration uses 29 agent attributes, nine map attributes, 20 points per
  polyline, and up to 768 source polylines. The exported ONNX contract, rather than these reference
  values alone, remains authoritative for deployment shapes.
- `mtr/models/motion_decoder/mtr_decoder.py` produces `pred_scores` and `pred_trajs`; the standard
  configuration uses six returned modes, 80 future frames, and seven trajectory features.
- `WaymoDataset.generate_prediction_dicts` rotates predicted trajectories out of each target
  object's centered frame and translates them into the world frame before publishing x/y paths.
  That transform belongs to Person B's decoder.

The pinned Deep ROS revision currently exposes only `Tensor run_inference(Tensor&)`, and both the
CPU and GPU ONNX Runtime executors bind only input 0 and output 0. That API cannot represent the
official MTR model's heterogeneous multi-input/multi-output contract. Working inference therefore
requires a named multi-tensor Deep ROS extension before `deep_mtr` can call the exported model.
Packing unrelated inputs into one anonymous tensor is rejected because it erases dtype, name, and
shape validation at the backend boundary.

The export spike must also resolve official MTR's custom CUDA attention/KNN operations before the
team treats ONNX Runtime or its TensorRT execution provider as viable. The repository must not
claim working MTR based only on a graph that exports but cannot execute with the deployment
provider.

The semantic ROS boundary intentionally does not carry model tensors, so Person A must define and
test these preprocessing policies inside `deep_mtr`:

- derive velocity and acceleration from timestamped per-track history when detections do not carry
  those fields;
- map WATO detection classes to numeric vehicle, pedestrian, and cyclist IDs;
- represent the ego vehicle as the official MTR SDC input using ego-pose history and documented
  defaults for unavailable dimensions;
- select target agents deterministically when a scene contains more objects than the exported
  model contract accepts; and
- convert the three available WATO lanelet semantics into the verified exported map-feature IDs
  without pretending the bridge supplies every Waymo roadgraph feature.

## Ownership Model

Paths below are relative to the named repository. Every current MTR-related file has one primary
owner. Reviews from another owner do not transfer primary ownership.

### Person A: scene preparation

Person A owns WATO-to-neutral scene adaptation in `wato_monorepo`:

| Current file | Responsibility |
|---|---|
| `src/world_modeling/prediction_ml/include/prediction_ml/mtr_scene_adapter.hpp` | Adapter contract |
| `src/world_modeling/prediction_ml/src/mtr_scene_adapter.cpp` | Detection, ego-pose, and lanelet conversion |
| `src/world_modeling/prediction_ml/test/test_mtr_scene_adapter.cpp` | Adapter and semantic-map tests |

Person A owns the Deep ROS input messages and the model-specific preprocessing modules that must be
created in `deep_ros`:

| Current Deep ROS file | Responsibility |
|---|---|
| `deep_msgs/msg/MapPolyline.msg` | Neutral map representation |
| `deep_msgs/msg/MtrScene.msg` | MTR request/input contract |

| New Deep ROS file to create | Responsibility |
|---|---|
| `deep_mtr/include/deep_mtr/mtr_scene_history.hpp` | Per-track temporal history contract |
| `deep_mtr/src/mtr_scene_history.cpp` | History accumulation and reset behavior |
| `deep_mtr/test/test_mtr_scene_history.cpp` | History correctness tests |
| `deep_mtr/include/deep_mtr/mtr_agent_input_packer.hpp` | Official centered-agent input contract |
| `deep_mtr/src/mtr_agent_input_packer.cpp` | Agent features, masks, last positions, target indices, and type IDs |
| `deep_mtr/test/test_mtr_agent_input_packer.cpp` | Agent shape, dtype, ordering, mask, and centering tests |
| `deep_mtr/include/deep_mtr/mtr_map_input_packer.hpp` | Official centered-map input contract |
| `deep_mtr/src/mtr_map_input_packer.cpp` | Polyline sampling, selection, direction, previous-point, mask, and center features |
| `deep_mtr/test/test_mtr_map_input_packer.cpp` | Map shape, dtype, ordering, mask, and centering tests |

Person A must keep WATO-specific lanelet conversion in `prediction_ml` and MTR tensor layout in
`deep_mtr`.

### Person B: inference and decoding

Person B owns the Deep ROS node, backend use, model contract, and output decoding. The following
current `deep_ros` files are Person B's starting surface:

| Current Deep ROS file | Responsibility |
|---|---|
| `deep_msgs/msg/MtrTrajectory.msg` | One decoded modal trajectory |
| `deep_msgs/msg/MtrObjectPrediction.msg` | Per-object decoded result |
| `deep_msgs/msg/MtrPredictionArray.msg` | Correlated result contract |
| `deep_mtr/include/deep_mtr/deep_mtr_node.hpp` | Lifecycle-node and orchestration contract |
| `deep_mtr/src/deep_mtr_node.cpp` | Subscription, inference orchestration, and publication |
| `deep_mtr/config/deep_mtr.yaml` | Model and backend configuration |
| `deep_mtr/launch/deep_mtr.launch.yaml` | Standalone Deep MTR launch |
| `deep_mtr/CMakeLists.txt` | Deep MTR targets and tests |
| `deep_mtr/package.xml` | Deep MTR dependencies |
| `deep_mtr/test/test_deep_mtr_node.cpp` | Lifecycle and no-fabricated-output tests |
| `deep_mtr/README.md` | User-facing Deep MTR status |
| `deep_mtr/DEVELOPING.md` | Deep MTR implementation boundary |

Person B creates and owns these files when implementing inference:

| New Deep ROS file to create | Responsibility |
|---|---|
| `deep_mtr/include/deep_mtr/mtr_model_contract.hpp` | Input/output tensor names, dtypes, and dimensions |
| `deep_mtr/test/test_mtr_model_contract.cpp` | Exported-model contract verification |
| `deep_mtr/include/deep_mtr/mtr_output_decoder.hpp` | Decoder contract |
| `deep_mtr/src/mtr_output_decoder.cpp` | Scores/trajectories decoding and centered-to-world transform |
| `deep_mtr/test/test_mtr_output_decoder.cpp` | Confidence, trajectory, frame-transform, and invalid-output tests |
| `deep_mtr/test/test_mtr_inference.cpp` | Real-model correctness and backend integration tests |
| `deep_mtr/tools/export_mtr_onnx.py` | Reproducible official-MTR checkpoint export |

The export contract must replace official MTR's string `center_objects_type` values with documented
numeric type IDs suitable for ONNX/TensorRT. The output decoder publishes mode confidence and x/y
paths from `pred_scores`/`pred_trajs`, applies the official world-frame transform, and explicitly
documents how velocity or path tangents determine `PoseStamped` orientation.

Before those files can run the official model, Person B owns the named multi-tensor extension in
Deep ROS:

| Deep ROS file | Change |
|---|---|
| `deep_core/include/deep_core/types/tensor_map.hpp` (new) | Named tensor collection with per-tensor dtype and shape |
| `deep_core/include/deep_core/plugin_interfaces/backend_inference_executor.hpp` | Add multi-input/multi-output execution without removing the single-tensor API |
| `deep_core/src/backend_inference_executor.cpp` | Validate and dispatch named tensor collections |
| `deep_core/include/deep_core/deep_node_base.hpp` | Expose the compatible named-tensor inference overload |
| `deep_core/src/deep_node_base.cpp` | Forward the overload to the selected backend |
| `deep_ort_backend_plugin/include/deep_ort_backend_plugin/ort_backend_executor.hpp` | CPU ONNX Runtime multi-tensor contract |
| `deep_ort_backend_plugin/src/ort_backend_executor.cpp` | Bind all named model inputs and outputs |
| `deep_ort_backend_plugin/test/test_ort_backend.cpp` | Preserve single-tensor behavior and test named tensors |
| `deep_ort_gpu_backend_plugin/include/deep_ort_gpu_backend_plugin/ort_gpu_backend_executor.hpp` | GPU/TensorRT multi-tensor contract |
| `deep_ort_gpu_backend_plugin/src/ort_gpu_backend_executor.cpp` | Bind all named GPU inputs and outputs |
| `deep_ort_gpu_backend_plugin/test/test_ort_gpu_backend.cpp` | CUDA/TensorRT named-tensor integration tests |
| `deep_test/include/test_fixtures/test_executor_fixture.hpp` | Reusable multi-tensor backend fixture |
| `deep_test/src/test_executor_fixture.cpp` | Fixture implementation |
| `deep_test/test/deep_core/test_integration.cpp` | Core-to-plugin multi-tensor integration coverage |

Person B calls the new named-tensor `DeepNodeBase::run_inference` overload through
`onnxruntime_gpu` with the TensorRT execution provider. Person B must preserve existing
single-tensor consumers and must not add a custom backend or direct serialized TensorRT engine
loader.

In `wato_monorepo`, Person B owns the external inference dependency surface:

| Current file | Responsibility |
|---|---|
| `config/deep_ros.ref` | Reviewed immutable Deep ROS revision |
| `docker/world_modeling.Dockerfile` | Deep ROS checkout and inference runtime dependencies |

### Person C: WATO runtime integration and fallback

Person C owns all remaining `prediction_ml` runtime, selection, build, and package files in
`wato_monorepo`:

| Current file group | Files |
|---|---|
| Fallback | `include/prediction_ml/fallback_prediction.hpp`, `src/fallback_prediction.cpp`, `test/test_fallback_prediction.cpp` |
| Result selection | `include/prediction_ml/mtr_result_cache.hpp`, `src/mtr_result_cache.cpp`, `test/test_mtr_result_cache.cpp` |
| Lifecycle node | `include/prediction_ml/prediction_ml_node.hpp`, `src/prediction_ml_node.cpp`, `src/prediction_ml_main.cpp`, `test/test_prediction_ml_node.cpp` |
| Package wiring | `config/params.yaml`, `launch/prediction_ml.launch.yaml`, `CMakeLists.txt`, `package.xml` |
| Package docs | `README.md`, `DEVELOPING.md` |

All paths in that table are under `src/world_modeling/prediction_ml/`. Person C also owns these
monorepo integration files:

| Current file | Responsibility |
|---|---|
| `src/world_modeling/world_modeling_bringup/launch/world_modeling.launch.yaml` | World-modeling launch integration |
| `modules/docker-compose.yaml` | GPU/device service configuration |
| `watod-config.sh` | Deep ROS build/ref argument wiring |

Person C preserves synchronous fallback output, validates and correlates returned predictions, and
keeps MTR disabled by default until the learned path meets its acceptance criteria.

Person C creates and owns a WATO-side result adapter so Deep ROS can publish the model-native MTR
horizon while the monorepo retains consumer-specific output policy:

| New monorepo file to create | Responsibility |
|---|---|
| `src/world_modeling/prediction_ml/include/prediction_ml/mtr_result_adapter.hpp` | Native-result-to-WATO horizon conversion contract |
| `src/world_modeling/prediction_ml/src/mtr_result_adapter.cpp` | Timestamp-aware truncation/resampling to `prediction_horizon` and `prediction_time_step` |
| `src/world_modeling/prediction_ml/test/test_mtr_result_adapter.cpp` | Horizon, interpolation, frame, and malformed-timestamp tests |

In `deep_ros`, Person C owns build-level verification of the shared MTR message boundary:

| Current Deep ROS file | Responsibility |
|---|---|
| `deep_msgs/CMakeLists.txt` | MTR interface generation and contract-test registration |
| `deep_msgs/package.xml` | Shared message dependencies |
| `deep_msgs/test/test_mtr_messages.cpp` | End-to-end request/result correlation contract |

### Coordinated changes

Person A is the primary owner of input messages and Person B is the primary owner of output
messages. Person C verifies that all message changes remain consumable by the WATO bridge. A change
to a `deep_msgs` MTR message requires review from Person C and from the owner on the other side of
the request/result boundary when correlation semantics change.

## Documentation Changes

- Update `prediction_ml/README.md` with the post-migration progress state and a short Person A/B/C
  next-work summary.
- Update `prediction_ml/DEVELOPING.md` with exact current file ownership, Deep ROS ownership,
  coordinated-file rules, handoff order, and completion criteria.
- Remove language that assigns work to deleted pre-migration abstractions or suggests inference is
  still implemented inside `prediction_ml`.

## Handoff Order

1. Person B exports the official MTR model and records every input/output name, dtype, shape, and
   coordinate-frame convention.
2. Person B adds backward-compatible named multi-tensor execution to Deep ROS and verifies CPU,
   CUDA, and TensorRT-provider behavior.
3. Person A implements history plus agent/map input packing against the verified exported contract
   while maintaining the WATO scene adapter. This includes derived motion state, numeric type
   mapping, ego/SDC representation, and deterministic target selection.
4. Person B implements decoding, including the official center-to-world transform, and proves
   real-model behavior in Deep ROS.
5. Person B updates the monorepo's Deep ROS pin and image dependencies after the Deep ROS work
   merges.
6. Person C adds WATO horizon conversion, verifies bridge correlation/fallback behavior, and
   enables deployment only after the learned path meets its acceptance criteria.

## Verification

- Every current `prediction_ml` production, test, build, configuration, and documentation file is
  assigned to Person A or Person C.
- Every current Deep ROS MTR source, message, test, build, configuration, launch, and documentation
  file is assigned to Person A, B, or C; proposed modules are explicitly labeled as new files.
- README and development-guide status statements agree that migration scaffolding exists but
  working MTR inference does not.
- No documentation claims direct `.engine` loading or enabled-by-default MTR deployment.
- Documentation records the official MTR reference revision and the unresolved multi-tensor API
  prerequisite; it does not claim the current single-tensor Deep ROS API can run MTR.

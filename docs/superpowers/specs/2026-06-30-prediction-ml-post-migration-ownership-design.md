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
| `deep_mtr/include/deep_mtr/mtr_input_packer.hpp` | Verified ONNX input-packing contract |
| `deep_mtr/src/mtr_input_packer.cpp` | History/map-to-tensor packing |
| `deep_mtr/test/test_mtr_input_packer.cpp` | Tensor shape, dtype, ordering, and mask tests |

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
| `deep_mtr/src/mtr_output_decoder.cpp` | Tensor-to-`MtrPredictionArray` decoding |
| `deep_mtr/test/test_mtr_output_decoder.cpp` | Confidence, trajectory, and invalid-output tests |
| `deep_mtr/test/test_mtr_inference.cpp` | Real-model correctness and backend integration tests |

Person B calls `DeepNodeBase::run_inference` through `onnxruntime_gpu` with the TensorRT execution
provider. Person B must not add a custom backend or direct serialized TensorRT engine loader.

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

1. Person A and Person B agree on the verified ONNX input/output contract.
2. Person A implements history and input packing in `deep_mtr` while maintaining the WATO scene
   adapter.
3. Person B implements inference and decoding and proves real-model behavior in Deep ROS.
4. Person B updates the monorepo's Deep ROS pin and image dependencies after the Deep ROS work
   merges.
5. Person C verifies bridge correlation/fallback behavior and enables deployment only after the
   learned path meets its acceptance criteria.

## Verification

- Every current `prediction_ml` production, test, build, configuration, and documentation file is
  assigned to Person A or Person C.
- Every current Deep ROS MTR source, message, test, build, configuration, launch, and documentation
  file is assigned to Person A, B, or C; proposed modules are explicitly labeled as new files.
- README and development-guide status statements agree that migration scaffolding exists but
  working MTR inference does not.
- No documentation claims direct `.engine` loading or enabled-by-default MTR deployment.

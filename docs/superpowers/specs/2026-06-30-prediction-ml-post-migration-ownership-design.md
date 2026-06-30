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

### Person A: scene preparation

Person A owns semantic input preparation in the monorepo and model-specific input preparation in
Deep ROS:

- `prediction_ml/include/prediction_ml/mtr_scene_adapter.hpp`
- `prediction_ml/src/mtr_scene_adapter.cpp`
- `prediction_ml/test/test_mtr_scene_adapter.cpp`
- future `deep_mtr` per-track history and ONNX input-packing components and their tests

Person A must keep WATO-specific lanelet conversion in `prediction_ml` and model-specific history
and tensor layout in `deep_mtr`.

### Person B: inference and decoding

Person B owns the working learned-inference path in `deep_ros/deep_mtr`:

- integration with `DeepNodeBase::run_inference`
- ONNX model configuration and the `onnxruntime_gpu` backend with TensorRT execution provider
- output validation and decoding into `deep_msgs/MtrPredictionArray`
- model-contract, correctness, and GPU tests

The current `deep_mtr_node.hpp`, `deep_mtr_node.cpp`, configuration, launch file, build metadata,
and skeleton test are Person B's starting surface. Person B should extract preprocessing modules
owned by Person A as the package grows. Person B must not introduce direct serialized TensorRT
engine loading.

### Person C: WATO runtime integration and fallback

Person C owns the monorepo runtime and deployment-facing integration:

- fallback prediction header, source, and test
- MTR result cache header, source, and test
- lifecycle node header/source/main and node test
- `prediction_ml` parameters, launch file, `CMakeLists.txt`, and `package.xml`
- world-modeling launch integration and the reviewed `config/deep_ros.ref` pin update after Deep ROS
  work merges

Person C preserves synchronous fallback output, validates/correlates returned predictions, and
ensures MTR remains disabled by default until the learned path satisfies its acceptance criteria.

### Coordinated contracts

Changes to `deep_msgs` MTR messages affect both repositories. Person A proposes input-contract
changes, Person B proposes output-contract changes, and Person C reviews compatibility with the
WATO bridge. Contract changes require agreement from all affected owners before either repository
depends on them.

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
4. Person C pins the merged Deep ROS revision, verifies bridge correlation/fallback behavior, and
   enables deployment only after the learned path meets its acceptance criteria.

## Verification

- Every current `prediction_ml` production and test file is assigned or explicitly coordinated.
- Deep ROS assignments name only files present at the pinned revision and label future modules as
  future files.
- README and development-guide status statements agree that migration scaffolding exists but
  working MTR inference does not.
- No documentation claims direct `.engine` loading or enabled-by-default MTR deployment.

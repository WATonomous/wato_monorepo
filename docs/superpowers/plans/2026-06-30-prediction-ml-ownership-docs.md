# Prediction ML Ownership Documentation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Update the prediction ML documentation with an accurate post-migration status and exact Person A/B/C file ownership across `wato_monorepo` and `deep_ros`.

**Architecture:** Keep `README.md` operational and concise: current behavior, the critical inference prerequisite, and a three-row ownership summary. Put official-MTR findings, complete current/proposed file maps, implementation order, and handoff rules in `DEVELOPING.md`.

**Tech Stack:** Markdown, ROS 2 package documentation, official `sshaoshuai/MTR` reference commit `a5ba7bdafa09a1a355cc34f8a895499a2b14ddb3`, pinned Deep ROS commit `c7e40a1c8c51e7d1cac48fb59a285d3af4915e38`.

---

## Task 1: Update the concise user-facing README

**Files:**
- Modify: `src/world_modeling/prediction_ml/README.md`

- [ ] **Step 1: Add a short current-status section**

State that repository-boundary migration is complete, `deep_mtr` remains non-inferencing, fallback remains the only deployable output, and official MTR cannot use the pinned single-tensor Deep ROS API without a backward-compatible named multi-tensor extension.

- [ ] **Step 2: Add the high-level ownership table**

Use exactly three rows:

```markdown
| Owner | Area | Next deliverable |
|---|---|---|
| Person A | Scene history and official-MTR input preparation | History plus agent/map tensor packers in `deep_ros/deep_mtr` |
| Person B | Model export, Deep ROS inference, and decoding | Verified ONNX contract, named multi-tensor execution, and world-frame results |
| Person C | WATO runtime, fallback, and deployment integration | WATO horizon adaptation, end-to-end validation, and safe enablement |
```

Link to `DEVELOPING.md` for files and handoffs. Do not copy detailed file lists into the README.

- [ ] **Step 3: Verify the README remains operational**

Run:

```bash
rg -n "Current status|Person A|Person B|Person C|DEVELOPING.md|fallback|multi-tensor" src/world_modeling/prediction_ml/README.md
```

Expected: the concise status and owner table are present, while launch/configuration sections remain intact.

### Task 2: Replace the development guide with the exact post-migration split

**Files:**
- Modify: `src/world_modeling/prediction_ml/DEVELOPING.md`

- [ ] **Step 1: Preserve the repository boundary and add verified context**

Document both reviewed revisions and these facts:

```markdown
- Official MTR separately prepares centered agent and map inputs.
- The encoder consumes multiple named tensors and target/type metadata.
- The decoder returns mode scores and centered trajectories requiring a world-frame transform.
- Pinned Deep ROS currently binds only input 0 and output 0.
- Official MTR custom CUDA attention/KNN operations remain an ONNX/TensorRT export gate.
```

- [ ] **Step 2: Add exact Person A file ownership**

Assign these current files:

```text
wato_monorepo/src/world_modeling/prediction_ml/include/prediction_ml/mtr_scene_adapter.hpp
wato_monorepo/src/world_modeling/prediction_ml/src/mtr_scene_adapter.cpp
wato_monorepo/src/world_modeling/prediction_ml/test/test_mtr_scene_adapter.cpp
deep_ros/deep_msgs/msg/MapPolyline.msg
deep_ros/deep_msgs/msg/MtrScene.msg
```

Assign these proposed Deep ROS files:

```text
deep_mtr/include/deep_mtr/mtr_scene_history.hpp
deep_mtr/src/mtr_scene_history.cpp
deep_mtr/test/test_mtr_scene_history.cpp
deep_mtr/include/deep_mtr/mtr_agent_input_packer.hpp
deep_mtr/src/mtr_agent_input_packer.cpp
deep_mtr/test/test_mtr_agent_input_packer.cpp
deep_mtr/include/deep_mtr/mtr_map_input_packer.hpp
deep_mtr/src/mtr_map_input_packer.cpp
deep_mtr/test/test_mtr_map_input_packer.cpp
```

Include velocity derivation, numeric class mapping, ego/SDC representation, deterministic target selection, and limited lanelet-semantic mapping.

- [ ] **Step 3: Add exact Person B file ownership**

Assign current Deep ROS output messages and the full current `deep_mtr` package:

```text
deep_msgs/msg/MtrTrajectory.msg
deep_msgs/msg/MtrObjectPrediction.msg
deep_msgs/msg/MtrPredictionArray.msg
deep_mtr/include/deep_mtr/deep_mtr_node.hpp
deep_mtr/src/deep_mtr_node.cpp
deep_mtr/config/deep_mtr.yaml
deep_mtr/launch/deep_mtr.launch.yaml
deep_mtr/CMakeLists.txt
deep_mtr/package.xml
deep_mtr/test/test_deep_mtr_node.cpp
deep_mtr/README.md
deep_mtr/DEVELOPING.md
```

Assign these proposed model files:

```text
deep_mtr/include/deep_mtr/mtr_model_contract.hpp
deep_mtr/test/test_mtr_model_contract.cpp
deep_mtr/include/deep_mtr/mtr_output_decoder.hpp
deep_mtr/src/mtr_output_decoder.cpp
deep_mtr/test/test_mtr_output_decoder.cpp
deep_mtr/test/test_mtr_inference.cpp
deep_mtr/tools/export_mtr_onnx.py
```

Assign these named-tensor extension files:

```text
deep_core/include/deep_core/types/tensor_map.hpp
deep_core/include/deep_core/plugin_interfaces/backend_inference_executor.hpp
deep_core/src/backend_inference_executor.cpp
deep_core/include/deep_core/deep_node_base.hpp
deep_core/src/deep_node_base.cpp
deep_ort_backend_plugin/include/deep_ort_backend_plugin/ort_backend_executor.hpp
deep_ort_backend_plugin/src/ort_backend_executor.cpp
deep_ort_backend_plugin/test/test_ort_backend.cpp
deep_ort_gpu_backend_plugin/include/deep_ort_gpu_backend_plugin/ort_gpu_backend_executor.hpp
deep_ort_gpu_backend_plugin/src/ort_gpu_backend_executor.cpp
deep_ort_gpu_backend_plugin/test/test_ort_gpu_backend.cpp
deep_test/include/test_fixtures/test_executor_fixture.hpp
deep_test/src/test_executor_fixture.cpp
deep_test/test/deep_core/test_integration.cpp
```

Assign `config/deep_ros.ref` and `docker/world_modeling.Dockerfile` in the monorepo.

- [ ] **Step 4: Add exact Person C file ownership**

Assign these current monorepo file groups:

```text
prediction_ml/include/prediction_ml/fallback_prediction.hpp
prediction_ml/src/fallback_prediction.cpp
prediction_ml/test/test_fallback_prediction.cpp
prediction_ml/include/prediction_ml/mtr_result_cache.hpp
prediction_ml/src/mtr_result_cache.cpp
prediction_ml/test/test_mtr_result_cache.cpp
prediction_ml/include/prediction_ml/prediction_ml_node.hpp
prediction_ml/src/prediction_ml_node.cpp
prediction_ml/src/prediction_ml_main.cpp
prediction_ml/test/test_prediction_ml_node.cpp
prediction_ml/config/params.yaml
prediction_ml/launch/prediction_ml.launch.yaml
prediction_ml/CMakeLists.txt
prediction_ml/package.xml
prediction_ml/README.md
prediction_ml/DEVELOPING.md
src/world_modeling/world_modeling_bringup/launch/world_modeling.launch.yaml
modules/docker-compose.yaml
watod-config.sh
```

Assign the proposed result adapter:

```text
prediction_ml/include/prediction_ml/mtr_result_adapter.hpp
prediction_ml/src/mtr_result_adapter.cpp
prediction_ml/test/test_mtr_result_adapter.cpp
```

Assign `deep_msgs/CMakeLists.txt`, `deep_msgs/package.xml`, and `deep_msgs/test/test_mtr_messages.cpp` in Deep ROS.

- [ ] **Step 5: Add handoff and change-control rules**

Record the order: export/contract, named-tensor API, input preparation, inference/decoding, Deep ROS pin/image, WATO result adaptation/validation. Require cross-owner review for shared message changes and preserve disabled-by-default fallback behavior.

- [ ] **Step 6: Preserve build, test, and smoke-check instructions**

Keep the existing commands unchanged and retain the statement that the current skeleton publishes no learned result.

### Task 3: Verify documentation consistency

**Files:**
- Verify: `src/world_modeling/prediction_ml/README.md`
- Verify: `src/world_modeling/prediction_ml/DEVELOPING.md`

- [ ] **Step 1: Check formatting and stale terminology**

Run:

```bash
git diff --check
rg -n "IMtrInferenceEngine|PREDICTION_ML_ENABLE_TENSORRT|mtr\.mode|\.engine" src/world_modeling/prediction_ml/README.md src/world_modeling/prediction_ml/DEVELOPING.md
```

Expected: `git diff --check` passes and stale pre-migration abstractions are absent. A `.engine` match is allowed only in a sentence prohibiting direct engine loading.

- [ ] **Step 2: Check required ownership and implementation gates**

Run:

```bash
rg -n "Person A|Person B|Person C|a5ba7bd|c7e40a1c|multi-tensor|custom CUDA|fallback|disabled by default" src/world_modeling/prediction_ml/README.md src/world_modeling/prediction_ml/DEVELOPING.md
```

Expected: both docs agree on status and high-level ownership; detailed revisions, risks, and files are present in `DEVELOPING.md`.

- [ ] **Step 3: Review the final diff**

Run:

```bash
git diff -- src/world_modeling/prediction_ml/README.md src/world_modeling/prediction_ml/DEVELOPING.md
```

Expected: README changes are short, DEVELOPING contains the full file split, and launch/configuration/testing guidance remains accurate.

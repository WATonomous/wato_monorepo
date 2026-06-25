# Prediction ML (MTR) Skeleton Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stand up a new `prediction_ml` ROS 2 package as a compiling skeleton with owner-scoped stub files, so 3 people can implement MTR (Motion Transformer) prediction in parallel with no merge conflicts, while the node runs as a safe constant-velocity fallback.

**Architecture:** A single lifecycle node owns three collaborators that meet only at two frozen headers (`mtr_types.hpp`, `mtr_inference_engine.hpp`): a SceneBuilder (Detection3DArray→MTR tensors, Person A), a pluggable inference engine (null + TensorRT, Person B), and an orchestration spine (runtime async-cache + output converter + node + CV fallback, Person C). The fallback publishes every tick; MTR runs async and replaces fallback per-object only when fresh/valid.

**Tech Stack:** ROS 2 Jazzy, C++17, `ament_cmake`, `rclcpp_lifecycle`, gtest (`ament_cmake_gtest`), `vision_msgs`, `world_model_msgs`, `geometry_msgs`, `lanelet_msgs`. TensorRT is optional, gated behind `PREDICTION_ML_ENABLE_TENSORRT` (off for this skeleton).

## Global Constraints

- C++ standard: **C++17** (set only if `CMAKE_CXX_STANDARD` unset, like sibling packages).
- Compile flags: `-Wall -Wextra -Wpedantic` for GNU/Clang (match `prediction/CMakeLists.txt`).
- Package name / namespace: **`prediction_ml`** package, **`prediction_ml`** C++ namespace.
- Copyright header: every source file starts with the Apache-2.0 WATonomous header block used across the repo (copy verbatim from `src/world_modeling/prediction/src/prediction_node.cpp` lines 1-13).
- I/O contract is fixed: subscribe `tracks_3d` (`vision_msgs/Detection3DArray`), publish `world_object_seeds` (`world_model_msgs/WorldObjectArray`). Optional `ego_pose` (`geometry_msgs/PoseStamped`), `lanelet_ahead` (`lanelet_msgs/LaneletAhead`).
- TensorRT OFF for the skeleton: `PREDICTION_ML_ENABLE_TENSORRT` defaults OFF; CPU-only `colcon build` must succeed and the node must run as pure CV fallback.
- The two shared headers are frozen after Task 1. Tasks 2-6 may **read** them but must not modify them.
- Build verification command (run from repo root inside the world_modeling dev container): `colcon build --packages-select prediction_ml` then `colcon test --packages-select prediction_ml`.

## File Structure

```
src/world_modeling/prediction_ml/
├── include/prediction_ml/
│   ├── mtr_types.hpp              # Task 1 (shared, frozen)
│   ├── mtr_inference_engine.hpp   # Task 1 (shared, frozen)
│   ├── mtr_backend.hpp            # Task 2 (Person B)
│   ├── scene_builder.hpp          # Task 3 (Person A)
│   ├── output_converter.hpp       # Task 4 (Person C)
│   ├── mtr_runtime.hpp            # Task 5 (Person C)
│   └── prediction_ml_node.hpp     # Task 6 (Person C)
├── src/
│   ├── null_backend.cpp           # Task 2 (Person B, always built)
│   ├── tensorrt_backend.cpp       # Task 2 (Person B, gated — stub only this milestone)
│   ├── scene_builder.cpp          # Task 3 (Person A)
│   ├── output_converter.cpp       # Task 4 (Person C)
│   ├── mtr_runtime.cpp            # Task 5 (Person C)
│   └── prediction_ml_node.cpp     # Task 6 (Person C, + main)
├── config/params.yaml             # Task 6
├── launch/prediction_ml.launch.yaml  # Task 6
├── test/
│   ├── test_backend.cpp           # Task 2 (Person B)
│   ├── test_scene_builder.cpp     # Task 3 (Person A)
│   └── test_runtime.cpp           # Task 5 (Person C)
├── CMakeLists.txt                 # Task 1, appended by each task at marked lines
├── package.xml                    # Task 1
└── README.md                      # Task 6
```

**Owner map:** A = SceneBuilder (Task 3). B = Inference backend (Task 2). C = Orchestration: converter + runtime + node (Tasks 4, 5, 6). Task 0 is a joint A+B+C verification gate; Task 1 is the lead's shared foundation that unblocks all three.

---

### Task 0: MTR model I/O verification + contract freeze sign-off (GATE before Task 1)

**Why this task exists:** Task 1 *freezes* the tensor representation in `mtr_types.hpp`.
If those struct fields/dtypes don't match the real MTR model's actual bindings, all three
people build on a wrong contract and the "frozen" header churns after the fact. This task
verifies the contract against the real model **once**, before freezing — so any amendment
lands in Task 1's code, not three branches later.

**Owners:** A + B jointly produce the evidence; A + B + C sign off together.

**Environment note:** This task is NOT runnable in the monorepo Mac checkout (no Python/
torch/GPU here). Run it where the MTR model lives (the team's training box or a GPU dev
node). It is a research/verification task, so its "test" is the recorded evidence doc, not
a gtest.

**Files:**
- Create: `docs/superpowers/specs/2026-06-24-mtr-io-verification.md`
- (On amendment) update the header code in Task 1 Step 2 / Step 3 before freezing.

**Interfaces:**
- Produces: a verified, signed-off field list for `MtrInputTensors`, `MtrOutputTensors`,
  `MtrTensorSpec`, and `MtrModelContract` — i.e. the exact binding names, dtypes, and
  shapes Task 1 will freeze.

- [ ] **Step 1: Obtain the model**

Clone the official repo and a checkpoint:
```bash
git clone https://github.com/sshaoshuai/MTR.git
# obtain a pretrained Waymo checkpoint per the repo README, set up its conda env
```

- [ ] **Step 2: Dump real input/output tensor specs**

Run one forward pass on a sample batch and record, for **every** key, the shape + dtype:
```python
# pseudo-runner — adapt to MTR's demo/eval entrypoint
batch = next(iter(dataloader))['input_dict']
for k in ['obj_trajs','obj_trajs_mask','obj_trajs_last_pos','track_index_to_predict',
          'center_objects_type','map_polylines','map_polylines_mask','map_polylines_center']:
    print(k, tuple(batch[k].shape), batch[k].dtype)
pred = model(batch)
for k in ['pred_scores','pred_trajs']:
    print(k, tuple(pred[k].shape), pred[k].dtype)
```
Record the **per-step feature dimension `C` of `obj_trajs`** and the **map polyline feature
dim** explicitly — these drive Person A's packing and are the most error-prone.

- [ ] **Step 3: If TensorRT is the deploy path, dump engine bindings**

Export to ONNX (this is non-trivial — MTR has no official export; note any custom ops /
dynamic-shape handling discovered) and list binding names/dtypes/shapes. These become the
expected `MtrModelContract` that `validateMtrModelContract` checks.

- [ ] **Step 4: Produce the delta table and sign-off doc**

Write `docs/superpowers/specs/2026-06-24-mtr-io-verification.md` containing: a table of each
tensor (name, real shape, real dtype) vs the proposed `mtr_types.hpp` field; a delta list of
anything that must change; and an explicit "frozen field list" section that Task 1 copies.
End with sign-off lines for A, B, C.

- [ ] **Step 5: Amend Task 1 headers if deltas exist**

If the verification found differences (e.g. `center_objects_type` is `int64` not `int32`, or
`obj_trajs` needs a separate feature-dim field), update the code blocks in Task 1 Step 2/3
accordingly **before** Task 1 is implemented. Otherwise note "no changes — proposed contract
verified as-is."

- [ ] **Step 6: Commit**

```bash
git add docs/superpowers/specs/2026-06-24-mtr-io-verification.md
git commit -m "docs(prediction_ml): MTR model I/O verification + contract freeze sign-off"
```

---

### Task 1: Package scaffold + frozen shared headers

**Files:**
- Create: `src/world_modeling/prediction_ml/package.xml`
- Create: `src/world_modeling/prediction_ml/CMakeLists.txt`
- Create: `src/world_modeling/prediction_ml/include/prediction_ml/mtr_types.hpp`
- Create: `src/world_modeling/prediction_ml/include/prediction_ml/mtr_inference_engine.hpp`
- Test: `src/world_modeling/prediction_ml/test/test_contract_compiles.cpp`

**Interfaces:**
- Produces (frozen): all structs/enums in `mtr_types.hpp` (`MtrMode`, `MtrConfig`, `MtrTensorSpec`, `MtrModelContract`, `MtrFrameContext`, `MtrTargetSidecar`, `MtrBatchSidecar`, `MtrInputTensors`, `MtrOutputTensors`, `MtrObjectPrediction`, `MtrInferenceResult`), free functions `parseMtrMode`, `loadMtrModelContract`, `validateMtrModelContract`; and the `IMtrInferenceEngine` interface + factories `createNullMtrInferenceEngine` / `createTensorRtMtrInferenceEngine` in `mtr_inference_engine.hpp`.

- [ ] **Step 1: Create `package.xml`**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>prediction_ml</name>
  <version>0.1.0</version>
  <description>ML (MTR) trajectory prediction with a constant-velocity fallback</description>
  <maintainer email="hello@watonomous.com">WATonomous</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_lifecycle</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>vision_msgs</depend>
  <depend>world_model_msgs</depend>
  <depend>lanelet_msgs</depend>

  <test_depend>ament_cmake_gtest</test_depend>
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 2: Create `include/prediction_ml/mtr_types.hpp`** (Apache header first, then:)

```cpp
#ifndef PREDICTION_ML__MTR_TYPES_HPP_
#define PREDICTION_ML__MTR_TYPES_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lanelet_msgs/msg/lanelet_ahead.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "world_model_msgs/msg/prediction.hpp"

namespace prediction_ml
{

// MTR execution mode. Disabled and Null both yield fallback-only output.
enum class MtrMode
{
  Disabled,
  Null,
  TensorRt
};

struct MtrConfig
{
  MtrMode mode{MtrMode::Disabled};
  std::string engine_path;
  std::string metadata_path;
  double cache_ttl_s{0.5};
  int selected_target_agent_limit{8};
  int history_steps{11};
  double history_rate_hz{10.0};
};

// One engine binding (name + dtype + shape) used for contract validation.
struct MtrTensorSpec
{
  std::string name;
  std::string dtype;
  std::vector<int64_t> shape;
};

struct MtrModelContract
{
  std::vector<MtrTensorSpec> inputs;
  std::vector<MtrTensorSpec> outputs;
};

// Per-tick bundle handed to the scene builder.
struct MtrFrameContext
{
  vision_msgs::msg::Detection3DArray detections;
  geometry_msgs::msg::PoseStamped ego_pose;
  lanelet_msgs::msg::LaneletAhead lanelet_ahead;
  double timestamp{0.0};
  bool has_ego{false};
  bool has_map{false};
};

// Maps a target slot back to its source detection and map-frame pose.
struct MtrTargetSidecar
{
  std::string detection_id;
  int track_index{0};
  double center_x{0.0};
  double center_y{0.0};
  double center_heading{0.0};
};

struct MtrBatchSidecar
{
  std::vector<MtrTargetSidecar> targets;
  std::string frame_id;
};

// Flat MTR input buffers (row-major) plus shapes and the sidecar.
struct MtrInputTensors
{
  std::vector<float> obj_trajs;
  std::vector<float> obj_trajs_mask;
  std::vector<float> obj_trajs_last_pos;
  std::vector<int32_t> track_index_to_predict;
  std::vector<int32_t> center_objects_type;
  std::vector<float> map_polylines;
  std::vector<float> map_polylines_mask;
  std::vector<float> map_polylines_center;
  std::vector<int64_t> obj_trajs_shape;
  std::vector<int64_t> map_polylines_shape;
  MtrBatchSidecar sidecar;
  bool valid{false};
};

// Raw MTR outputs: pred_scores [num_target, K], pred_trajs [num_target, K, T, C].
struct MtrOutputTensors
{
  std::vector<float> pred_scores;
  std::vector<float> pred_trajs;
  std::vector<int64_t> scores_shape;
  std::vector<int64_t> trajs_shape;
  bool valid{false};
};

// Converted per-object result (map frame, world_model_msgs/Prediction).
struct MtrObjectPrediction
{
  std::string detection_id;
  std::vector<world_model_msgs::msg::Prediction> predictions;
};

struct MtrInferenceResult
{
  std::vector<MtrObjectPrediction> objects;
  bool ok{false};
  std::string error;
};

// Parse "disabled" | "null" | "tensorrt" (case-insensitive) into MtrMode.
MtrMode parseMtrMode(const std::string & mode_str);

// Load expected/actual engine contract from a JSON/YAML metadata file.
MtrModelContract loadMtrModelContract(const std::string & metadata_path);

// Returns true if actual matches expected; fills error otherwise.
bool validateMtrModelContract(
  const MtrModelContract & expected, const MtrModelContract & actual, std::string & error);

}  // namespace prediction_ml

#endif  // PREDICTION_ML__MTR_TYPES_HPP_
```

- [ ] **Step 3: Create `include/prediction_ml/mtr_inference_engine.hpp`** (Apache header first, then:)

```cpp
#ifndef PREDICTION_ML__MTR_INFERENCE_ENGINE_HPP_
#define PREDICTION_ML__MTR_INFERENCE_ENGINE_HPP_

#include <memory>
#include <string>

#include "prediction_ml/mtr_types.hpp"

namespace prediction_ml
{

// Backend contract. Both null and TensorRT engines implement this.
class IMtrInferenceEngine
{
public:
  virtual ~IMtrInferenceEngine() = default;
  virtual bool ready() const = 0;
  virtual std::string lastError() const = 0;
  virtual MtrOutputTensors infer(const MtrInputTensors & input) = 0;
};

std::unique_ptr<IMtrInferenceEngine> createNullMtrInferenceEngine(const MtrConfig & config);
std::unique_ptr<IMtrInferenceEngine> createTensorRtMtrInferenceEngine(const MtrConfig & config);

}  // namespace prediction_ml

#endif  // PREDICTION_ML__MTR_INFERENCE_ENGINE_HPP_
```

- [ ] **Step 4: Create `test/test_contract_compiles.cpp`** (Apache header first, then:)

```cpp
#include <gtest/gtest.h>

#include "prediction_ml/mtr_inference_engine.hpp"
#include "prediction_ml/mtr_types.hpp"

TEST(MtrContract, ParseMode)
{
  EXPECT_EQ(prediction_ml::parseMtrMode("disabled"), prediction_ml::MtrMode::Disabled);
  EXPECT_EQ(prediction_ml::parseMtrMode("null"), prediction_ml::MtrMode::Null);
  EXPECT_EQ(prediction_ml::parseMtrMode("tensorrt"), prediction_ml::MtrMode::TensorRt);
}

TEST(MtrContract, DefaultsAreFallbackSafe)
{
  prediction_ml::MtrConfig cfg;
  EXPECT_EQ(cfg.mode, prediction_ml::MtrMode::Disabled);
  prediction_ml::MtrInputTensors in;
  EXPECT_FALSE(in.valid);
  prediction_ml::MtrOutputTensors out;
  EXPECT_FALSE(out.valid);
}
```

Note: `parseMtrMode` is defined in Task 2's `null_backend.cpp` (always built). This test links once Task 2 lands; for Task 1 it verifies the headers compile and the struct defaults are correct. If running Task 1 in isolation, expect a link error on `parseMtrMode` only — that is resolved by Task 2.

- [ ] **Step 5: Create `CMakeLists.txt`** with marked append slots:

```cmake
# Copyright (c) 2025-present WATonomous. All rights reserved.
# (Apache-2.0 header block — copy verbatim from prediction/CMakeLists.txt lines 1-13)
cmake_minimum_required(VERSION 3.10)
project(prediction_ml)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(PREDICTION_ML_ENABLE_TENSORRT "Build the TensorRT MTR backend" OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(world_model_msgs REQUIRED)
find_package(lanelet_msgs REQUIRED)

add_library(prediction_ml_lib
  # >>> PERSON_B sources
  src/null_backend.cpp
  # <<< PERSON_B sources
  # >>> PERSON_A sources
  src/scene_builder.cpp
  # <<< PERSON_A sources
  # >>> PERSON_C sources
  src/output_converter.cpp
  src/mtr_runtime.cpp
  # <<< PERSON_C sources
)

if(PREDICTION_ML_ENABLE_TENSORRT)
  target_sources(prediction_ml_lib PRIVATE src/tensorrt_backend.cpp)
  target_compile_definitions(prediction_ml_lib PRIVATE PREDICTION_ML_ENABLE_TENSORRT)
endif()

set_target_properties(prediction_ml_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(prediction_ml_lib PUBLIC include)
target_link_libraries(prediction_ml_lib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${vision_msgs_TARGETS}
  ${world_model_msgs_TARGETS}
  ${lanelet_msgs_TARGETS}
)

add_executable(prediction_ml_node src/prediction_ml_node.cpp)
target_link_libraries(prediction_ml_node prediction_ml_lib)

install(TARGETS prediction_ml_lib
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS prediction_ml_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_contract_compiles test/test_contract_compiles.cpp)
  target_link_libraries(test_contract_compiles prediction_ml_lib)
  # >>> PERSON_B tests
  # ament_add_gtest(test_backend test/test_backend.cpp)
  # target_link_libraries(test_backend prediction_ml_lib)
  # <<< PERSON_B tests
  # >>> PERSON_A tests
  # ament_add_gtest(test_scene_builder test/test_scene_builder.cpp)
  # target_link_libraries(test_scene_builder prediction_ml_lib)
  # <<< PERSON_A tests
  # >>> PERSON_C tests
  # ament_add_gtest(test_runtime test/test_runtime.cpp)
  # target_link_libraries(test_runtime prediction_ml_lib)
  # <<< PERSON_C tests
endif()

ament_package()
```

Note: this CMake references files created in Tasks 2-6. It will not build until those land. That is expected — Task 1's deliverable is the frozen contract + scaffold. To verify Task 1 in isolation, temporarily comment every `src/*.cpp` line and the `add_executable`, build, confirm headers compile, then restore. The first green full build happens at Task 6.

- [ ] **Step 6: Commit**

```bash
git add src/world_modeling/prediction_ml/package.xml \
        src/world_modeling/prediction_ml/CMakeLists.txt \
        src/world_modeling/prediction_ml/include/prediction_ml/mtr_types.hpp \
        src/world_modeling/prediction_ml/include/prediction_ml/mtr_inference_engine.hpp \
        src/world_modeling/prediction_ml/test/test_contract_compiles.cpp
git commit -m "feat(prediction_ml): scaffold package + frozen MTR shared contract headers"
```

---

### Task 2: Person B — inference backend (null + gated TensorRT stub)

**Files:**
- Create: `src/world_modeling/prediction_ml/include/prediction_ml/mtr_backend.hpp`
- Create: `src/world_modeling/prediction_ml/src/null_backend.cpp`
- Create: `src/world_modeling/prediction_ml/src/tensorrt_backend.cpp`
- Test: `src/world_modeling/prediction_ml/test/test_backend.cpp`
- Modify: `src/world_modeling/prediction_ml/CMakeLists.txt` (uncomment PERSON_B test block)

**Interfaces:**
- Consumes: `IMtrInferenceEngine`, `MtrConfig`, `MtrInputTensors`, `MtrOutputTensors`, `MtrModelContract` (Task 1).
- Produces: `createNullMtrInferenceEngine(const MtrConfig&) -> std::unique_ptr<IMtrInferenceEngine>` (always built), `createTensorRtMtrInferenceEngine(const MtrConfig&)` (gated stub returns null engine when TRT off), and definitions of `parseMtrMode`, `loadMtrModelContract`, `validateMtrModelContract`.

- [ ] **Step 1: Write the failing test** — `test/test_backend.cpp` (Apache header first, then:)

```cpp
#include <gtest/gtest.h>

#include "prediction_ml/mtr_inference_engine.hpp"

TEST(NullBackend, NotReadyAndInvalidOutput)
{
  prediction_ml::MtrConfig cfg;
  auto engine = prediction_ml::createNullMtrInferenceEngine(cfg);
  ASSERT_NE(engine, nullptr);
  EXPECT_FALSE(engine->ready());
  prediction_ml::MtrInputTensors in;
  auto out = engine->infer(in);
  EXPECT_FALSE(out.valid);
}

TEST(NullBackend, TensorRtFactoryFallsBackWhenDisabled)
{
  prediction_ml::MtrConfig cfg;
  auto engine = prediction_ml::createTensorRtMtrInferenceEngine(cfg);
  ASSERT_NE(engine, nullptr);
  EXPECT_FALSE(engine->ready());
}
```

- [ ] **Step 2: Create `include/prediction_ml/mtr_backend.hpp`** (Apache header first, then:)

```cpp
#ifndef PREDICTION_ML__MTR_BACKEND_HPP_
#define PREDICTION_ML__MTR_BACKEND_HPP_

#include <string>

#include "prediction_ml/mtr_inference_engine.hpp"
#include "prediction_ml/mtr_types.hpp"

namespace prediction_ml
{

// Always-available engine that is never ready; forces fallback-only output.
class NullMtrInferenceEngine : public IMtrInferenceEngine
{
public:
  explicit NullMtrInferenceEngine(MtrConfig config);
  bool ready() const override;
  std::string lastError() const override;
  MtrOutputTensors infer(const MtrInputTensors & input) override;

private:
  MtrConfig config_;
  std::string last_error_{"mtr disabled (null backend)"};
};

}  // namespace prediction_ml

#endif  // PREDICTION_ML__MTR_BACKEND_HPP_
```

- [ ] **Step 3: Create `src/null_backend.cpp`** (Apache header first, then:)

```cpp
#include "prediction_ml/mtr_backend.hpp"

#include <algorithm>
#include <cctype>
#include <memory>
#include <string>
#include <utility>

namespace prediction_ml
{

NullMtrInferenceEngine::NullMtrInferenceEngine(MtrConfig config) : config_(std::move(config)) {}

bool NullMtrInferenceEngine::ready() const { return false; }

std::string NullMtrInferenceEngine::lastError() const { return last_error_; }

MtrOutputTensors NullMtrInferenceEngine::infer(const MtrInputTensors & /*input*/)
{
  MtrOutputTensors out;
  out.valid = false;
  return out;
}

std::unique_ptr<IMtrInferenceEngine> createNullMtrInferenceEngine(const MtrConfig & config)
{
  return std::make_unique<NullMtrInferenceEngine>(config);
}

#ifndef PREDICTION_ML_ENABLE_TENSORRT
std::unique_ptr<IMtrInferenceEngine> createTensorRtMtrInferenceEngine(const MtrConfig & config)
{
  // TensorRT not compiled in: behave exactly like the null backend.
  return std::make_unique<NullMtrInferenceEngine>(config);
}
#endif

MtrMode parseMtrMode(const std::string & mode_str)
{
  std::string s = mode_str;
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
  if (s == "null") {
    return MtrMode::Null;
  }
  if (s == "tensorrt" || s == "trt") {
    return MtrMode::TensorRt;
  }
  return MtrMode::Disabled;
}

MtrModelContract loadMtrModelContract(const std::string & /*metadata_path*/)
{
  // TODO(Person B): parse metadata file into expected contract.
  return MtrModelContract{};
}

bool validateMtrModelContract(
  const MtrModelContract & /*expected*/, const MtrModelContract & /*actual*/, std::string & error)
{
  // TODO(Person B): compare binding names/dtypes/shapes.
  error.clear();
  return true;
}

}  // namespace prediction_ml
```

- [ ] **Step 4: Create `src/tensorrt_backend.cpp`** (gated; stub for this milestone) (Apache header first, then:)

```cpp
#include <memory>

#include "prediction_ml/mtr_backend.hpp"

// This translation unit is compiled ONLY when PREDICTION_ML_ENABLE_TENSORRT is set.
namespace prediction_ml
{

std::unique_ptr<IMtrInferenceEngine> createTensorRtMtrInferenceEngine(const MtrConfig & config)
{
  // TODO(Person B): load engine_path, validate bindings against MtrModelContract,
  // implement infer(). For the skeleton, fall back to the null engine so a TRT-enabled
  // build still links and runs.
  return createNullMtrInferenceEngine(config);
}

}  // namespace prediction_ml
```

- [ ] **Step 5: Uncomment the PERSON_B test block** in `CMakeLists.txt`:

```cmake
  # >>> PERSON_B tests
  ament_add_gtest(test_backend test/test_backend.cpp)
  target_link_libraries(test_backend prediction_ml_lib)
  # <<< PERSON_B tests
```

- [ ] **Step 6: Build and run the test**

Run: `colcon build --packages-select prediction_ml && colcon test --packages-select prediction_ml --ctest-args -R test_backend`
Expected: PASS (both `NullBackend` tests; `test_contract_compiles` also links now).

- [ ] **Step 7: Commit**

```bash
git add src/world_modeling/prediction_ml/include/prediction_ml/mtr_backend.hpp \
        src/world_modeling/prediction_ml/src/null_backend.cpp \
        src/world_modeling/prediction_ml/src/tensorrt_backend.cpp \
        src/world_modeling/prediction_ml/test/test_backend.cpp \
        src/world_modeling/prediction_ml/CMakeLists.txt
git commit -m "feat(prediction_ml): null inference backend + gated TensorRT stub (Person B)"
```

---

### Task 3: Person A — scene builder stub (Detection3DArray → MTR tensors)

**Files:**
- Create: `src/world_modeling/prediction_ml/include/prediction_ml/scene_builder.hpp`
- Create: `src/world_modeling/prediction_ml/src/scene_builder.cpp`
- Test: `src/world_modeling/prediction_ml/test/test_scene_builder.cpp`
- Modify: `src/world_modeling/prediction_ml/CMakeLists.txt` (uncomment PERSON_A test block)

**Interfaces:**
- Consumes: `MtrFrameContext`, `MtrInputTensors`, `MtrConfig` (Task 1).
- Produces: class `SceneBuilder` with `explicit SceneBuilder(MtrConfig)`, `void addFrame(const vision_msgs::msg::Detection3DArray &)`, and `MtrInputTensors build(const MtrFrameContext & frame)`. For the skeleton, `build` returns `MtrInputTensors{}` with `valid=false`.

- [ ] **Step 1: Write the failing test** — `test/test_scene_builder.cpp` (Apache header first, then:)

```cpp
#include <gtest/gtest.h>

#include "prediction_ml/scene_builder.hpp"

TEST(SceneBuilder, EmptyFrameProducesInvalidTensors)
{
  prediction_ml::MtrConfig cfg;
  prediction_ml::SceneBuilder builder(cfg);
  prediction_ml::MtrFrameContext frame;
  auto tensors = builder.build(frame);
  EXPECT_FALSE(tensors.valid);
  EXPECT_TRUE(tensors.sidecar.targets.empty());
}
```

- [ ] **Step 2: Create `include/prediction_ml/scene_builder.hpp`** (Apache header first, then:)

```cpp
#ifndef PREDICTION_ML__SCENE_BUILDER_HPP_
#define PREDICTION_ML__SCENE_BUILDER_HPP_

#include "prediction_ml/mtr_types.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

namespace prediction_ml
{

// Maintains per-object history and packs MTR input tensors. (Person A)
class SceneBuilder
{
public:
  explicit SceneBuilder(MtrConfig config);

  // Append the latest tracked detections into per-object history.
  void addFrame(const vision_msgs::msg::Detection3DArray & detections);

  // Pack the current scene into MTR input tensors + sidecar.
  MtrInputTensors build(const MtrFrameContext & frame);

private:
  MtrConfig config_;
};

}  // namespace prediction_ml

#endif  // PREDICTION_ML__SCENE_BUILDER_HPP_
```

- [ ] **Step 3: Create `src/scene_builder.cpp`** (Apache header first, then:)

```cpp
#include "prediction_ml/scene_builder.hpp"

#include <utility>

namespace prediction_ml
{

SceneBuilder::SceneBuilder(MtrConfig config) : config_(std::move(config)) {}

void SceneBuilder::addFrame(const vision_msgs::msg::Detection3DArray & /*detections*/)
{
  // TODO(Person A): update per-object history keyed by detection id
  // (pose, dims, heading, velocity, type, timestamp, validity).
}

MtrInputTensors SceneBuilder::build(const MtrFrameContext & /*frame*/)
{
  // TODO(Person A): resample history, select target agents up to
  // config_.selected_target_agent_limit, pack obj_trajs* / track_index_to_predict /
  // center_objects_type, convert lanelet context to map_polylines*, fill sidecar.
  MtrInputTensors tensors;
  tensors.valid = false;
  return tensors;
}

}  // namespace prediction_ml
```

- [ ] **Step 4: Uncomment the PERSON_A test block** in `CMakeLists.txt`:

```cmake
  # >>> PERSON_A tests
  ament_add_gtest(test_scene_builder test/test_scene_builder.cpp)
  target_link_libraries(test_scene_builder prediction_ml_lib)
  # <<< PERSON_A tests
```

- [ ] **Step 5: Build and run the test**

Run: `colcon build --packages-select prediction_ml && colcon test --packages-select prediction_ml --ctest-args -R test_scene_builder`
Expected: PASS (`SceneBuilder.EmptyFrameProducesInvalidTensors`).

- [ ] **Step 6: Commit**

```bash
git add src/world_modeling/prediction_ml/include/prediction_ml/scene_builder.hpp \
        src/world_modeling/prediction_ml/src/scene_builder.cpp \
        src/world_modeling/prediction_ml/test/test_scene_builder.cpp \
        src/world_modeling/prediction_ml/CMakeLists.txt
git commit -m "feat(prediction_ml): scene builder stub (Person A)"
```

---

### Task 4: Person C — output converter stub (MTR tensors → WorldObject predictions)

**Files:**
- Create: `src/world_modeling/prediction_ml/include/prediction_ml/output_converter.hpp`
- Create: `src/world_modeling/prediction_ml/src/output_converter.cpp`

**Interfaces:**
- Consumes: `MtrOutputTensors`, `MtrBatchSidecar`, `MtrInferenceResult`, `MtrObjectPrediction` (Task 1).
- Produces: free function `MtrInferenceResult convertMtrOutput(const MtrOutputTensors & out, const MtrBatchSidecar & sidecar, const std::string & frame_id, double horizon_s, double time_step_s)`. For the skeleton, returns `MtrInferenceResult{}` with `ok=false` when `out.valid` is false.

- [ ] **Step 1: Create `include/prediction_ml/output_converter.hpp`** (Apache header first, then:)

```cpp
#ifndef PREDICTION_ML__OUTPUT_CONVERTER_HPP_
#define PREDICTION_ML__OUTPUT_CONVERTER_HPP_

#include <string>

#include "prediction_ml/mtr_types.hpp"

namespace prediction_ml
{

// Convert raw MTR outputs (target frame) into map-frame world_model predictions.
MtrInferenceResult convertMtrOutput(
  const MtrOutputTensors & out, const MtrBatchSidecar & sidecar, const std::string & frame_id,
  double horizon_s, double time_step_s);

}  // namespace prediction_ml

#endif  // PREDICTION_ML__OUTPUT_CONVERTER_HPP_
```

- [ ] **Step 2: Create `src/output_converter.cpp`** (Apache header first, then:)

```cpp
#include "prediction_ml/output_converter.hpp"

namespace prediction_ml
{

MtrInferenceResult convertMtrOutput(
  const MtrOutputTensors & out, const MtrBatchSidecar & /*sidecar*/, const std::string & /*frame_id*/,
  double /*horizon_s*/, double /*time_step_s*/)
{
  MtrInferenceResult result;
  if (!out.valid) {
    result.ok = false;
    result.error = "mtr output invalid";
    return result;
  }
  // TODO(Person C): validate pred_scores/pred_trajs, rotate/translate each mode from
  // target frame to map frame using sidecar, infer yaw from consecutive points, and
  // emit world_model_msgs/Prediction with conf = pred_score.
  result.ok = true;
  return result;
}

}  // namespace prediction_ml
```

- [ ] **Step 3: Build to confirm it compiles**

Run: `colcon build --packages-select prediction_ml`
Expected: SUCCESS (no new test yet; converter is exercised via Task 5's runtime test).

- [ ] **Step 4: Commit**

```bash
git add src/world_modeling/prediction_ml/include/prediction_ml/output_converter.hpp \
        src/world_modeling/prediction_ml/src/output_converter.cpp
git commit -m "feat(prediction_ml): output converter stub (Person C)"
```

---

### Task 5: Person C — async runtime stub (submitFrame / selectOutput / cache)

**Files:**
- Create: `src/world_modeling/prediction_ml/include/prediction_ml/mtr_runtime.hpp`
- Create: `src/world_modeling/prediction_ml/src/mtr_runtime.cpp`
- Test: `src/world_modeling/prediction_ml/test/test_runtime.cpp`
- Modify: `src/world_modeling/prediction_ml/CMakeLists.txt` (uncomment PERSON_C test block)

**Interfaces:**
- Consumes: `IMtrInferenceEngine` + factories (Task 1/2), `convertMtrOutput` (Task 4), `MtrInputTensors`, `MtrConfig` (Task 1).
- Produces: class `MtrRuntime` with `explicit MtrRuntime(MtrConfig)`, `void submitFrame(const MtrInputTensors &, const std::string & frame_id, double horizon_s, double time_step_s)`, `std::vector<world_model_msgs::msg::WorldObject> selectOutput(const std::vector<world_model_msgs::msg::WorldObject> & fallback, double now_s)`, `bool ready() const`, `std::string lastError() const`. For the skeleton, `selectOutput` returns the fallback unchanged.

- [ ] **Step 1: Write the failing test** — `test/test_runtime.cpp` (Apache header first, then:)

```cpp
#include <gtest/gtest.h>

#include <vector>

#include "prediction_ml/mtr_runtime.hpp"

TEST(MtrRuntime, DisabledReturnsFallbackUnchanged)
{
  prediction_ml::MtrConfig cfg;  // Disabled
  prediction_ml::MtrRuntime runtime(cfg);

  std::vector<world_model_msgs::msg::WorldObject> fallback(3);
  fallback[0].detection.id = "a";
  fallback[1].detection.id = "b";
  fallback[2].detection.id = "c";

  auto out = runtime.selectOutput(fallback, 0.0);
  ASSERT_EQ(out.size(), 3u);
  EXPECT_EQ(out[0].detection.id, "a");
  EXPECT_FALSE(runtime.ready());
}
```

- [ ] **Step 2: Create `include/prediction_ml/mtr_runtime.hpp`** (Apache header first, then:)

```cpp
#ifndef PREDICTION_ML__MTR_RUNTIME_HPP_
#define PREDICTION_ML__MTR_RUNTIME_HPP_

#include <memory>
#include <string>
#include <vector>

#include "prediction_ml/mtr_inference_engine.hpp"
#include "prediction_ml/mtr_types.hpp"
#include "world_model_msgs/msg/world_object.hpp"

namespace prediction_ml
{

// Owns the inference engine, runs it async (latest-only), caches per-object
// predictions with a TTL, and merges them onto the fallback. (Person C)
class MtrRuntime
{
public:
  explicit MtrRuntime(MtrConfig config);
  ~MtrRuntime();

  // Hand the latest packed tensors to the (future) async worker.
  void submitFrame(
    const MtrInputTensors & input, const std::string & frame_id, double horizon_s,
    double time_step_s);

  // Start from fallback; replace only objects with fresh valid cached MTR predictions.
  std::vector<world_model_msgs::msg::WorldObject> selectOutput(
    const std::vector<world_model_msgs::msg::WorldObject> & fallback, double now_s);

  bool ready() const;
  std::string lastError() const;

private:
  MtrConfig config_;
  std::unique_ptr<IMtrInferenceEngine> engine_;
};

}  // namespace prediction_ml

#endif  // PREDICTION_ML__MTR_RUNTIME_HPP_
```

- [ ] **Step 3: Create `src/mtr_runtime.cpp`** (Apache header first, then:)

```cpp
#include "prediction_ml/mtr_runtime.hpp"

#include <utility>

namespace prediction_ml
{

MtrRuntime::MtrRuntime(MtrConfig config) : config_(std::move(config))
{
  if (config_.mode == MtrMode::TensorRt) {
    engine_ = createTensorRtMtrInferenceEngine(config_);
  } else {
    engine_ = createNullMtrInferenceEngine(config_);
  }
}

MtrRuntime::~MtrRuntime() = default;

void MtrRuntime::submitFrame(
  const MtrInputTensors & /*input*/, const std::string & /*frame_id*/, double /*horizon_s*/,
  double /*time_step_s*/)
{
  // TODO(Person C): hand latest-only frame to a background worker; on completion,
  // run convertMtrOutput and store results in a per-object TTL cache.
}

std::vector<world_model_msgs::msg::WorldObject> MtrRuntime::selectOutput(
  const std::vector<world_model_msgs::msg::WorldObject> & fallback, double /*now_s*/)
{
  // TODO(Person C): replace fallback entries whose detection id has a fresh
  // (age <= config_.cache_ttl_s) valid cached MTR prediction.
  return fallback;
}

bool MtrRuntime::ready() const { return engine_ && engine_->ready(); }

std::string MtrRuntime::lastError() const
{
  return engine_ ? engine_->lastError() : "no engine";
}

}  // namespace prediction_ml
```

- [ ] **Step 4: Uncomment the PERSON_C test block** in `CMakeLists.txt`:

```cmake
  # >>> PERSON_C tests
  ament_add_gtest(test_runtime test/test_runtime.cpp)
  target_link_libraries(test_runtime prediction_ml_lib)
  # <<< PERSON_C tests
```

- [ ] **Step 5: Build and run the test**

Run: `colcon build --packages-select prediction_ml && colcon test --packages-select prediction_ml --ctest-args -R test_runtime`
Expected: PASS (`MtrRuntime.DisabledReturnsFallbackUnchanged`).

- [ ] **Step 6: Commit**

```bash
git add src/world_modeling/prediction_ml/include/prediction_ml/mtr_runtime.hpp \
        src/world_modeling/prediction_ml/src/mtr_runtime.cpp \
        src/world_modeling/prediction_ml/test/test_runtime.cpp \
        src/world_modeling/prediction_ml/CMakeLists.txt
git commit -m "feat(prediction_ml): async runtime stub with fallback passthrough (Person C)"
```

---

### Task 6: Person C — node + CV fallback + config/launch (integration, green build)

**Files:**
- Create: `src/world_modeling/prediction_ml/include/prediction_ml/prediction_ml_node.hpp`
- Create: `src/world_modeling/prediction_ml/src/prediction_ml_node.cpp`
- Create: `src/world_modeling/prediction_ml/config/params.yaml`
- Create: `src/world_modeling/prediction_ml/launch/prediction_ml.launch.yaml`
- Create: `src/world_modeling/prediction_ml/README.md`

**Interfaces:**
- Consumes: `SceneBuilder` (Task 3), `MtrRuntime` (Task 5), `MtrFrameContext`, `MtrConfig`, `parseMtrMode` (Task 1/2).
- Produces: the runnable `prediction_ml_node` executable. Subscribes `tracks_3d`, `ego_pose`, `lanelet_ahead`; publishes `world_object_seeds`. Each tick: build CV fallback, `scene_builder_.addFrame` + `runtime_.submitFrame`, publish `runtime_.selectOutput(fallback, now)`.

- [ ] **Step 1: Create `include/prediction_ml/prediction_ml_node.hpp`** (Apache header first, then:)

```cpp
#ifndef PREDICTION_ML__PREDICTION_ML_NODE_HPP_
#define PREDICTION_ML__PREDICTION_ML_NODE_HPP_

#include <memory>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lanelet_msgs/msg/lanelet_ahead.hpp"
#include "prediction_ml/mtr_runtime.hpp"
#include "prediction_ml/scene_builder.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "world_model_msgs/msg/world_object.hpp"
#include "world_model_msgs/msg/world_object_array.hpp"

namespace prediction_ml
{

class PredictionMlNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit PredictionMlNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PredictionMlNode() override = default;

protected:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void trackedObjectsCallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg);
  void egoPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void laneletAheadCallback(const lanelet_msgs::msg::LaneletAhead::SharedPtr msg);

  // Constant-velocity straight-line fallback (ported from simple_prediction).
  std::vector<world_model_msgs::msg::WorldObject> buildFallback(
    const vision_msgs::msg::Detection3DArray & msg) const;

  MtrConfig loadMtrConfig();

  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr tracked_objects_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ego_pose_sub_;
  rclcpp::Subscription<lanelet_msgs::msg::LaneletAhead>::SharedPtr lanelet_ahead_sub_;
  rclcpp_lifecycle::LifecyclePublisher<world_model_msgs::msg::WorldObjectArray>::SharedPtr
    world_objects_pub_;

  std::unique_ptr<SceneBuilder> scene_builder_;
  std::unique_ptr<MtrRuntime> runtime_;

  geometry_msgs::msg::PoseStamped::SharedPtr ego_pose_;
  lanelet_msgs::msg::LaneletAhead::SharedPtr lanelet_ahead_;

  double prediction_horizon_{3.0};
  double prediction_time_step_{0.2};
};

}  // namespace prediction_ml

#endif  // PREDICTION_ML__PREDICTION_ML_NODE_HPP_
```

- [ ] **Step 2: Create `src/prediction_ml_node.cpp`** (Apache header first, then:)

```cpp
#include "prediction_ml/prediction_ml_node.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace prediction_ml
{

PredictionMlNode::PredictionMlNode(const rclcpp::NodeOptions & options)
: LifecycleNode("prediction_ml_node", options)
{
  this->declare_parameter("prediction_horizon", 3.0);
  this->declare_parameter("prediction_time_step", 0.2);
  this->declare_parameter("mtr.mode", "disabled");
  this->declare_parameter("mtr.engine_path", "");
  this->declare_parameter("mtr.metadata_path", "");
  this->declare_parameter("mtr.cache_ttl_s", 0.5);
  this->declare_parameter("mtr.selected_target_agent_limit", 8);
  this->declare_parameter("mtr.history_steps", 11);
  this->declare_parameter("mtr.history_rate_hz", 10.0);
  RCLCPP_INFO(this->get_logger(), "PredictionMlNode created (unconfigured)");
}

MtrConfig PredictionMlNode::loadMtrConfig()
{
  MtrConfig cfg;
  cfg.mode = parseMtrMode(this->get_parameter("mtr.mode").as_string());
  cfg.engine_path = this->get_parameter("mtr.engine_path").as_string();
  cfg.metadata_path = this->get_parameter("mtr.metadata_path").as_string();
  cfg.cache_ttl_s = this->get_parameter("mtr.cache_ttl_s").as_double();
  cfg.selected_target_agent_limit =
    static_cast<int>(this->get_parameter("mtr.selected_target_agent_limit").as_int());
  cfg.history_steps = static_cast<int>(this->get_parameter("mtr.history_steps").as_int());
  cfg.history_rate_hz = this->get_parameter("mtr.history_rate_hz").as_double();
  return cfg;
}

PredictionMlNode::CallbackReturn PredictionMlNode::on_configure(const rclcpp_lifecycle::State &)
{
  prediction_horizon_ = this->get_parameter("prediction_horizon").as_double();
  prediction_time_step_ = this->get_parameter("prediction_time_step").as_double();

  const MtrConfig cfg = loadMtrConfig();
  scene_builder_ = std::make_unique<SceneBuilder>(cfg);
  runtime_ = std::make_unique<MtrRuntime>(cfg);

  world_objects_pub_ =
    this->create_publisher<world_model_msgs::msg::WorldObjectArray>("world_object_seeds", 10);
  RCLCPP_INFO(this->get_logger(), "Configured (horizon=%.1fs, step=%.2fs)", prediction_horizon_,
    prediction_time_step_);
  return CallbackReturn::SUCCESS;
}

PredictionMlNode::CallbackReturn PredictionMlNode::on_activate(const rclcpp_lifecycle::State &)
{
  tracked_objects_sub_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
    "tracks_3d", 10,
    std::bind(&PredictionMlNode::trackedObjectsCallback, this, std::placeholders::_1));
  ego_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "ego_pose", 10, std::bind(&PredictionMlNode::egoPoseCallback, this, std::placeholders::_1));
  lanelet_ahead_sub_ = this->create_subscription<lanelet_msgs::msg::LaneletAhead>(
    "lanelet_ahead", 10,
    std::bind(&PredictionMlNode::laneletAheadCallback, this, std::placeholders::_1));
  world_objects_pub_->on_activate();
  RCLCPP_INFO(this->get_logger(), "Activated");
  return CallbackReturn::SUCCESS;
}

PredictionMlNode::CallbackReturn PredictionMlNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  tracked_objects_sub_.reset();
  ego_pose_sub_.reset();
  lanelet_ahead_sub_.reset();
  world_objects_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

PredictionMlNode::CallbackReturn PredictionMlNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  tracked_objects_sub_.reset();
  ego_pose_sub_.reset();
  lanelet_ahead_sub_.reset();
  world_objects_pub_.reset();
  scene_builder_.reset();
  runtime_.reset();
  ego_pose_.reset();
  lanelet_ahead_.reset();
  return CallbackReturn::SUCCESS;
}

PredictionMlNode::CallbackReturn PredictionMlNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

void PredictionMlNode::egoPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  ego_pose_ = msg;
}

void PredictionMlNode::laneletAheadCallback(const lanelet_msgs::msg::LaneletAhead::SharedPtr msg)
{
  lanelet_ahead_ = msg;
}

std::vector<world_model_msgs::msg::WorldObject> PredictionMlNode::buildFallback(
  const vision_msgs::msg::Detection3DArray & msg) const
{
  std::vector<world_model_msgs::msg::WorldObject> objects;
  const std::string & frame_id = msg.header.frame_id;
  for (const auto & detection : msg.detections) {
    world_model_msgs::msg::WorldObject obj;
    obj.detection = detection;

    const double x = detection.bbox.center.position.x;
    const double y = detection.bbox.center.position.y;
    const double z = detection.bbox.center.position.z;
    const auto & q = detection.bbox.center.orientation;
    const double yaw =
      std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double speed = (detection.bbox.size.x > 3.5) ? 5.0 : 1.4;

    world_model_msgs::msg::Prediction pred;
    pred.header.frame_id = frame_id;
    pred.conf = 1.0;
    for (double t = prediction_time_step_; t <= prediction_horizon_; t += prediction_time_step_) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = frame_id;
      ps.pose.position.x = x + speed * std::cos(yaw) * t;
      ps.pose.position.y = y + speed * std::sin(yaw) * t;
      ps.pose.position.z = z;
      ps.pose.orientation = detection.bbox.center.orientation;
      pred.poses.push_back(ps);
    }
    obj.predictions.push_back(pred);
    objects.push_back(obj);
  }
  return objects;
}

void PredictionMlNode::trackedObjectsCallback(
  const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
  const double now_s = this->get_clock()->now().seconds();

  auto fallback = buildFallback(*msg);

  scene_builder_->addFrame(*msg);
  MtrFrameContext frame;
  frame.detections = *msg;
  if (ego_pose_) {
    frame.ego_pose = *ego_pose_;
    frame.has_ego = true;
  }
  if (lanelet_ahead_) {
    frame.lanelet_ahead = *lanelet_ahead_;
    frame.has_map = true;
  }
  frame.timestamp = now_s;
  MtrInputTensors tensors = scene_builder_->build(frame);
  runtime_->submitFrame(tensors, msg->header.frame_id, prediction_horizon_, prediction_time_step_);

  world_model_msgs::msg::WorldObjectArray output;
  output.header = msg->header;
  output.objects = runtime_->selectOutput(fallback, now_s);
  world_objects_pub_->publish(output);
}

}  // namespace prediction_ml

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<prediction_ml::PredictionMlNode>(rclcpp::NodeOptions());
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 3: Create `config/params.yaml`**

```yaml
---
# prediction_ml node parameters. Person C owns this header block; Person A and
# Person B append their own clearly-delimited sub-blocks below.
prediction_ml_node:
  ros__parameters:
    prediction_horizon: 3.0      # seconds — fallback + MTR horizon
    prediction_time_step: 0.2    # seconds — waypoint spacing

    # --- MTR group (Person B owns engine_path/metadata_path semantics) ---
    mtr.mode: "disabled"         # disabled | null | tensorrt
    mtr.engine_path: ""          # injected at launch for deploy
    mtr.metadata_path: ""        # injected at launch for deploy
    mtr.cache_ttl_s: 0.5
    mtr.selected_target_agent_limit: 8

    # --- Scene builder (Person A) ---
    mtr.history_steps: 11
    mtr.history_rate_hz: 10.0
```

- [ ] **Step 4: Create `launch/prediction_ml.launch.yaml`**

```yaml
launch:
  - arg:
      name: param_file
      default: $(find-pkg-share prediction_ml)/config/params.yaml
      description: Path to config file for prediction_ml node
  - arg:
      name: engine_path
      default: ""
      description: TensorRT engine path (deploy override)
  - arg:
      name: metadata_path
      default: ""
      description: MTR model metadata path (deploy override)

  - node:
      pkg: prediction_ml
      exec: prediction_ml_node
      name: prediction_ml_node
      namespace: "world_modeling"
      output: screen
      param:
        - from: $(var param_file)
        - name: mtr.engine_path
          value: $(var engine_path)
        - name: mtr.metadata_path
          value: $(var metadata_path)
      remap:
        - from: tracks_3d
          to: /perception/tracked_detections_3d
        - from: ego_pose
          to: /localization/pose
        - from: lanelet_ahead
          to: lanelet/lanelet_ahead
        - from: world_object_seeds
          to: /world_modeling/world_object_seeds
```

- [ ] **Step 5: Create `README.md`** documenting the package, the I/O contract, the 3-owner file map, and the `PREDICTION_ML_ENABLE_TENSORRT` flag (summarize the design spec at `docs/superpowers/specs/2026-06-24-prediction-ml-mtr-skeleton-design.md`).

- [ ] **Step 6: Full build + all tests**

Run: `colcon build --packages-select prediction_ml && colcon test --packages-select prediction_ml && colcon test-result --verbose`
Expected: build SUCCESS; `test_contract_compiles`, `test_backend`, `test_scene_builder`, `test_runtime` all PASS.

- [ ] **Step 7: Runtime smoke check**

Run (inside dev container, sourced):
```bash
ros2 launch prediction_ml prediction_ml.launch.yaml &
ros2 lifecycle set /world_modeling/prediction_ml_node configure
ros2 lifecycle set /world_modeling/prediction_ml_node activate
ros2 topic hz /world_modeling/world_object_seeds   # publishes when detections arrive
```
Expected: node activates; with a `tracks_3d` source, `world_object_seeds` publishes CV fallback predictions (MTR disabled).

- [ ] **Step 8: Commit**

```bash
git add src/world_modeling/prediction_ml/include/prediction_ml/prediction_ml_node.hpp \
        src/world_modeling/prediction_ml/src/prediction_ml_node.cpp \
        src/world_modeling/prediction_ml/config/params.yaml \
        src/world_modeling/prediction_ml/launch/prediction_ml.launch.yaml \
        src/world_modeling/prediction_ml/README.md
git commit -m "feat(prediction_ml): lifecycle node + CV fallback + config/launch (Person C)"
```

---

## After the skeleton

Each owner then fills their TODOs against the frozen contract on their own branch
(`ryan/prediction-ml-A-scene`, `-B-backend`, `-C-runtime`), touching only their files plus
their marked CMake/params slots. The first integration point is when Person B ships a real
`tensorrt_backend.cpp` behind `PREDICTION_ML_ENABLE_TENSORRT=ON`; until then `main` always
builds and runs CPU-only as the CV fallback.

## Self-Review

- **Spec coverage:** contract verified against real model before freeze ✓ Task 0 (gate); package home (new `prediction_ml`) ✓ Task 1; shared frozen headers ✓ Task 1; null backend + gated TRT + contract validation ✓ Task 2; scene builder ✓ Task 3; output converter ✓ Task 4; async runtime + selectOutput merge ✓ Task 5; node + CV fallback + config/launch + CPU-only green build ✓ Task 6; per-owner tests ✓ Tasks 2/3/5; CMake link-safety (null TU owns gated factory) ✓ Task 2 Step 3; acceptance criteria 1-4 ✓ Task 6 Steps 6-7.
- **Placeholder scan:** all code blocks are complete compiling stubs; `TODO(Person X)` markers are intentional implementation handoffs inside otherwise-complete functions, not plan gaps.
- **Type consistency:** `MtrConfig.history_rate_hz` used consistently (types + params + node); factory signatures `createNullMtrInferenceEngine(const MtrConfig&)` / `createTensorRtMtrInferenceEngine(const MtrConfig&)` match across header, null TU, gated TU, and runtime; `selectOutput(fallback, now_s)` signature matches between `mtr_runtime.hpp`, its test, and the node call site; `convertMtrOutput(...)` signature matches header and definition.

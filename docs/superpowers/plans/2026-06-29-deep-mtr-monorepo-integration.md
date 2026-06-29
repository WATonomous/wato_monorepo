# Deep MTR Monorepo Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace `prediction_ml`'s custom inference skeleton with a neutral ROS bridge to the merged `deep_ros/deep_mtr` skeleton while preserving fallback-only behavior.

**Architecture:** `prediction_ml` converts WATO detections, ego pose, and lanelets into `deep_msgs/MtrScene`; it validates and caches any future `MtrPredictionArray` results, but always computes fallback synchronously. The world-modeling image includes an immutable `deep_ros` revision and GPU runtime support, while both the bridge and `deep_mtr` remain disabled in live bringup by default.

**Tech Stack:** ROS 2 Jazzy, C++17, `ament_cmake`, `deep_msgs`, `lanelet_msgs`, `world_model_msgs`, gtest, Docker Compose, CUDA 12.8, ONNX Runtime TensorRT execution provider.

---

Prerequisite: complete and merge the `deep_ros` plan first. Execute this plan in a clean
`wato_monorepo` worktree created from the updated prediction branch. Preserve unrelated user files
and changes. Replace the current working-tree review edits only where this plan explicitly names
the same `prediction_ml` files.

### Task 1: Add semantic scene adaptation

**Files:**
- Create: `src/world_modeling/prediction_ml/include/prediction_ml/mtr_scene_adapter.hpp`
- Create: `src/world_modeling/prediction_ml/src/mtr_scene_adapter.cpp`
- Create: `src/world_modeling/prediction_ml/test/test_mtr_scene_adapter.cpp`
- Modify: `src/world_modeling/prediction_ml/CMakeLists.txt`
- Modify: `src/world_modeling/prediction_ml/package.xml`

- [ ] **Step 1: Write failing tests for WATO-to-neutral conversion**

Create `test/test_mtr_scene_adapter.cpp`:

```cpp
#include <gtest/gtest.h>

#include "prediction_ml/mtr_scene_adapter.hpp"

namespace prediction_ml
{
TEST(MtrSceneAdapter, ConvertsLaneletGeometryWithoutPackingModelTensors)
{
  MtrSceneAdapter adapter;
  vision_msgs::msg::Detection3DArray detections;
  detections.header.frame_id = "map";
  detections.header.stamp.sec = 10;
  detections.detections.resize(1);
  detections.detections[0].id = "track-1";

  lanelet_msgs::msg::LaneletAhead ahead;
  ahead.lanelets.resize(1);
  ahead.lanelets[0].id = 7;
  ahead.lanelets[0].centerline.resize(2);
  ahead.lanelets[0].left_boundary.resize(2);
  ahead.lanelets[0].right_boundary.resize(2);

  const auto scene = adapter.build(detections, nullptr, &ahead);

  EXPECT_EQ(scene.header, detections.header);
  EXPECT_EQ(scene.detections.detections[0].id, "track-1");
  EXPECT_TRUE(scene.has_map);
  ASSERT_EQ(scene.map_polylines.size(), 3u);
  EXPECT_EQ(scene.map_polylines[0].semantic_type, deep_msgs::msg::MapPolyline::CENTERLINE);
  EXPECT_EQ(scene.map_polylines[1].semantic_type, deep_msgs::msg::MapPolyline::LEFT_BOUNDARY);
  EXPECT_EQ(scene.map_polylines[2].semantic_type, deep_msgs::msg::MapPolyline::RIGHT_BOUNDARY);
}

TEST(MtrSceneAdapter, GeneratesUniqueCorrelatedRequests)
{
  MtrSceneAdapter adapter;
  vision_msgs::msg::Detection3DArray detections;
  detections.header.stamp.sec = 4;
  const auto first = adapter.build(detections, nullptr, nullptr);
  const auto second = adapter.build(detections, nullptr, nullptr);
  EXPECT_NE(first.request_id, second.request_id);
  EXPECT_FALSE(first.has_ego_pose);
  EXPECT_FALSE(first.has_map);
}
}  // namespace prediction_ml
```

Register `test_mtr_scene_adapter` in `CMakeLists.txt` and add `deep_msgs` to `package.xml` and the
CMake dependency list.

- [ ] **Step 2: Run the focused build and verify it fails**

Run inside the world-modeling development image that contains the merged `deep_ros` source:

```bash
colcon build --packages-select prediction_ml --cmake-args -DBUILD_TESTING=ON
```

Expected: FAIL because `mtr_scene_adapter.hpp` does not exist.

- [ ] **Step 3: Add the adapter interface**

Create `include/prediction_ml/mtr_scene_adapter.hpp`:

```cpp
#ifndef PREDICTION_ML__MTR_SCENE_ADAPTER_HPP_
#define PREDICTION_ML__MTR_SCENE_ADAPTER_HPP_

#include <cstdint>

#include "deep_msgs/msg/mtr_scene.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lanelet_msgs/msg/lanelet_ahead.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

namespace prediction_ml
{
class MtrSceneAdapter
{
public:
  deep_msgs::msg::MtrScene build(
    const vision_msgs::msg::Detection3DArray & detections,
    const geometry_msgs::msg::PoseStamped * ego_pose,
    const lanelet_msgs::msg::LaneletAhead * lanelets);

private:
  uint64_t sequence_{0};
};
}  // namespace prediction_ml
#endif
```

- [ ] **Step 4: Implement semantic conversion only**

Create `src/mtr_scene_adapter.cpp`:

```cpp
#include "prediction_ml/mtr_scene_adapter.hpp"

#include <string>

namespace prediction_ml
{
deep_msgs::msg::MtrScene MtrSceneAdapter::build(
  const vision_msgs::msg::Detection3DArray & detections,
  const geometry_msgs::msg::PoseStamped * ego_pose,
  const lanelet_msgs::msg::LaneletAhead * lanelets)
{
  deep_msgs::msg::MtrScene scene;
  scene.header = detections.header;
  scene.detections = detections;
  scene.request_id = std::to_string(detections.header.stamp.sec) + "-" +
    std::to_string(detections.header.stamp.nanosec) + "-" + std::to_string(sequence_++);

  if (ego_pose != nullptr) {
    scene.ego_pose = *ego_pose;
    scene.has_ego_pose = true;
  }

  if (lanelets != nullptr) {
    const auto append = [&scene](
      int64_t id, uint8_t semantic_type, const std::vector<geometry_msgs::msg::Point> & points) {
        if (points.empty()) {
          return;
        }
        deep_msgs::msg::MapPolyline polyline;
        polyline.lanelet_id = id;
        polyline.semantic_type = semantic_type;
        polyline.points = points;
        scene.map_polylines.push_back(std::move(polyline));
      };
    for (const auto & lanelet : lanelets->lanelets) {
      append(lanelet.id, deep_msgs::msg::MapPolyline::CENTERLINE, lanelet.centerline);
      append(lanelet.id, deep_msgs::msg::MapPolyline::LEFT_BOUNDARY, lanelet.left_boundary);
      append(lanelet.id, deep_msgs::msg::MapPolyline::RIGHT_BOUNDARY, lanelet.right_boundary);
    }
    scene.has_map = !scene.map_polylines.empty();
  }
  return scene;
}
}  // namespace prediction_ml
```

Add `src/mtr_scene_adapter.cpp` to `prediction_ml_lib`.

- [ ] **Step 5: Run focused tests and commit**

```bash
colcon build --packages-select prediction_ml --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select prediction_ml --ctest-args -R test_mtr_scene_adapter
colcon test-result --verbose
git add src/world_modeling/prediction_ml
git commit -m "feat(prediction_ml): adapt WATO context to deep MTR scenes"
```

Expected: both adapter tests pass.

### Task 2: Add request correlation and safe result caching

**Files:**
- Create: `src/world_modeling/prediction_ml/include/prediction_ml/mtr_result_cache.hpp`
- Create: `src/world_modeling/prediction_ml/src/mtr_result_cache.cpp`
- Create: `src/world_modeling/prediction_ml/test/test_mtr_result_cache.cpp`
- Modify: `src/world_modeling/prediction_ml/CMakeLists.txt`

- [ ] **Step 1: Write failing acceptance and rejection tests**

Create `test/test_mtr_result_cache.cpp` with these cases:

```cpp
#include <gtest/gtest.h>

#include "prediction_ml/mtr_result_cache.hpp"

namespace prediction_ml
{
namespace
{
deep_msgs::msg::MtrScene request()
{
  deep_msgs::msg::MtrScene scene;
  scene.header.frame_id = "map";
  scene.header.stamp.sec = 10;
  scene.request_id = "req-1";
  scene.detections.detections.resize(1);
  scene.detections.detections[0].id = "track-1";
  return scene;
}

deep_msgs::msg::MtrPredictionArray validResult()
{
  deep_msgs::msg::MtrPredictionArray result;
  result.header.frame_id = "map";
  result.header.stamp.sec = 10;
  result.request_id = "req-1";
  result.objects.resize(1);
  result.objects[0].track_id = "track-1";
  result.objects[0].trajectories.resize(1);
  result.objects[0].trajectories[0].confidence = 0.8F;
  result.objects[0].trajectories[0].poses.resize(1);
  result.objects[0].trajectories[0].poses[0].header.frame_id = "map";
  return result;
}
}  // namespace

TEST(MtrResultCache, ReplacesFallbackOnlyForFreshCorrelatedResult)
{
  MtrResultCache cache(0.5);
  cache.rememberRequest(request());
  EXPECT_TRUE(cache.accept(validResult(), 10.2));
  std::vector<world_model_msgs::msg::WorldObject> fallback(1);
  fallback[0].detection.id = "track-1";
  const auto output = cache.select(fallback, 10.2);
  ASSERT_EQ(output[0].predictions.size(), 1u);
  EXPECT_FLOAT_EQ(output[0].predictions[0].conf, 0.8F);
}

TEST(MtrResultCache, RejectsUnknownStaleAndMalformedResults)
{
  MtrResultCache cache(0.5);
  cache.rememberRequest(request());
  auto unknown = validResult();
  unknown.request_id = "unknown";
  EXPECT_FALSE(cache.accept(unknown, 10.1));
  EXPECT_FALSE(cache.accept(validResult(), 10.6));
  auto malformed = validResult();
  malformed.objects[0].trajectories[0].poses.clear();
  EXPECT_FALSE(cache.accept(malformed, 10.1));
}
}  // namespace prediction_ml
```

- [ ] **Step 2: Run the test and verify it fails on the missing cache**

```bash
colcon build --packages-select prediction_ml --cmake-args -DBUILD_TESTING=ON
```

Expected: FAIL because `mtr_result_cache.hpp` does not exist.

- [ ] **Step 3: Add the cache interface**

Create `include/prediction_ml/mtr_result_cache.hpp`:

```cpp
#ifndef PREDICTION_ML__MTR_RESULT_CACHE_HPP_
#define PREDICTION_ML__MTR_RESULT_CACHE_HPP_

#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "deep_msgs/msg/mtr_prediction_array.hpp"
#include "deep_msgs/msg/mtr_scene.hpp"
#include "world_model_msgs/msg/world_object.hpp"

namespace prediction_ml
{
class MtrResultCache
{
public:
  explicit MtrResultCache(double ttl_s);
  void rememberRequest(const deep_msgs::msg::MtrScene & scene);
  bool accept(const deep_msgs::msg::MtrPredictionArray & result, double now_s);
  std::vector<world_model_msgs::msg::WorldObject> select(
    const std::vector<world_model_msgs::msg::WorldObject> & fallback, double now_s) const;

private:
  struct Pending {
    double source_s;
    std::string frame_id;
    std::unordered_set<std::string> track_ids;
  };
  struct Cached {
    double source_s;
    std::vector<world_model_msgs::msg::Prediction> predictions;
  };

  double ttl_s_;
  mutable std::mutex mutex_;
  std::unordered_map<std::string, Pending> pending_;
  std::unordered_map<std::string, Cached> cached_;
};
}  // namespace prediction_ml
#endif
```

- [ ] **Step 4: Implement strict validation and conversion**

In `src/mtr_result_cache.cpp`, implement the following complete acceptance rules:

```cpp
const auto seconds = [](const builtin_interfaces::msg::Time & stamp) {
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
};
const auto finite_pose = [](const geometry_msgs::msg::PoseStamped & pose) {
  const auto & p = pose.pose.position;
  const auto & q = pose.pose.orientation;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
};
```

`rememberRequest` stores the scene timestamp, frame, and every non-empty detection ID under the
request ID. `accept` must reject unless all of these are true:

```cpp
pending_.count(result.request_id) == 1
result.header.frame_id == pending.frame_id
seconds(result.header.stamp) == pending.source_s
now_s - pending.source_s >= 0.0
now_s - pending.source_s <= ttl_s_
!result.objects.empty()
```

For every object, require a known non-empty track ID and at least one trajectory. For every
trajectory, require finite non-negative confidence, at least one pose, the expected frame on every
pose, and `finite_pose(pose)`. Convert each accepted trajectory as follows:

```cpp
world_model_msgs::msg::Prediction prediction;
prediction.header = result.header;
prediction.conf = trajectory.confidence;
prediction.poses = trajectory.poses;
```

Only after the entire result validates, replace `cached_[track_id]` entries atomically and erase
the pending request. `select` copies the fallback vector and replaces `predictions` only when the
cached track timestamp is not older than `ttl_s_`. Protect all maps with `mutex_`.

- [ ] **Step 5: Run all cache tests, then commit**

```bash
colcon build --packages-select prediction_ml --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select prediction_ml --ctest-args -R test_mtr_result_cache
colcon test-result --verbose
git add src/world_modeling/prediction_ml
git commit -m "feat(prediction_ml): validate and cache deep MTR results"
```

### Task 3: Rewire the node and remove the duplicate backend skeleton

**Files:**
- Modify: `src/world_modeling/prediction_ml/include/prediction_ml/prediction_ml_node.hpp`
- Modify: `src/world_modeling/prediction_ml/src/prediction_ml_node.cpp`
- Modify: `src/world_modeling/prediction_ml/CMakeLists.txt`
- Modify: `src/world_modeling/prediction_ml/package.xml`
- Modify: `src/world_modeling/prediction_ml/config/params.yaml`
- Delete: `src/world_modeling/prediction_ml/include/prediction_ml/mtr_backend.hpp`
- Delete: `src/world_modeling/prediction_ml/include/prediction_ml/mtr_inference_engine.hpp`
- Delete: `src/world_modeling/prediction_ml/include/prediction_ml/mtr_runtime.hpp`
- Delete: `src/world_modeling/prediction_ml/include/prediction_ml/mtr_types.hpp`
- Delete: `src/world_modeling/prediction_ml/include/prediction_ml/output_converter.hpp`
- Delete: `src/world_modeling/prediction_ml/include/prediction_ml/scene_builder.hpp`
- Delete: `src/world_modeling/prediction_ml/src/mtr_runtime.cpp`
- Delete: `src/world_modeling/prediction_ml/src/null_backend.cpp`
- Delete: `src/world_modeling/prediction_ml/src/output_converter.cpp`
- Delete: `src/world_modeling/prediction_ml/src/scene_builder.cpp`
- Delete: `src/world_modeling/prediction_ml/src/tensorrt_backend.cpp`
- Delete: `src/world_modeling/prediction_ml/test/test_backend.cpp`
- Delete: `src/world_modeling/prediction_ml/test/test_contract_compiles.cpp`
- Delete: `src/world_modeling/prediction_ml/test/test_runtime.cpp`
- Delete: `src/world_modeling/prediction_ml/test/test_scene_builder.cpp`

- [ ] **Step 1: Add a fallback-continuity node test before rewiring**

Add a test that constructs `PredictionMlNode` with `mtr.enabled=false`, configures and activates it,
publishes one tracked detection, and asserts that `world_object_seeds` receives one object with one
fallback prediction. Register it as `test_prediction_ml_node`.

- [ ] **Step 2: Add bridge state to the node header**

Replace `SceneBuilder` and `MtrRuntime` members with:

```cpp
std::unique_ptr<MtrSceneAdapter> scene_adapter_;
std::unique_ptr<MtrResultCache> result_cache_;
rclcpp_lifecycle::LifecyclePublisher<deep_msgs::msg::MtrScene>::SharedPtr mtr_scene_pub_;
rclcpp::Subscription<deep_msgs::msg::MtrPredictionArray>::SharedPtr mtr_result_sub_;
bool mtr_enabled_{false};
std::string mtr_request_topic_{"/mtr/scenes"};
std::string mtr_result_topic_{"/mtr/predictions"};
```

Add `mtrResultCallback` accepting `MtrPredictionArray::ConstSharedPtr`.

- [ ] **Step 3: Replace backend parameters and lifecycle wiring**

Declare only these MTR parameters:

```cpp
declare_parameter("mtr.enabled", false);
declare_parameter("mtr.request_topic", "/mtr/scenes");
declare_parameter("mtr.result_topic", "/mtr/predictions");
declare_parameter("mtr.cache_ttl_s", 0.5);
```

During configure, create `MtrSceneAdapter` and `MtrResultCache`. When enabled, create the scene
lifecycle publisher and result subscription. Activate/deactivate the scene publisher with the
world-object publisher and reset all bridge resources during cleanup/shutdown.

Use this callback flow:

```cpp
auto fallback = buildFallback(*msg);
const double now_s = get_clock()->now().seconds();
if (mtr_enabled_ && mtr_scene_pub_ && mtr_scene_pub_->is_activated()) {
  auto scene = scene_adapter_->build(*msg, ego_pose_.get(), lanelet_ahead_.get());
  result_cache_->rememberRequest(scene);
  mtr_scene_pub_->publish(scene);
}
world_model_msgs::msg::WorldObjectArray output;
output.header = msg->header;
output.objects = result_cache_->select(fallback, now_s);
world_objects_pub_->publish(output);
```

The result callback calls `result_cache_->accept(*msg, get_clock()->now().seconds())` and emits a
throttled warning when validation fails.

- [ ] **Step 4: Remove duplicate engine/runtime sources and tests**

Delete the files listed above. Remove `PREDICTION_ML_ENABLE_TENSORRT`, backend source selection,
old source entries, and old test targets from `CMakeLists.txt`. Keep fallback, scene-adapter,
result-cache, and node tests.

- [ ] **Step 5: Update the parameter file**

Replace the old MTR block in `config/params.yaml` with:

```yaml
    mtr:
      enabled: false
      request_topic: "/mtr/scenes"
      result_topic: "/mtr/predictions"
      cache_ttl_s: 0.5
```

- [ ] **Step 6: Run package tests and commit**

```bash
colcon build --packages-select prediction_ml --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select prediction_ml
colcon test-result --verbose
git add src/world_modeling/prediction_ml
git commit -m "refactor(prediction_ml): replace custom backend with deep ROS bridge"
```

Expected: all retained and new tests pass; `rg "TensorRt|PREDICTION_ML_ENABLE_TENSORRT|IMtrInferenceEngine" src/world_modeling/prediction_ml` returns no matches.

### Task 4: Add disabled-by-default launch wiring and correct documentation

**Files:**
- Modify: `src/world_modeling/prediction_ml/launch/prediction_ml.launch.yaml`
- Modify: `src/world_modeling/prediction_ml/README.md`
- Modify: `src/world_modeling/prediction_ml/DEVELOPING.md`
- Modify: `src/world_modeling/world_modeling_bringup/launch/world_modeling.launch.yaml`
- Modify: `src/world_modeling/world_modeling_bringup/package.xml`

- [ ] **Step 1: Replace engine launch arguments with bridge arguments**

In `prediction_ml.launch.yaml`, define `enable_mtr` defaulting to `false`. Pass it as
`mtr.enabled` to `prediction_ml_node`. Add a conditional `deep_mtr_node` entry:

```yaml
  - node:
      if: "$(var enable_mtr)"
      pkg: deep_mtr
      exec: deep_mtr_node
      name: deep_mtr_node
      namespace: "world_modeling"
      output: screen
      param:
        - from: $(find-pkg-share deep_mtr)/config/deep_mtr.yaml
```

Remove `engine_path` and `metadata_path` arguments.

- [ ] **Step 2: Keep live bringup opt-in**

Add `enable_prediction_ml` with default `false` to `world_modeling.launch.yaml`. Conditionally
include `prediction_ml.launch.yaml`, passing `enable_mtr: false`. Add `<exec_depend>prediction_ml</exec_depend>`
and `<exec_depend>deep_mtr</exec_depend>` to `world_modeling_bringup/package.xml`.

- [ ] **Step 3: Rewrite user and developer documentation**

`README.md` must describe fallback behavior and the disabled bridge. `DEVELOPING.md` must show the
cross-repository data flow and preserve this ownership statement:

```text
The inference owner implements temporal history, MTR tensor packing, ONNX model execution, and
output decoding in WATonomous/deep_ros/deep_mtr. Do not reintroduce backend or tensor code in
prediction_ml.
```

Remove `.engine`, `PREDICTION_ML_ENABLE_TENSORRT`, and direct backend-selection documentation.

- [ ] **Step 4: Validate launch descriptions and commit**

```bash
ros2 launch prediction_ml prediction_ml.launch.yaml --show-args
ros2 launch world_modeling_bringup world_modeling.launch.yaml --show-args
rg -n "\.engine|PREDICTION_ML_ENABLE_TENSORRT|tensorrt_backend" src/world_modeling/prediction_ml
```

Expected: launch parsing succeeds; the grep returns no matches.

```bash
git add src/world_modeling/prediction_ml src/world_modeling/world_modeling_bringup
git commit -m "docs(prediction_ml): document deep ROS ownership boundary"
```

### Task 5: Pin `deep_ros` into the GPU-enabled world-modeling image

**Files:**
- Modify: `docker/world_modeling.Dockerfile`
- Modify: `modules/docker-compose.yaml`
- Create: `config/deep_ros.ref`
- Modify: `watod-config.sh`

- [ ] **Step 1: Obtain the full merged `deep_ros` SHA**

Run in the merged `deep_ros` checkout:

```bash
git fetch origin main
git rev-parse origin/main
```

Expected: a 40-character SHA containing the merged `deep_mtr` commit. Record it in the monorepo
without a hand-edited placeholder:

```bash
test -n "${WATO_MONOREPO:?set WATO_MONOREPO to the monorepo checkout path}"
git rev-parse origin/main > "${WATO_MONOREPO}/config/deep_ros.ref"
test "$(wc -c < "${WATO_MONOREPO}/config/deep_ros.ref")" -eq 41
```

Expected: the length check succeeds (40 hexadecimal characters plus newline).

- [ ] **Step 2: Switch the world-modeling source image and pin the clone**

Change the first line of `docker/world_modeling.Dockerfile` to:

```dockerfile
ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:cuda12.8.1-cudnn-runtime-ubuntu24.04
```

Before local `COPY` instructions, add the recorded 40-character literal:

```dockerfile
ARG DEEP_ROS_REF
RUN test -n "${DEEP_ROS_REF}" && \
    git clone https://github.com/WATonomous/deep_ros.git deep_ros && \
    git -C deep_ros checkout --detach "${DEEP_ROS_REF}"
```

Require the build argument in all world-modeling source builds rather than defaulting to a moving
branch. Load the tracked ref file from `watod-config.sh`:

```bash
DEEP_ROS_REF=${DEEP_ROS_REF:-$(tr -d '[:space:]' < "$MONO_DIR/config/deep_ros.ref")}
export DEEP_ROS_REF
```

Pass it under the `world_modeling_source.build.args` block in `modules/docker-compose.yaml`:

```yaml
args:
  DEEP_ROS_REF: "${DEEP_ROS_REF:?}"
```

- [ ] **Step 3: Install the required TensorRT runtime libraries**

Copy only the TensorRT runtime install block from `docker/perception.Dockerfile` into the
world-modeling dependency stage:

```dockerfile
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    libnvinfer10 \
    libnvinfer-plugin10 \
    libnvonnxparsers10 && \
    rm -rf /var/lib/apt/lists/*
```

Do not copy Isaac ROS camera dependencies.

- [ ] **Step 4: Give world modeling GPU access and model storage**

Add to `world_modeling_bringup` in `modules/docker-compose.yaml`:

```yaml
runtime: nvidia
volumes:
  - ${MONO_DIR}/maps:/opt/watonomous/maps
  - ${MONO_DIR}/models:/opt/watonomous/models
environment:
  - NVIDIA_VISIBLE_DEVICES=all
  - NVIDIA_DRIVER_CAPABILITIES=all
deploy:
  resources:
    reservations:
      devices:
        - driver: nvidia
          capabilities: [gpu]
```

Retain the existing ROS and Zenoh environment entries.

- [ ] **Step 5: Build the complete world-modeling image**

```bash
./watod -m world_modeling build
```

Expected: the source stage checks out the recorded detached SHA, `deep_msgs`, `deep_mtr`, and
`prediction_ml` build, and the deploy image completes without requiring an MTR model.

- [ ] **Step 6: Commit Docker integration**

```bash
git add config/deep_ros.ref docker/world_modeling.Dockerfile modules/docker-compose.yaml watod-config.sh
git commit -m "build(world_modeling): include pinned deep ROS GPU runtime"
```

### Task 6: Run migration acceptance checks

- [ ] **Step 1: Run the world-modeling package suite**

```bash
./watod test world_modeling prediction_ml
```

Expected: all `prediction_ml` tests pass, including fallback continuity, scene adaptation, and
result rejection.

- [ ] **Step 2: Verify fallback-only standalone launch**

Start `prediction_ml.launch.yaml` with defaults and verify:

```bash
ros2 param get /world_modeling/prediction_ml_node mtr.enabled
ros2 topic info /mtr/scenes
ros2 topic echo /world_modeling/world_object_seeds --once
```

Expected: `mtr.enabled` is `false`, no scenes are published, and a tracked-detection input produces
a fallback `WorldObjectArray` without any model file.

- [ ] **Step 3: Verify the opt-in skeleton never fabricates output**

Launch with `enable_mtr:=true`, configure/activate both lifecycle nodes, publish a scene, and run:

```bash
timeout 2 ros2 topic echo /mtr/predictions --once
```

Expected: timeout exit status `124` and no message. The log contains the bounded
`MTR inference is not implemented` diagnostic.

- [ ] **Step 4: Run static ownership checks**

```bash
rg -n "PREDICTION_ML_ENABLE_TENSORRT|IMtrInferenceEngine|createTensorRt|\.engine" \
  src/world_modeling/prediction_ml
rg -n "lanelet_msgs|world_model_msgs" /ws/src/deep_ros/deep_mtr /ws/src/deep_ros/deep_msgs
```

Expected: both commands return no matches.

- [ ] **Step 5: Commit any verification-only corrections**

If verification required changes, stage only the files changed for those failures and commit:

```bash
git commit -m "fix(prediction_ml): satisfy deep ROS migration acceptance"
```

If no files changed, do not create an empty commit.

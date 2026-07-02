# Deep MTR `deep_ros` Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add neutral MTR ROS interfaces and a deliberately non-inferencing `deep_mtr` lifecycle-node skeleton to `WATonomous/deep_ros`.

**Architecture:** `deep_msgs` defines semantic scene requests and multimodal trajectory results without WATO-specific dependencies. `deep_mtr` inherits `deep_ros::DeepNodeBase`, subscribes to scenes, and intentionally publishes no predictions until the inference owner implements tensor preparation, ONNX execution, and decoding.

**Tech Stack:** ROS 2 Jazzy, C++17, `ament_cmake`, `rosidl`, `rclcpp_lifecycle`, `rclcpp_components`, `deep_core`, `deep_test`, Catch2.

---

Execute this plan in a clean `deep_ros` worktree. Do not work in the temporary planning clone. The
resulting PR must merge before executing the monorepo integration plan.

## Task 1: Define the neutral MTR message contract

**Files:**
- Create: `deep_msgs/msg/MapPolyline.msg`
- Create: `deep_msgs/msg/MtrScene.msg`
- Create: `deep_msgs/msg/MtrTrajectory.msg`
- Create: `deep_msgs/msg/MtrObjectPrediction.msg`
- Create: `deep_msgs/msg/MtrPredictionArray.msg`
- Create: `deep_msgs/test/test_mtr_messages.cpp`
- Modify: `deep_msgs/CMakeLists.txt`
- Modify: `deep_msgs/package.xml`

- [ ] **Step 1: Add the failing message-contract test**

Create `deep_msgs/test/test_mtr_messages.cpp`:

```cpp
#include <gtest/gtest.h>

#include <deep_msgs/msg/map_polyline.hpp>
#include <deep_msgs/msg/mtr_prediction_array.hpp>
#include <deep_msgs/msg/mtr_scene.hpp>

TEST(MtrMessages, PreserveCorrelationAndSemanticMapData)
{
  deep_msgs::msg::MtrScene scene;
  scene.request_id = "request-7";
  scene.has_ego_pose = true;
  scene.has_map = true;

  deep_msgs::msg::MapPolyline centerline;
  centerline.lanelet_id = 42;
  centerline.semantic_type = deep_msgs::msg::MapPolyline::CENTERLINE;
  centerline.points.resize(2);
  scene.map_polylines.push_back(centerline);

  EXPECT_EQ(scene.request_id, "request-7");
  ASSERT_EQ(scene.map_polylines.size(), 1u);
  EXPECT_EQ(scene.map_polylines.front().lanelet_id, 42);
  EXPECT_EQ(scene.map_polylines.front().semantic_type, deep_msgs::msg::MapPolyline::CENTERLINE);

  deep_msgs::msg::MtrPredictionArray result;
  result.request_id = scene.request_id;
  EXPECT_EQ(result.request_id, scene.request_id);
}
```

Inside `deep_msgs/CMakeLists.txt`, after `rosidl_generate_interfaces`, register the test:

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mtr_messages test/test_mtr_messages.cpp)
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(test_mtr_messages "${cpp_typesupport_target}")
endif()
```

Add `<test_depend>ament_cmake_gtest</test_depend>` to `deep_msgs/package.xml`.

- [ ] **Step 2: Run the build and verify it fails on missing generated headers**

Run:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select deep_msgs --cmake-args -DBUILD_TESTING=ON
```

Expected: FAIL because `deep_msgs/msg/map_polyline.hpp`, `mtr_scene.hpp`, and
`mtr_prediction_array.hpp` do not exist.

- [ ] **Step 3: Add the message definitions**

Create `deep_msgs/msg/MapPolyline.msg`:

```text
uint8 CENTERLINE=0
uint8 LEFT_BOUNDARY=1
uint8 RIGHT_BOUNDARY=2

int64 lanelet_id
uint8 semantic_type
geometry_msgs/Point[] points
```

Create `deep_msgs/msg/MtrScene.msg`:

```text
std_msgs/Header header
string request_id
vision_msgs/Detection3DArray detections
geometry_msgs/PoseStamped ego_pose
bool has_ego_pose
MapPolyline[] map_polylines
bool has_map
```

Create `deep_msgs/msg/MtrTrajectory.msg`:

```text
float32 confidence
geometry_msgs/PoseStamped[] poses
```

Create `deep_msgs/msg/MtrObjectPrediction.msg`:

```text
string track_id
MtrTrajectory[] trajectories
```

Create `deep_msgs/msg/MtrPredictionArray.msg`:

```text
std_msgs/Header header
string request_id
MtrObjectPrediction[] objects
```

- [ ] **Step 4: Register all messages and dependencies**

Add `find_package(geometry_msgs REQUIRED)` to `deep_msgs/CMakeLists.txt`, and extend
`rosidl_generate_interfaces`:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MultiImage.msg"
  "msg/MultiImageCompressed.msg"
  "msg/MultiCameraInfo.msg"
  "msg/MultiDetection2DArray.msg"
  "msg/MapPolyline.msg"
  "msg/MtrScene.msg"
  "msg/MtrTrajectory.msg"
  "msg/MtrObjectPrediction.msg"
  "msg/MtrPredictionArray.msg"
  DEPENDENCIES geometry_msgs std_msgs sensor_msgs vision_msgs
)
```

Add `<depend>geometry_msgs</depend>` and `<depend>vision_msgs</depend>` to
`deep_msgs/package.xml`.

- [ ] **Step 5: Build and test the message package**

Run:

```bash
rm -rf build/deep_msgs install/deep_msgs log
colcon build --packages-select deep_msgs --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select deep_msgs
colcon test-result --verbose
```

Expected: build succeeds and `test_mtr_messages` passes.

- [ ] **Step 6: Commit the interface contract**

```bash
git add deep_msgs
git commit -m "feat(deep_msgs): add MTR scene and prediction interfaces"
```

### Task 2: Add the non-inferencing `deep_mtr` lifecycle skeleton

**Files:**
- Create: `deep_mtr/include/deep_mtr/deep_mtr_node.hpp`
- Create: `deep_mtr/src/deep_mtr_node.cpp`
- Create: `deep_mtr/test/test_deep_mtr_node.cpp`
- Create: `deep_mtr/CMakeLists.txt`
- Create: `deep_mtr/package.xml`

- [ ] **Step 1: Write the failing lifecycle and placeholder-behavior tests**

Create `deep_mtr/test/test_deep_mtr_node.cpp`:

```cpp
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include <catch2/catch_test_macros.hpp>
#include <deep_msgs/msg/mtr_prediction_array.hpp>
#include <deep_msgs/msg/mtr_scene.hpp>
#include <deep_mtr/deep_mtr_node.hpp>
#include <deep_test/deep_test.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

TEST_CASE("DeepMtrNode runs as an empty lifecycle skeleton", "[deep_mtr][lifecycle]")
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<deep_mtr::DeepMtrNode>();

  REQUIRE(node->configure().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  REQUIRE(node->activate().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  REQUIRE(node->deactivate().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  REQUIRE(node->cleanup().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_CASE("DeepMtrNode never fabricates a prediction", "[deep_mtr][placeholder]")
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<deep_mtr::DeepMtrNode>();
  auto client = std::make_shared<rclcpp::Node>("deep_mtr_test_client");
  std::atomic<int> result_count{0};
  auto result_sub = client->create_subscription<deep_msgs::msg::MtrPredictionArray>(
    "/mtr/predictions", 10,
    [&result_count](deep_msgs::msg::MtrPredictionArray::ConstSharedPtr) {++result_count;});
  auto scene_pub = client->create_publisher<deep_msgs::msg::MtrScene>("/mtr/scenes", 10);

  REQUIRE(node->configure().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  REQUIRE(node->activate().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(client);
  deep_msgs::msg::MtrScene scene;
  scene.request_id = "must-not-produce-output";
  scene_pub->publish(scene);
  const auto deadline = std::chrono::steady_clock::now() + 300ms;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  REQUIRE(result_count.load() == 0);
  node->deactivate();
  node->cleanup();
}
```

- [ ] **Step 2: Add minimal build metadata and verify the test fails**

Create `deep_mtr/package.xml` with dependencies on `deep_core`, `deep_msgs`, `pluginlib`,
`rclcpp`, `rclcpp_components`, and `rclcpp_lifecycle`; add `deep_test` as a test dependency.
Create `deep_mtr/CMakeLists.txt` using the same exported-target pattern as `deep_sample` and
register `test_deep_mtr_node` with `add_deep_test`.

Run:

```bash
colcon build --packages-select deep_msgs deep_mtr --cmake-args -DBUILD_TESTING=ON
```

Expected: FAIL because `deep_mtr/deep_mtr_node.hpp` and its component library do not exist.

- [ ] **Step 3: Implement the lifecycle skeleton header**

Create `deep_mtr/include/deep_mtr/deep_mtr_node.hpp`:

```cpp
#pragma once

#include <memory>
#include <string>

#include <deep_core/deep_node_base.hpp>
#include <deep_msgs/msg/mtr_prediction_array.hpp>
#include <deep_msgs/msg/mtr_scene.hpp>

namespace deep_mtr
{
class DeepMtrNode : public deep_ros::DeepNodeBase
{
public:
  explicit DeepMtrNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  deep_ros::CallbackReturn on_configure_impl(const rclcpp_lifecycle::State &) override;
  deep_ros::CallbackReturn on_activate_impl(const rclcpp_lifecycle::State &) override;
  deep_ros::CallbackReturn on_deactivate_impl(const rclcpp_lifecycle::State &) override;
  deep_ros::CallbackReturn on_cleanup_impl(const rclcpp_lifecycle::State &) override;

private:
  void scene_callback(deep_msgs::msg::MtrScene::ConstSharedPtr scene);

  std::string input_topic_;
  std::string output_topic_;
  rclcpp::Subscription<deep_msgs::msg::MtrScene>::SharedPtr scene_sub_;
  rclcpp_lifecycle::LifecyclePublisher<deep_msgs::msg::MtrPredictionArray>::SharedPtr result_pub_;
};
}  // namespace deep_mtr
```

- [ ] **Step 4: Implement the intentional no-output behavior**

Create `deep_mtr/src/deep_mtr_node.cpp`:

```cpp
#include "deep_mtr/deep_mtr_node.hpp"

#include <functional>
#include <utility>

namespace deep_mtr
{
DeepMtrNode::DeepMtrNode(const rclcpp::NodeOptions & options)
: DeepNodeBase("deep_mtr_node", options)
{
  declare_parameter("input_topic", "/mtr/scenes");
  declare_parameter("output_topic", "/mtr/predictions");
}

deep_ros::CallbackReturn DeepMtrNode::on_configure_impl(const rclcpp_lifecycle::State &)
{
  input_topic_ = get_parameter("input_topic").as_string();
  output_topic_ = get_parameter("output_topic").as_string();
  result_pub_ = create_publisher<deep_msgs::msg::MtrPredictionArray>(output_topic_, 10);
  return deep_ros::CallbackReturn::SUCCESS;
}

deep_ros::CallbackReturn DeepMtrNode::on_activate_impl(const rclcpp_lifecycle::State &)
{
  result_pub_->on_activate();
  scene_sub_ = create_subscription<deep_msgs::msg::MtrScene>(
    input_topic_, 10, std::bind(&DeepMtrNode::scene_callback, this, std::placeholders::_1));
  return deep_ros::CallbackReturn::SUCCESS;
}

deep_ros::CallbackReturn DeepMtrNode::on_deactivate_impl(const rclcpp_lifecycle::State &)
{
  scene_sub_.reset();
  result_pub_->on_deactivate();
  return deep_ros::CallbackReturn::SUCCESS;
}

deep_ros::CallbackReturn DeepMtrNode::on_cleanup_impl(const rclcpp_lifecycle::State &)
{
  scene_sub_.reset();
  result_pub_.reset();
  return deep_ros::CallbackReturn::SUCCESS;
}

void DeepMtrNode::scene_callback(deep_msgs::msg::MtrScene::ConstSharedPtr scene)
{
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "MTR inference is not implemented; ignoring scene request '%s'", scene->request_id.c_str());
}
}  // namespace deep_mtr

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(deep_mtr::DeepMtrNode)
```

- [ ] **Step 5: Complete `CMakeLists.txt` and `package.xml`**

Use this target structure in `deep_mtr/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.22)
project(deep_mtr)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(deep_core REQUIRED)
find_package(deep_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(deep_mtr_component SHARED src/deep_mtr_node.cpp)
target_include_directories(deep_mtr_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(deep_mtr_component PUBLIC
  pluginlib::pluginlib rclcpp::rclcpp rclcpp_components::component
  rclcpp_lifecycle::rclcpp_lifecycle ${deep_msgs_TARGETS}
  PRIVATE deep_core::deep_core_lib)
rclcpp_components_register_node(deep_mtr_component
  PLUGIN deep_mtr::DeepMtrNode EXECUTABLE deep_mtr_node)

install(TARGETS deep_mtr_component EXPORT export_deep_mtr
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(deep_test REQUIRED)
  add_deep_test(test_deep_mtr_node test/test_deep_mtr_node.cpp
    LIBRARIES deep_mtr_component deep_core::deep_core_lib)
endif()

ament_export_targets(export_deep_mtr HAS_LIBRARY_TARGET)
ament_package()
```

- [ ] **Step 6: Run the focused tests**

```bash
rm -rf build/deep_mtr install/deep_mtr
colcon build --packages-select deep_msgs deep_mtr --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select deep_msgs deep_mtr
colcon test-result --verbose
```

Expected: all focused tests pass; the placeholder test observes zero result messages.

- [ ] **Step 7: Commit the node skeleton**

```bash
git add deep_mtr
git commit -m "feat(deep_mtr): add non-inferencing node skeleton"
```

### Task 3: Add configuration, launch, and ownership documentation

**Files:**
- Create: `deep_mtr/config/deep_mtr.yaml`
- Create: `deep_mtr/launch/deep_mtr.launch.yaml`
- Create: `deep_mtr/README.md`
- Create: `deep_mtr/DEVELOPING.md`
- Modify: `deep_mtr/CMakeLists.txt`
- Modify: `README.md`

- [ ] **Step 1: Add a backend-neutral skeleton configuration**

Create `deep_mtr/config/deep_mtr.yaml`:

```yaml
---
deep_mtr_node:
  ros__parameters:
    Backend:
      plugin: ""
    model_path: ""
    Bond:
      enable: false
    input_topic: "/mtr/scenes"
    output_topic: "/mtr/predictions"
```

Create `deep_mtr/launch/deep_mtr.launch.yaml` with one lifecycle node using this parameter file.
Install `config` and `launch` from `CMakeLists.txt`:

```cmake
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
```

- [ ] **Step 2: Document the exact ownership boundary**

In `deep_mtr/README.md`, state that the package is buildable migration scaffolding and does not
run MTR. In `deep_mtr/DEVELOPING.md`, assign these follow-up seams to the inference owner:

```text
1. Maintain per-track history from repeated MtrScene messages.
2. Pack the verified MTR ONNX input contract.
3. Call DeepNodeBase::run_inference through onnxruntime_gpu.
4. Decode outputs into MtrPredictionArray.
5. Add real-model contract, correctness, and GPU tests.
```

Document eventual deployment parameters as `Backend.plugin: onnxruntime_gpu` and
`Backend.execution_provider: tensorrt`; explicitly reject direct `.engine` loading.

- [ ] **Step 3: Run formatting and the full repository test suite**

```bash
pre-commit run --all-files
rm -rf build install log
colcon build --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
IS_CI=1 colcon test
colcon test-result --verbose
```

Expected: formatting succeeds, all packages build, and all non-GPU tests pass.

- [ ] **Step 4: Commit documentation and launch assets**

```bash
git add README.md deep_mtr
git commit -m "docs(deep_mtr): define migration ownership boundary"
```

### Task 4: Merge `deep_ros` first and record the immutable handoff SHA

- [ ] **Step 1: Push the `deep_ros` branch and open its PR**

Push the branch and open a PR whose description states that working inference is excluded and the
node intentionally publishes no result.

- [ ] **Step 2: After approval, merge and record the exact merge commit**

```bash
git fetch origin main
git rev-parse origin/main
```

Expected: one 40-character SHA. Copy that exact value into Task 5 of the monorepo integration plan;
do not use `main`, a branch name, or an abbreviated SHA in the deployment Dockerfile.

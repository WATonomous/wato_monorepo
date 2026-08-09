// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <tf2_ros/buffer.h>

#include <memory>
#include <string>
#include <vector>

#include <catch2/catch_all.hpp>
#include <deep_msgs/msg/multi_camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <wato_test/wato_test.hpp>

#define private public
#include "bevfusion/bevfusion_node.hpp"
#undef private

using wato::perception::bevfusion::BEVFusionNode;

// =============================================================================
// TEST 1: Constructor starts unconfigured
// WHY: The node should be created without side effects before lifecycle hooks
//      are invoked. This is the cheapest sanity check that does not require the
//      GPU model assets used by configure().
// =============================================================================
TEST_CASE("BEVFusionNode: constructor starts unconfigured", "[node][fast]")
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  auto node = std::make_shared<BEVFusionNode>(rclcpp::NodeOptions{});

  REQUIRE(node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  rclcpp::shutdown();
}

// =============================================================================
// TEST 2: Parameter overrides via NodeOptions are respected
// WHY: The node must accept runtime overrides (e.g., from YAML via launch file)
//      so teams can swap model directories or tune thresholds without recompiling.
// =============================================================================
TEST_CASE("declareParameters: NodeOptions overrides are respected", "[params][fast]")
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  rclcpp::NodeOptions options;
  options.append_parameter_override("model_dir", "/custom/test/bevfusion/model");
  options.append_parameter_override("confidence_threshold", 0.75);

  auto node = std::make_shared<BEVFusionNode>(options);
  node->declareParameters();

  REQUIRE(node->get_parameter("model_dir").as_string() == "/custom/test/bevfusion/model");
  REQUIRE_THAT(node->get_parameter("confidence_threshold").as_double(), Catch::Matchers::WithinRel(0.75));

  rclcpp::shutdown();
}

// =============================================================================
// TEST 3: Activation uses cached camera info
// WHY: on_activate() now tries to compute calibration immediately if camera info
//      was already cached. This guards the new startup path so a node that
//      receives camera info before activation still transitions cleanly.
// =============================================================================
TEST_CASE("BEVFusionNode: activation computes calibration from cached camera info", "[node][activation][fast]")
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  auto node = std::make_shared<BEVFusionNode>(rclcpp::NodeOptions{});
  node->declareParameters();

  node->subscriber_qos_ = node->createSubscriberQoS("best_effort", 10);
  node->publisher_qos_ = node->createPublisherQoS("reliable", "transient_local", 10);
  node->sync_queue_size_ = 10;
  node->sync_max_time_diff_ms_ = 200.0;
  node->sync_max_time_diff_sec_ = 0.2;

  deep_msgs::msg::MultiCameraInfo camera_info;
  camera_info.camera_infos.resize(1);
  camera_info.camera_infos[0].header.frame_id = "cam0";

  node->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  geometry_msgs::msg::TransformStamped tf_stamped;
  tf_stamped.header.frame_id = node->lidar_frame_id_;
  tf_stamped.child_frame_id = "cam0";
  tf_stamped.transform.rotation.w = 1.0;
  node->tf_buffer_->setTransform(tf_stamped, "test");

  {
    std::lock_guard<std::mutex> lock(node->camera_info_mutex_);
    node->cached_multi_camera_info_ = std::make_shared<deep_msgs::msg::MultiCameraInfo>(camera_info);
  }

  REQUIRE(
    node->on_activate(node->get_current_state()) ==
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
  REQUIRE(node->calibration_initialized_.load());

  REQUIRE(
    node->on_deactivate(node->get_current_state()) ==
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
  REQUIRE(
    node->on_cleanup(node->get_current_state()) ==
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  rclcpp::shutdown();
}

// =============================================================================
// TEST 4: Cleanup clears cached camera info state
// WHY: The cached camera info is protected by a mutex and must be reset during
//      cleanup so later reconfiguration starts from a clean slate.
// =============================================================================
TEST_CASE("BEVFusionNode: cleanup clears cached camera info and calibration state", "[node][cleanup][fast]")
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  auto node = std::make_shared<BEVFusionNode>(rclcpp::NodeOptions{});
  node->declareParameters();

  deep_msgs::msg::MultiCameraInfo camera_info;
  camera_info.camera_infos.resize(1);
  camera_info.camera_infos[0].header.frame_id = "cam0";

  node->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  geometry_msgs::msg::TransformStamped tf_stamped;
  tf_stamped.header.frame_id = node->lidar_frame_id_;
  tf_stamped.child_frame_id = "cam0";
  tf_stamped.transform.rotation.w = 1.0;
  node->tf_buffer_->setTransform(tf_stamped, "test");
  {
    std::lock_guard<std::mutex> lock(node->camera_info_mutex_);
    node->cached_multi_camera_info_ = std::make_shared<deep_msgs::msg::MultiCameraInfo>(camera_info);
  }

  node->computeCalibrationMatrices();
  REQUIRE(node->calibration_initialized_.load());

  REQUIRE(
    node->on_cleanup(node->get_current_state()) ==
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  {
    std::lock_guard<std::mutex> lock(node->camera_info_mutex_);
    REQUIRE(node->cached_multi_camera_info_ == nullptr);
  }
  REQUIRE_FALSE(node->calibration_initialized_.load());

  rclcpp::shutdown();
}

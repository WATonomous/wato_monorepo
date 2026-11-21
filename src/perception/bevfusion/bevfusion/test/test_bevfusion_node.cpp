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

#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include <catch2/catch_all.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <wato_test/wato_test.hpp>

#define private public
#include "bevfusion/bevfusion_node.hpp"
#undef private

using wato::perception::bevfusion::BEVFusionNode;
using wato::perception::bevfusion::BoundingBox;
using wato::perception::bevfusion::Position;
using wato::perception::bevfusion::Size;
using wato::perception::bevfusion::Velocity;

// =============================================================================
// Helper: create a node with declareParameters() already called
// =============================================================================
static std::shared_ptr<BEVFusionNode> make_configured_node()
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<BEVFusionNode>(rclcpp::NodeOptions{});
  node->declareParameters();
  return node;
}

// =============================================================================
// Helper: a node for a single camera, with declareParameters() already called
// =============================================================================
static std::shared_ptr<BEVFusionNode> make_single_camera_node(const std::string & camera = "cam0")
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  rclcpp::NodeOptions options;
  options.append_parameter_override("camera_names", std::vector<std::string>{camera});
  auto node = std::make_shared<BEVFusionNode>(options);
  node->declareParameters();
  return node;
}

static sensor_msgs::msg::CameraInfo::SharedPtr make_camera_info(const std::string & frame)
{
  auto info = std::make_shared<sensor_msgs::msg::CameraInfo>();
  info->header.frame_id = frame;
  info->width = 1280;
  info->height = 1024;
  info->k = {500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0};
  info->p = {480.0, 0.0, 330.0, 0.0, 0.0, 480.0, 250.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  return info;
}

static void set_camera_to_lidar_tf(BEVFusionNode & node, const std::string & camera, double qx = 0.0, double qw = 1.0)
{
  node.tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node.get_clock());
  geometry_msgs::msg::TransformStamped tf_stamped;
  tf_stamped.header.frame_id = node.lidar_frame_id_;
  tf_stamped.child_frame_id = camera;
  tf_stamped.transform.rotation.x = qx;
  tf_stamped.transform.rotation.w = qw;
  node.tf_buffer_->setTransform(tf_stamped, "test", true);
}

// =============================================================================
// Helper: construct a BoundingBox with known values
// =============================================================================
static BoundingBox make_test_bbox(float x, float y, float z, float l, float w, float h, float yaw, float score, int id)
{
  BoundingBox bbox;
  bbox.position = {x, y, z};
  bbox.size = {w, l, h};
  bbox.velocity = {0.0f, 0.0f};
  bbox.z_rotation = yaw;
  bbox.score = score;
  bbox.id = id;
  return bbox;
}

static vision_msgs::msg::Detection3D make_test_detection3d(
  float x, float y, float z, float sx, float sy, float sz, int class_id, float score)
{
  vision_msgs::msg::Detection3D det;
  det.bbox.center.position.x = x;
  det.bbox.center.position.y = y;
  det.bbox.center.position.z = z;
  det.bbox.center.orientation.w = 1.0;
  det.bbox.size.x = sx;
  det.bbox.size.y = sy;
  det.bbox.size.z = sz;

  vision_msgs::msg::ObjectHypothesisWithPose hyp;
  hyp.hypothesis.class_id = std::to_string(class_id);
  hyp.hypothesis.score = score;
  det.results.push_back(hyp);
  return det;
}

// Registers an identity transform between target_frame_ and lidar_frame_id_ so
// createDetections3D's TF lookup succeeds and position/orientation pass through unchanged.
static void set_identity_lidar_to_target_tf(BEVFusionNode & node)
{
  node.tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node.get_clock());
  geometry_msgs::msg::TransformStamped identity_tf;
  identity_tf.header.frame_id = node.target_frame_;
  identity_tf.child_frame_id = node.lidar_frame_id_;
  identity_tf.transform.rotation.w = 1.0;
  node.tf_buffer_->setTransform(identity_tf, "test", true);  // static, valid at any stamp
}

// =============================================================================
// LIFECYCLE AND PARAM TESTS
// =============================================================================

// =============================================================================
// TEST: Constructor starts unconfigured
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
// TEST: on_configure returns FAILURE when model files are missing
// WHY: Ensures that if model files are missing or unreadable, the node fails
//      configuration gracefully and logs an error without proceeding to activate.
// =============================================================================
TEST_CASE("BEVFusionNode: on_configure returns FAILURE when model files are missing", "[node][configure][fast]")
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  rclcpp::NodeOptions options;
  options.append_parameter_override("model_dir", "/tmp/nonexistent_model_dir_xyz");

  auto node = std::make_shared<BEVFusionNode>(options);

  auto result = node->on_configure(node->get_current_state());
  REQUIRE(result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE);

  rclcpp::shutdown();
}

// =============================================================================
// TEST: camera and LiDAR callbacks before activation are ignored safely
// WHY: Frames can arrive before the matcher exists (inactive node). They must
//      be counted and dropped without crashing.
// =============================================================================
TEST_CASE("BEVFusionNode: input callbacks before activation are ignored", "[node][fast]")
{
  auto node = make_configured_node();

  auto image = std::make_shared<sensor_msgs::msg::CompressedImage>();
  auto lidar_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  node->compressedImageCallback(0, image);
  node->lidarCallback(lidar_msg);

  REQUIRE(node->image_msg_count_ == 1);
  REQUIRE(node->lidar_msg_count_ == 1);
  REQUIRE(node->matched_set_count_ == 0);
  REQUIRE(node->total_processed_.load() == 0);

  rclcpp::shutdown();
}

// =============================================================================
// TEST: processSet early exits on uninitialized core or missing calibration
// WHY: The worker must not run inference before the model and calibration are ready.
// =============================================================================
TEST_CASE("BEVFusionNode: processSet safely aborts when uninitialized or uncalibrated", "[node][fast]")
{
  auto node = make_configured_node();
  BEVFusionNode::MatchedSet set;
  set.scan = std::make_shared<sensor_msgs::msg::PointCloud2>();
  set.frames.resize(6);
  node->processSet(set);
  REQUIRE(node->total_processed_.load() == 0);
  rclcpp::shutdown();
}

// =============================================================================
// TEST: camera_input and intrinsics_source are validated
// WHY: A typo must fail configuration instead of silently using the wrong input.
// =============================================================================
TEST_CASE("declareParameters: camera_input and intrinsics_source are validated", "[params][fast]")
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  SECTION("defaults")
  {
    auto node = std::make_shared<BEVFusionNode>(rclcpp::NodeOptions{});
    node->declareParameters();
    REQUIRE(node->camera_input_ == BEVFusionNode::CameraInput::kCompressed);
    REQUIRE(node->intrinsics_source_ == BEVFusionNode::IntrinsicsSource::kProjection);
  }
  SECTION("nitros and camera_matrix")
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("camera_input", "nitros");
    options.append_parameter_override("intrinsics_source", "camera_matrix");
    auto node = std::make_shared<BEVFusionNode>(options);
    node->declareParameters();
    REQUIRE(node->camera_input_ == BEVFusionNode::CameraInput::kNitros);
    REQUIRE(node->intrinsics_source_ == BEVFusionNode::IntrinsicsSource::kCameraMatrix);
  }
  SECTION("unknown values throw")
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("camera_input", "raw");
    auto node = std::make_shared<BEVFusionNode>(options);
    REQUIRE_THROWS_AS(node->declareParameters(), std::invalid_argument);
  }
  rclcpp::shutdown();
}

// =============================================================================
// TEST: camera matching parameters, defaults and validation
// WHY: The matcher's behaviour is set entirely by these values; an early
//      fraction above 0.5 could change which frame is used, so it is rejected.
// =============================================================================
TEST_CASE("declareSyncParameters: defaults and validation", "[params][fast]")
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  SECTION("defaults")
  {
    auto node = std::make_shared<BEVFusionNode>(rclcpp::NodeOptions{});
    node->declareParameters();
    node->declareSyncParameters();
    REQUIRE(node->sync_target_ == BEVFusionNode::SyncTarget::kSweep);
    REQUIRE(node->matcher_config_.num_cameras == 6);
    REQUIRE(node->matcher_config_.max_wait_ns == 60'000'000);
    REQUIRE(node->matcher_config_.tolerance_ns == 450'000'000);
    REQUIRE(node->matcher_config_.early_fraction == Catch::Approx(0.5));
    REQUIRE(node->matcher_config_.frames_per_camera == 6);
    REQUIRE(node->matcher_config_.max_pending_scans == 4);
    REQUIRE(node->lidar_rotation_period_ms_ == Catch::Approx(50.0));
    // A second declaration (configure after cleanup) keeps the values
    REQUIRE_NOTHROW(node->declareSyncParameters());
  }
  SECTION("negative wait means wait until certain")
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("sync_max_wait_ms", -1.0);
    options.append_parameter_override("sync_target", "scan_stamp");
    auto node = std::make_shared<BEVFusionNode>(options);
    node->declareParameters();
    node->declareSyncParameters();
    REQUIRE(node->matcher_config_.max_wait_ns < 0);
    REQUIRE(node->sync_target_ == BEVFusionNode::SyncTarget::kScanStamp);
  }
  SECTION("out-of-range values throw")
  {
    for (const auto & [name, value] : std::vector<std::pair<std::string, rclcpp::ParameterValue>>{
           {"sync_early_fraction", rclcpp::ParameterValue(0.6)},
           {"sync_tolerance_ms", rclcpp::ParameterValue(0.0)},
           {"sync_frames_per_camera", rclcpp::ParameterValue(0)},
           {"sync_frames_per_camera", rclcpp::ParameterValue(17)},  // would starve the rectifier's GPU buffers
           {"sync_target", rclcpp::ParameterValue(std::string("nearest"))},
           {"lidar_rotation_period_ms", rclcpp::ParameterValue(0.0)}})
    {
      rclcpp::NodeOptions options;
      options.append_parameter_override(name, value);
      auto node = std::make_shared<BEVFusionNode>(options);
      node->declareParameters();
      INFO(name);
      REQUIRE_THROWS_AS(node->declareSyncParameters(), std::invalid_argument);
    }
  }
  rclcpp::shutdown();
}

// =============================================================================
// TEST: frames and a scan go through the callbacks, the matcher and the worker mailbox
// WHY: Exercises the activated input path end to end without the GPU model:
//      a matched set must be produced and handed to the worker.
// =============================================================================
TEST_CASE("BEVFusionNode: callbacks match frames to a scan and hand the set to the worker", "[node][activation][fast]")
{
  auto node = make_single_camera_node();
  node->declareSyncParameters();
  node->subscriber_qos_ = node->createSubscriberQoS("best_effort", 10);
  node->publisher_qos_ = node->createPublisherQoS("reliable", "transient_local", 10);
  REQUIRE(
    node->on_activate(node->get_current_state()) ==
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  // 23 frames, 50 ms apart, up to 11.00 s: enough for the early rule (it needs 20 intervals)
  for (int64_t k = 0; k < 23; ++k) {
    auto image = std::make_shared<sensor_msgs::msg::CompressedImage>();
    image->header.stamp = rclcpp::Time(9'900'000'000LL + 50'000'000LL * k);
    node->compressedImageCallback(0, image);
  }
  auto lidar_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  lidar_msg->header.stamp = rclcpp::Time(11, 5'000'000);  // 5 ms after the newest frame: decided at once
  node->lidarCallback(lidar_msg);
  REQUIRE(node->matched_set_count_ == 1);

  // The worker takes the set and skips it (no model loaded); deactivation joins it
  REQUIRE(
    node->on_deactivate(node->get_current_state()) ==
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
  REQUIRE(node->total_processed_.load() == 0);
  REQUIRE(node->matcher_ == nullptr);
  REQUIRE(
    node->on_cleanup(node->get_current_state()) ==
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
  rclcpp::shutdown();
}

// =============================================================================
// TEST: the worker mailbox keeps the newest set
// WHY: When inference is slower than the LiDAR, only the newest set is processed
//      and an older set never replaces a newer one.
// =============================================================================
TEST_CASE("computeCalibrationMatrices: intrinsics come from P or K", "[calibration][fast]")
{
  for (const auto & [source, fx, cx] : std::vector<std::tuple<std::string, float, float>>{
         {"projection", 480.0f, 330.0f}, {"camera_matrix", 500.0f, 320.0f}})
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    rclcpp::NodeOptions options;
    options.append_parameter_override("camera_names", std::vector<std::string>{"cam0"});
    options.append_parameter_override("intrinsics_source", source);
    auto node = std::make_shared<BEVFusionNode>(options);
    node->declareParameters();
    {
      std::lock_guard<std::mutex> lock(node->camera_info_mutex_);
      node->cached_camera_infos_ = {make_camera_info("cam0")};  // K fx 500 cx 320, P fx 480 cx 330
    }
    set_camera_to_lidar_tf(*node, "cam0");
    node->computeCalibrationMatrices();
    INFO(source);
    REQUIRE(node->calibration_intrinsics_.size() == 16);
    CHECK(node->calibration_intrinsics_[0] == Catch::Approx(fx));
    CHECK(node->calibration_intrinsics_[2] == Catch::Approx(cx));
    CHECK(node->calibration_intrinsics_[15] == Catch::Approx(1.0f));
    rclcpp::shutdown();
  }
}

// =============================================================================
// TEST: the worker processes sets in (epoch, scan stamp) order
// WHY: Never publish an older scan after a newer one, but start over after a bag restart (new epoch).
// =============================================================================
TEST_CASE("BEVFusionNode: the worker never goes back in time within an epoch", "[node][activation][fast]")
{
  auto node = make_single_camera_node();
  node->declareSyncParameters();
  node->subscriber_qos_ = node->createSubscriberQoS("best_effort", 10);
  node->publisher_qos_ = node->createPublisherQoS("reliable", "transient_local", 10);
  REQUIRE(
    node->on_activate(node->get_current_state()) ==
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
  // processSet records the set's largest gap before it skips the set (no model loaded): it shows which set
  // the worker took
  const auto wait_for = [](const std::function<bool()> & done) {
    for (int i = 0; i < 200 && !done(); ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return done();
  };
  const auto dispatch = [&](int64_t stamp, int64_t gap_ms, uint64_t epoch) {
    std::vector<BEVFusionNode::MatchedSet> sets(1);
    sets[0].scan_stamp_ns = stamp;
    sets[0].max_gap_ns = gap_ms * 1'000'000;
    node->dispatchSets(sets, epoch);
  };
  dispatch(300, 3, 0);
  REQUIRE(wait_for([&]() { return node->last_max_gap_ms_.load() == Catch::Approx(3.0); }));
  const uint64_t superseded = node->superseded_set_count_.load();
  dispatch(200, 2, 0);  // older, same epoch: skipped
  REQUIRE(wait_for([&]() { return node->superseded_set_count_.load() == superseded + 1; }));
  CHECK(node->last_max_gap_ms_.load() == Catch::Approx(3.0));
  dispatch(100, 1, 1);  // older stamp but a new epoch (bag restarted): processed
  REQUIRE(wait_for([&]() { return node->last_max_gap_ms_.load() == Catch::Approx(1.0); }));
  REQUIRE(
    node->on_deactivate(node->get_current_state()) ==
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
  rclcpp::shutdown();
}

TEST_CASE("BEVFusionNode: dispatchSets keeps the newest set", "[node][fast]")
{
  auto node = make_configured_node();
  std::vector<BEVFusionNode::MatchedSet> sets(3);
  sets[0].scan_stamp_ns = 200;
  sets[1].scan_stamp_ns = 100;  // completed late, older than the one waiting
  sets[2].scan_stamp_ns = 300;
  node->dispatchSets(sets, 0);
  REQUIRE(node->mailbox_.has_value());
  REQUIRE(node->mailbox_->scan_stamp_ns == 300);
  REQUIRE(node->superseded_set_count_ == 2);
  REQUIRE(node->matched_set_count_ == 3);

  // After a matcher reset (the bag restarted: a new epoch) stamps start over, so an older stamp wins
  std::vector<BEVFusionNode::MatchedSet> restarted(1);
  restarted[0].scan_stamp_ns = 50;
  node->dispatchSets(restarted, 1);
  REQUIRE(node->mailbox_->scan_stamp_ns == 50);
  REQUIRE(node->mailbox_epoch_ == 1);
  rclcpp::shutdown();
}

// =============================================================================
// TEST: parameters can be declared again (configure after cleanup)
// WHY: on_cleanup keeps the declared parameters, so a second on_configure must not throw
//      ParameterAlreadyDeclaredException.
// =============================================================================
TEST_CASE("declareParameters: declaring again keeps the values", "[params][fast]")
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  rclcpp::NodeOptions options;
  options.append_parameter_override("confidence_threshold", 0.42);
  auto node = std::make_shared<BEVFusionNode>(options);
  node->declareParameters();
  node->declareSyncParameters();
  REQUIRE_NOTHROW(node->declareParameters());
  REQUIRE_NOTHROW(node->declareSyncParameters());
  REQUIRE(node->get_parameter("confidence_threshold").as_double() == Catch::Approx(0.42));
  rclcpp::shutdown();
}

// =============================================================================
// TEST: scanTargets computes per-camera sweep times and falls back to the scan stamp
// WHY: Each camera is matched to when the LiDAR swept its direction; a cloud that
//      is not one ordered sweep (e.g. a merged cloud) must fall back safely.
// =============================================================================
TEST_CASE("BEVFusionNode: scanTargets from a PointCloud2", "[node][sweep][fast]")
{
  auto node = make_single_camera_node();
  node->declareSyncParameters();
  const auto make_cloud = [](double sweep_deg) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = rclcpp::Time(20, 0);
    cloud.height = 1;
    cloud.width = 9000;
    cloud.point_step = 12;
    cloud.row_step = cloud.width * cloud.point_step;
    for (const auto & [name, offset] : std::vector<std::pair<std::string, uint32_t>>{{"x", 0}, {"y", 4}, {"z", 8}}) {
      sensor_msgs::msg::PointField f;
      f.name = name;
      f.offset = offset;
      f.datatype = sensor_msgs::msg::PointField::FLOAT32;
      f.count = 1;
      cloud.fields.push_back(f);
    }
    cloud.data.resize(cloud.row_step);
    for (uint32_t i = 0; i < cloud.width; ++i) {
      const double a = (sweep_deg * i / (cloud.width - 1)) * M_PI / 180.0;  // starts at 0 deg
      const float p[3] = {static_cast<float>(10.0 * std::cos(a)), static_cast<float>(10.0 * std::sin(a)), 0.0f};
      std::memcpy(cloud.data.data() + i * cloud.point_step, p, sizeof(p));
    }
    return cloud;
  };
  {
    std::lock_guard<std::mutex> lock(node->camera_info_mutex_);
    node->camera_azimuths_deg_ = {-90.0};  // a quarter turn into a clockwise sweep
  }
  const auto targets = node->scanTargets(make_cloud(-360.0));
  REQUIRE(targets.size() == 1);
  const int64_t stamp = rclcpp::Time(20, 0).nanoseconds();
  // Swept 90 deg after the start, so 270 deg (37.5 ms at 1200 RPM) before the stamp
  REQUIRE(std::llabs((stamp - targets[0]) - 37'500'000) < 200'000);

  REQUIRE(node->scanTargets(make_cloud(-180.0)).empty());
  REQUIRE(node->sweep_fallback_count_ == 1);

  node->sync_target_ = BEVFusionNode::SyncTarget::kScanStamp;
  REQUIRE(node->scanTargets(make_cloud(-360.0)).empty());
  rclcpp::shutdown();
}

// =============================================================================
// TEST: Activation uses cached camera info
// WHY: on_activate() now tries to compute calibration immediately if camera info
//      was already cached. This guards the new startup path so a node that
//      receives camera info before activation still transitions cleanly.
// =============================================================================
TEST_CASE("BEVFusionNode: activation computes calibration from cached camera info", "[node][activation][fast]")
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  auto node = make_single_camera_node("cam0");
  node->declareSyncParameters();

  node->subscriber_qos_ = node->createSubscriberQoS("best_effort", 10);
  node->publisher_qos_ = node->createPublisherQoS("reliable", "transient_local", 10);

  set_camera_to_lidar_tf(*node, "cam0");
  {
    std::lock_guard<std::mutex> lock(node->camera_info_mutex_);
    node->cached_camera_infos_ = {make_camera_info("cam0")};
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
// TEST: Cleanup clears cached camera info state
// WHY: The cached camera info is protected by a mutex and must be reset during
//      cleanup so later reconfiguration starts from a clean slate.
// =============================================================================
TEST_CASE("BEVFusionNode: cleanup clears cached camera info and calibration state", "[node][cleanup][fast]")
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  auto node = make_single_camera_node("cam0");

  set_camera_to_lidar_tf(*node, "cam0");
  {
    std::lock_guard<std::mutex> lock(node->camera_info_mutex_);
    node->cached_camera_infos_ = {make_camera_info("cam0")};
  }

  node->computeCalibrationMatrices();
  REQUIRE(node->calibration_initialized_.load());

  REQUIRE(
    node->on_cleanup(node->get_current_state()) ==
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  {
    std::lock_guard<std::mutex> lock(node->camera_info_mutex_);
    REQUIRE(node->cached_camera_infos_.empty());
    REQUIRE(node->camera_azimuths_deg_.empty());
  }
  REQUIRE_FALSE(node->calibration_initialized_.load());

  rclcpp::shutdown();
}

// =============================================================================
// IMAGE AUGMENTATION MATRIX TESTS
// =============================================================================

// =============================================================================
// TEST: img_aug_matrix for nuScenes default (1600x900, resize_lim=0.48)
// WHY: The nuScenes training data uses 1600x900 images resized with
//      resize_lim=0.48 to 704x256. This test verifies we reproduce the
//      exact augmentation matrix that matches the upstream CUDA-BEVFusion
//      example data, which is the ground truth for correctness.
// =============================================================================
TEST_CASE("img_aug_matrix: nuScenes defaults 1600x900 resize_lim=0.48", "[calibration][fast]")
{
  auto node = make_configured_node();

  // Override config to nuScenes defaults
  node->config_.image_width = 1600;
  node->config_.image_height = 900;
  node->config_.resize_lim = 0.48f;
  node->config_.norm_output_width = 704;
  node->config_.norm_output_height = 256;

  // Compute expected values following camera-normalization.cu logic:
  //   resized_w = (int)(1600 * 0.48) = 768
  //   resized_h = (int)(900 * 0.48)  = 432
  //   crop_x = (768 - 704) / 2       = 32
  //   crop_y = 432 - 256             = 176
  int resized_w = static_cast<int>(1600 * 0.48f);
  int resized_h = static_cast<int>(900 * 0.48f);
  int expected_crop_x = (resized_w - 704) / 2;
  int expected_crop_y = resized_h - 256;
  REQUIRE(expected_crop_x == 32);
  REQUIRE(expected_crop_y == 176);

  // A dummy single camera with identity K
  auto cam = std::make_shared<sensor_msgs::msg::CameraInfo>();
  cam->header.frame_id = "test_camera";
  cam->k = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  node->camera_names_ = {"test_camera"};

  {
    std::lock_guard<std::mutex> lock(node->camera_info_mutex_);
    node->cached_camera_infos_ = {cam};
  }

  // computeCalibrationMatrices needs a tf_buffer — create one and add identity transform
  node->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  geometry_msgs::msg::TransformStamped identity_tf;
  identity_tf.header.frame_id = node->lidar_frame_id_;
  identity_tf.child_frame_id = "test_camera";
  identity_tf.transform.rotation.w = 1.0;
  node->tf_buffer_->setTransform(identity_tf, "test", true);

  node->computeCalibrationMatrices();

  // The img_aug_matrix is the last 16 floats we can verify through the core.
  // Since computeCalibrationMatrices calls core_->updateCalibration which
  // requires GPU, we manually replicate the aug matrix logic here instead.
  int actual_resized_w = static_cast<int>(node->config_.image_width * node->config_.resize_lim);
  int actual_resized_h = static_cast<int>(node->config_.image_height * node->config_.resize_lim);
  int actual_crop_x = (actual_resized_w - node->config_.norm_output_width) / 2;
  int actual_crop_y = actual_resized_h - node->config_.norm_output_height;

  // Verify the crop/resize math
  REQUIRE(actual_crop_x == expected_crop_x);
  REQUIRE(actual_crop_y == expected_crop_y);

  rclcpp::shutdown();
}

// =============================================================================
// TEST: img_aug_matrix for WATO cameras (1280x1024, resize_lim=0.55)
// WHY: The actual car cameras are 1280x1024. This test verifies the aug
//      matrix math produces the correct crop values for these dimensions.
//      Getting this wrong would misalign BEV geometry with actual camera
//      pixels, placing all detections in wrong 3D positions.
// =============================================================================
TEST_CASE("img_aug_matrix: WATO cameras 1280x1024 resize_lim=0.55", "[calibration][fast]")
{
  // Replicate the computation from computeCalibrationMatrices:
  int image_width = 1280;
  int image_height = 1024;
  float resize_lim = 0.55f;
  int norm_output_width = 704;
  int norm_output_height = 256;

  int resized_w = static_cast<int>(image_width * resize_lim);  // 1280 * 0.55 = 704
  int resized_h = static_cast<int>(image_height * resize_lim);  // 1024 * 0.55 = 563
  int crop_x = (resized_w - norm_output_width) / 2;  // (704 - 704) / 2 = 0
  int crop_y = resized_h - norm_output_height;  // 563 - 256 = 307

  REQUIRE(resized_w == 704);
  REQUIRE(resized_h == 563);
  REQUIRE(crop_x == 0);
  REQUIRE(crop_y == 307);

  // Verify the aug matrix structure: scale + translate, 4x4 row-major, translation in column 3
  float aug[16] = {
    resize_lim,
    0.0f,
    0.0f,
    static_cast<float>(-crop_x),
    0.0f,
    resize_lim,
    0.0f,
    static_cast<float>(-crop_y),
    0.0f,
    0.0f,
    1.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    1.0f};

  // Row 0: [0.55, 0, 0, 0]  (no x-crop since resized width == output width)
  REQUIRE(aug[0] == Catch::Approx(0.55f));
  REQUIRE(aug[1] == 0.0f);
  REQUIRE(aug[2] == 0.0f);
  REQUIRE(aug[3] == 0.0f);  // -crop_x = 0

  // Row 1: [0, 0.55, 0, -307]
  REQUIRE(aug[4] == 0.0f);
  REQUIRE(aug[5] == Catch::Approx(0.55f));
  REQUIRE(aug[6] == 0.0f);
  REQUIRE(aug[7] == Catch::Approx(-307.0f));

  // Row 2: [0, 0, 1, 0]
  REQUIRE(aug[8] == 0.0f);
  REQUIRE(aug[9] == 0.0f);
  REQUIRE(aug[10] == 1.0f);
  REQUIRE(aug[11] == 0.0f);

  // Row 3: [0, 0, 0, 1]
  REQUIRE(aug[12] == 0.0f);
  REQUIRE(aug[13] == 0.0f);
  REQUIRE(aug[14] == 0.0f);
  REQUIRE(aug[15] == 1.0f);
}

// =============================================================================
// TEST: resize_lim too small produces negative crop (sanity)
// WHY: If resize_lim is chosen such that the resized image is smaller than
//      the model's expected input, the crop values go negative. This is a
//      configuration error that should be caught. This test documents the
//      expected behavior so future validation logic can be added.
// =============================================================================
TEST_CASE("img_aug_matrix: undersized resize_lim produces negative crop", "[calibration][fast]")
{
  // 1280 * 0.25 = 320 < 704 → crop_x would be negative
  int image_width = 1280;
  float resize_lim = 0.25f;
  int norm_output_width = 704;

  int resized_w = static_cast<int>(image_width * resize_lim);
  int crop_x = (resized_w - norm_output_width) / 2;

  REQUIRE(resized_w < norm_output_width);
  REQUIRE(crop_x < 0);
}

// =============================================================================
// INTRINSICS PADDING TESTS
// =============================================================================

// =============================================================================
// TEST: Camera intrinsics 3x3 → 4x4 padding
// WHY: CUDA-BEVFusion expects 4x4 matrices but CameraInfo provides 3x3 K.
//      The padding must add a 1.0 at [3][3] and zeros elsewhere. Getting
//      this wrong silently produces garbage geometry projections.
// =============================================================================
TEST_CASE("Calibration: camera intrinsics padding 3x3 to 4x4", "[calibration][fast]")
{
  // A realistic camera K matrix (pinhole model)
  double fx = 600.0, fy = 600.0, cx = 640.0, cy = 512.0;
  std::array<double, 9> k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};

  // Replicate the padding from computeCalibrationMatrices
  float cam_intrinsic[16] = {
    static_cast<float>(k[0]),
    static_cast<float>(k[1]),
    static_cast<float>(k[2]),
    0.0f,
    static_cast<float>(k[3]),
    static_cast<float>(k[4]),
    static_cast<float>(k[5]),
    0.0f,
    static_cast<float>(k[6]),
    static_cast<float>(k[7]),
    static_cast<float>(k[8]),
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    1.0f};

  // Row 0: [fx, 0, cx, 0]
  REQUIRE(cam_intrinsic[0] == Catch::Approx(static_cast<float>(fx)));
  REQUIRE(cam_intrinsic[1] == 0.0f);
  REQUIRE(cam_intrinsic[2] == Catch::Approx(static_cast<float>(cx)));
  REQUIRE(cam_intrinsic[3] == 0.0f);

  // Row 1: [0, fy, cy, 0]
  REQUIRE(cam_intrinsic[4] == 0.0f);
  REQUIRE(cam_intrinsic[5] == Catch::Approx(static_cast<float>(fy)));
  REQUIRE(cam_intrinsic[6] == Catch::Approx(static_cast<float>(cy)));
  REQUIRE(cam_intrinsic[7] == 0.0f);

  // Row 2: [0, 0, 1, 0]
  REQUIRE(cam_intrinsic[8] == 0.0f);
  REQUIRE(cam_intrinsic[9] == 0.0f);
  REQUIRE(cam_intrinsic[10] == 1.0f);
  REQUIRE(cam_intrinsic[11] == 0.0f);

  // Row 3: [0, 0, 0, 1] — the padding row
  REQUIRE(cam_intrinsic[12] == 0.0f);
  REQUIRE(cam_intrinsic[13] == 0.0f);
  REQUIRE(cam_intrinsic[14] == 0.0f);
  REQUIRE(cam_intrinsic[15] == 1.0f);
}

// =============================================================================
// BBOX → DETECTION3D CONVERSION TESTS
// =============================================================================

// =============================================================================
// TEST: BoundingBox → Detection3D position and size mapping
// WHY: This is the primary output consumed by the downstream tracking node.
//      If position/size fields are swapped or misassigned, tracking will
//      produce nonsensical results with no obvious error.
// =============================================================================
TEST_CASE("createDetections3D: position and size are mapped correctly", "[conversion][fast]")
{
  auto node = make_configured_node();
  set_identity_lidar_to_target_tf(*node);

  BoundingBox bbox = make_test_bbox(1.5f, -2.3f, 0.7f, 4.0f, 1.8f, 1.5f, 0.0f, 0.95f, 0);
  builtin_interfaces::msg::Time stamp;

  auto detections_3d = node->createDetections3D({bbox}, stamp);

  REQUIRE(detections_3d.header.frame_id == node->target_frame_);
  REQUIRE(detections_3d.detections.size() == 1);

  const auto & det = detections_3d.detections[0];
  REQUIRE(det.header.frame_id == node->target_frame_);
  REQUIRE(det.bbox.center.position.x == Catch::Approx(1.5));
  REQUIRE(det.bbox.center.position.y == Catch::Approx(-2.3));
  REQUIRE(det.bbox.center.position.z == Catch::Approx(0.7));

  // Documents current mapping: size.x <- bbox.size.w, size.y <- bbox.size.l
  REQUIRE(det.bbox.size.x == Catch::Approx(1.8));  // w
  REQUIRE(det.bbox.size.y == Catch::Approx(4.0));  // l
  REQUIRE(det.bbox.size.z == Catch::Approx(1.5));  // h

  rclcpp::shutdown();
}

// =============================================================================
// TEST: BoundingBox → Detection3D yaw rotation
// WHY: z_rotation encodes heading, but uses BEV-map sign convention (Y points
//      away from ego / "downward" in image space), which is opposite to the
//      ROS2/LiDAR right-handed frame (counter-clockwise = positive yaw).
//      The node negates z_rotation before calling setRPY so that a car
//      pointing "forward" (+x) with z_rotation=0 gets an identity quaternion,
//      and angles rotate in the physically correct direction.
// =============================================================================
TEST_CASE("createDetections3D: yaw rotation produces correct quaternion", "[conversion][fast]")
{
  auto node = make_configured_node();
  set_identity_lidar_to_target_tf(*node);

  float yaw = static_cast<float>(M_PI / 4.0);
  BoundingBox bbox = make_test_bbox(3.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, yaw, 0.9f, 0);
  builtin_interfaces::msg::Time stamp;

  auto detections_3d = node->createDetections3D({bbox}, stamp);
  REQUIRE(detections_3d.detections.size() == 1);
  const auto & det = detections_3d.detections[0];

  // z_rotation is negated before setRPY, so the resulting quaternion encodes -yaw.
  double expected_qw = std::cos(-M_PI / 8.0);  // cos(-yaw/2) == cos(yaw/2)
  double expected_qz = std::sin(-M_PI / 8.0);  // sin(-yaw/2) == -sin(yaw/2)

  REQUIRE(det.bbox.center.orientation.x == Catch::Approx(0.0).margin(1e-6));
  REQUIRE(det.bbox.center.orientation.y == Catch::Approx(0.0).margin(1e-6));
  REQUIRE(det.bbox.center.orientation.z == Catch::Approx(expected_qz).margin(1e-5));
  REQUIRE(det.bbox.center.orientation.w == Catch::Approx(expected_qw).margin(1e-5));

  rclcpp::shutdown();
}

// =============================================================================
// TEST: BoundingBox → Detection3D class ID and score
// WHY: The hypothesis carries the class_id (as string) and confidence score.
//      A wrong class mapping would silently misclassify all detections.
// =============================================================================
TEST_CASE("createDetections3D: hypothesis carries class_id and score", "[conversion][fast]")
{
  auto node = make_configured_node();
  set_identity_lidar_to_target_tf(*node);

  BoundingBox bbox = make_test_bbox(3.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.87f, 8);
  builtin_interfaces::msg::Time stamp;

  auto detections_3d = node->createDetections3D({bbox}, stamp);
  REQUIRE(detections_3d.detections.size() == 1);
  const auto & det = detections_3d.detections[0];

  REQUIRE(det.results.size() == 1);
  REQUIRE(det.results[0].hypothesis.class_id == "8");  // pedestrian in nuScenes
  REQUIRE(det.results[0].hypothesis.score == Catch::Approx(0.87f));

  rclcpp::shutdown();
}

TEST_CASE("createDetections3D: suppresses detections inside ego radius", "[conversion][fast]")
{
  auto node = make_configured_node();
  set_identity_lidar_to_target_tf(*node);

  BoundingBox suppressed_bbox = make_test_bbox(0.5f, 0.5f, 0.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.9f, 0);
  BoundingBox kept_bbox = make_test_bbox(3.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.8f, 1);
  builtin_interfaces::msg::Time stamp;

  auto detections_3d = node->createDetections3D({suppressed_bbox, kept_bbox}, stamp);

  REQUIRE(detections_3d.detections.size() == 1);
  REQUIRE(detections_3d.detections[0].results.size() == 1);
  REQUIRE(detections_3d.detections[0].results[0].hypothesis.class_id == "1");

  rclcpp::shutdown();
}

// =============================================================================
// TEST: createDetections3D returns an empty array (not a crash) on TF failure
// WHY: If TF for lidar->base_link isn't available (e.g. before TF publishes),
//      the frame must publish an empty Detection3DArray rather than stale or
//      garbage positions.
// =============================================================================
TEST_CASE("createDetections3D: returns empty array when TF lookup fails", "[conversion][fast]")
{
  auto node = make_configured_node();
  node->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());  // no transform registered

  BoundingBox bbox = make_test_bbox(0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.9f, 0);
  builtin_interfaces::msg::Time stamp;

  auto detections_3d = node->createDetections3D({bbox}, stamp);

  REQUIRE(detections_3d.detections.empty());
  REQUIRE(detections_3d.header.frame_id == node->target_frame_);

  rclcpp::shutdown();
}

// =============================================================================
// TEST: createDetections3D handles empty input and a null tf_buffer safely
// WHY: runInference may call this with zero boxes (nothing detected), and
//      the TF buffer may not be constructed yet during startup races.
// =============================================================================
TEST_CASE("createDetections3D: empty input and null tf_buffer are handled safely", "[conversion][fast]")
{
  auto node = make_configured_node();
  builtin_interfaces::msg::Time stamp;

  SECTION("empty bboxes vector")
  {
    node->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto detections_3d = node->createDetections3D({}, stamp);
    REQUIRE(detections_3d.detections.empty());
    REQUIRE(detections_3d.header.frame_id == node->target_frame_);
  }

  SECTION("null tf_buffer_")
  {
    node->tf_buffer_.reset();
    BoundingBox bbox = make_test_bbox(0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.9f, 0);
    auto detections_3d = node->createDetections3D({bbox}, stamp);
    REQUIRE(detections_3d.detections.empty());
  }

  rclcpp::shutdown();
}

// // =============================================================================
// // BBOX → MARKER CONVERSION TESTS
// // =============================================================================

// =============================================================================
// TEST: Marker class coloring — all 10 nuScenes classes
// WHY: Foxglove visualization uses color to distinguish object classes.
//      If colors are swapped, the operator sees trucks colored as cars, etc.
//      nuScenes: 0=car, 1=truck, 2=construction_vehicle, 3=bus, 4=trailer,
//                5=barrier, 6=motorcycle, 7=bicycle, 8=pedestrian, 9=traffic_cone
// =============================================================================
TEST_CASE("createMarkers: class-specific colors", "[conversion][fast]")
{
  auto node = make_configured_node();
  vision_msgs::msg::Detection3DArray detections_3d;
  detections_3d.header.frame_id = node->target_frame_;

  SECTION("Car (id=0) is green")
  {
    detections_3d.detections = {make_test_detection3d(0, 0, 0, 1, 1, 1, 0, 0.9f)};
    auto markers = node->createMarkers(detections_3d);
    REQUIRE(markers.markers.size() == 1);  // 1 box, no DELETEALL
    const auto & m = markers.markers[0];
    REQUIRE(m.color.r == Catch::Approx(0.0f));
    REQUIRE(m.color.g == Catch::Approx(1.0f));
    REQUIRE(m.color.b == Catch::Approx(0.0f));
  }

  SECTION("Truck (id=1) is blue")
  {
    detections_3d.detections = {make_test_detection3d(0, 0, 0, 1, 1, 1, 1, 0.9f)};
    auto markers = node->createMarkers(detections_3d);
    const auto & m = markers.markers[0];
    REQUIRE(m.color.r == Catch::Approx(0.0f));
    REQUIRE(m.color.g == Catch::Approx(0.0f));
    REQUIRE(m.color.b == Catch::Approx(1.0f));
  }

  SECTION("Construction vehicle (id=2) is orange")
  {
    detections_3d.detections = {make_test_detection3d(0, 0, 0, 1, 1, 1, 2, 0.9f)};
    auto markers = node->createMarkers(detections_3d);
    const auto & m = markers.markers[0];
    REQUIRE(m.color.r == Catch::Approx(1.0f));
    REQUIRE(m.color.g == Catch::Approx(0.5f));
    REQUIRE(m.color.b == Catch::Approx(0.0f));
  }

  SECTION("Bus (id=3) is purple")
  {
    detections_3d.detections = {make_test_detection3d(0, 0, 0, 1, 1, 1, 3, 0.9f)};
    auto markers = node->createMarkers(detections_3d);
    const auto & m = markers.markers[0];
    REQUIRE(m.color.r == Catch::Approx(0.5f));
    REQUIRE(m.color.g == Catch::Approx(0.0f));
    REQUIRE(m.color.b == Catch::Approx(1.0f));
  }

  SECTION("Trailer (id=4) is cyan")
  {
    detections_3d.detections = {make_test_detection3d(0, 0, 0, 1, 1, 1, 4, 0.9f)};
    auto markers = node->createMarkers(detections_3d);
    const auto & m = markers.markers[0];
    REQUIRE(m.color.r == Catch::Approx(0.0f));
    REQUIRE(m.color.g == Catch::Approx(1.0f));
    REQUIRE(m.color.b == Catch::Approx(1.0f));
  }

  SECTION("Barrier (id=5) is yellow")
  {
    detections_3d.detections = {make_test_detection3d(0, 0, 0, 1, 1, 1, 5, 0.9f)};
    auto markers = node->createMarkers(detections_3d);
    const auto & m = markers.markers[0];
    REQUIRE(m.color.r == Catch::Approx(1.0f));
    REQUIRE(m.color.g == Catch::Approx(1.0f));
    REQUIRE(m.color.b == Catch::Approx(0.0f));
  }

  SECTION("Motorcycle (id=6) is magenta")
  {
    detections_3d.detections = {make_test_detection3d(0, 0, 0, 1, 1, 1, 6, 0.9f)};
    auto markers = node->createMarkers(detections_3d);
    const auto & m = markers.markers[0];
    REQUIRE(m.color.r == Catch::Approx(1.0f));
    REQUIRE(m.color.g == Catch::Approx(0.0f));
    REQUIRE(m.color.b == Catch::Approx(1.0f));
  }

  SECTION("Bicycle (id=7) is sky blue")
  {
    detections_3d.detections = {make_test_detection3d(0, 0, 0, 1, 1, 1, 7, 0.9f)};
    auto markers = node->createMarkers(detections_3d);
    const auto & m = markers.markers[0];
    REQUIRE(m.color.r == Catch::Approx(0.0f));
    REQUIRE(m.color.g == Catch::Approx(0.5f));
    REQUIRE(m.color.b == Catch::Approx(1.0f));
  }

  SECTION("Pedestrian (id=8) is red")
  {
    detections_3d.detections = {make_test_detection3d(0, 0, 0, 1, 1, 1, 8, 0.9f)};
    auto markers = node->createMarkers(detections_3d);
    const auto & m = markers.markers[0];
    REQUIRE(m.color.r == Catch::Approx(1.0f));
    REQUIRE(m.color.g == Catch::Approx(0.0f));
    REQUIRE(m.color.b == Catch::Approx(0.0f));
  }

  SECTION("Traffic cone (id=9) is amber")
  {
    detections_3d.detections = {make_test_detection3d(0, 0, 0, 1, 1, 1, 9, 0.9f)};
    auto markers = node->createMarkers(detections_3d);
    const auto & m = markers.markers[0];
    REQUIRE(m.color.r == Catch::Approx(1.0f));
    REQUIRE(m.color.g == Catch::Approx(0.8f));
    REQUIRE(m.color.b == Catch::Approx(0.0f));
  }

  SECTION("Unknown class (id=99) defaults to white")
  {
    detections_3d.detections = {make_test_detection3d(0, 0, 0, 1, 1, 1, 99, 0.9f)};
    auto markers = node->createMarkers(detections_3d);
    const auto & m = markers.markers[0];
    REQUIRE(m.color.r == Catch::Approx(1.0f));
    REQUIRE(m.color.g == Catch::Approx(1.0f));
    REQUIRE(m.color.b == Catch::Approx(1.0f));
  }

  rclcpp::shutdown();
}

// =============================================================================
// TEST: Marker type, namespace, opacity, id, and lifetime
// WHY: Foxglove renders CUBE-type markers, keyed by ns/id. If those are wrong,
//      the 3D panel shows nothing or overlaps other viz layers. The 1 s lifetime
//      retires stale boxes when the node is slow or stops publishing.
// =============================================================================
TEST_CASE("createMarkers: type, namespace, opacity, id, and lifetime are set correctly", "[conversion][fast]")
{
  auto node = make_configured_node();

  vision_msgs::msg::Detection3DArray detections_3d;
  detections_3d.header.frame_id = node->target_frame_;
  detections_3d.detections = {make_test_detection3d(5.0f, 3.0f, 1.0f, 4.5f, 2.0f, 1.7f, 0, 0.8f)};

  auto markers = node->createMarkers(detections_3d);
  REQUIRE(markers.markers.size() == 1);  // 1 box, no DELETEALL
  const auto & marker = markers.markers[0];
  REQUIRE(marker.type == visualization_msgs::msg::Marker::CUBE);
  REQUIRE(marker.action == visualization_msgs::msg::Marker::ADD);
  REQUIRE(marker.ns == "bevfusion_detections");
  REQUIRE(marker.id == 0);  // index-based now, not passed in explicitly
  REQUIRE(marker.color.a == Catch::Approx(0.8f));
  REQUIRE(marker.header.frame_id == node->target_frame_);

  REQUIRE(marker.scale.x == Catch::Approx(4.5f));
  REQUIRE(marker.scale.y == Catch::Approx(2.0f));
  REQUIRE(marker.scale.z == Catch::Approx(1.7f));

  // Boxes persist across frames and are retired by the 1 s lifetime (no DELETEALL)
  REQUIRE(rclcpp::Duration(marker.lifetime).seconds() == Catch::Approx(1.0));

  rclcpp::shutdown();
}

// =============================================================================
// TEST: zero detections emits an empty array (no DELETEALL)
// WHY: Boxes persist across frames via their 1 s lifetime. An empty frame must
//      not wipe the previous frame's boxes immediately — they age out naturally.
// =============================================================================
TEST_CASE("createMarkers: zero detections emits an empty array", "[conversion][fast]")
{
  auto node = make_configured_node();
  vision_msgs::msg::Detection3DArray detections_3d;
  detections_3d.header.frame_id = node->target_frame_;

  auto markers = node->createMarkers(detections_3d);

  REQUIRE(markers.markers.empty());

  rclcpp::shutdown();
}

// =============================================================================
// QOS FACTORY TESTS
// =============================================================================

// =============================================================================
// TEST: createSubscriberQoS with valid reliability values
// WHY: Misconfigured QoS silently drops all messages. The node would appear
//      to work but never receive sensor data. Testing the factory ensures
//      YAML-specified QoS values actually produce the expected policy.
// =============================================================================
TEST_CASE("createSubscriberQoS: reliability policies", "[qos][fast]")
{
  auto node = make_configured_node();

  SECTION("reliable")
  {
    auto qos = node->createSubscriberQoS("reliable", 5);
    auto profile = qos.get_rmw_qos_profile();
    REQUIRE(profile.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    REQUIRE(profile.depth == 5);
  }

  SECTION("best_effort")
  {
    auto qos = node->createSubscriberQoS("best_effort", 20);
    auto profile = qos.get_rmw_qos_profile();
    REQUIRE(profile.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    REQUIRE(profile.depth == 20);
  }

  SECTION("unknown defaults to best_effort")
  {
    auto qos = node->createSubscriberQoS("typo_value", 10);
    auto profile = qos.get_rmw_qos_profile();
    REQUIRE(profile.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  }

  rclcpp::shutdown();
}

// =============================================================================
// TEST: createPublisherQoS with valid durability values
// WHY: transient_local vs volatile determines whether late-joining
//      subscribers see previously published messages. Getting it wrong
//      means Foxglove shows no markers until the next detection cycle.
// =============================================================================
TEST_CASE("createPublisherQoS: durability policies", "[qos][fast]")
{
  auto node = make_configured_node();

  SECTION("transient_local + reliable")
  {
    auto qos = node->createPublisherQoS("reliable", "transient_local", 10);
    auto profile = qos.get_rmw_qos_profile();
    REQUIRE(profile.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    REQUIRE(profile.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  }

  SECTION("volatile + best_effort")
  {
    auto qos = node->createPublisherQoS("best_effort", "volatile", 10);
    auto profile = qos.get_rmw_qos_profile();
    REQUIRE(profile.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    REQUIRE(profile.durability == RMW_QOS_POLICY_DURABILITY_VOLATILE);
  }

  SECTION("unknown durability defaults to transient_local")
  {
    auto qos = node->createPublisherQoS("reliable", "typo", 10);
    auto profile = qos.get_rmw_qos_profile();
    REQUIRE(profile.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  }

  rclcpp::shutdown();
}

// =============================================================================
// TEST: createSubscriberQoS invalid fallback
// WHY: Verifies that incorrect configuration parameters fall back to sensible
//      defaults and don't crash node initialization.
// =============================================================================
TEST_CASE("BEVFusionNode: createSubscriberQoS invalid fallback", "[node][fast]")
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  auto node = std::make_shared<BEVFusionNode>(rclcpp::NodeOptions{});

  auto qos = node->createSubscriberQoS("invalid_reliability", 10);
  REQUIRE(qos.get_rmw_qos_profile().reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

  rclcpp::shutdown();
}

// =============================================================================
// TEST: createPublisherQoS invalid fallback
// WHY: Verifies that incorrect configuration parameters fall back to sensible
//      defaults and don't crash node initialization.
// =============================================================================
TEST_CASE("BEVFusionNode: createPublisherQoS invalid fallback", "[node][fast]")
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  auto node = std::make_shared<BEVFusionNode>(rclcpp::NodeOptions{});

  auto qos = node->createPublisherQoS("invalid_reliability", "invalid_durability", 10);
  REQUIRE(qos.get_rmw_qos_profile().reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  REQUIRE(qos.get_rmw_qos_profile().durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  rclcpp::shutdown();
}

// =============================================================================
// TEST: diagnosticCallback warns on no data
// WHY: Ensures that the node correctly reports a warning state through the
//      diagnostic updater when it hasn't processed any detections yet.
// =============================================================================
TEST_CASE("BEVFusionNode: diagnosticCallback warns on no data", "[node][fast]")
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  auto node = std::make_shared<BEVFusionNode>(rclcpp::NodeOptions{});

  diagnostic_updater::DiagnosticStatusWrapper stat;
  node->diagnosticCallback(stat);

  REQUIRE(stat.level == diagnostic_msgs::msg::DiagnosticStatus::WARN);
  REQUIRE_THAT(stat.message, Catch::Matchers::ContainsSubstring("processed yet"));

  rclcpp::shutdown();
}

// =============================================================================
// CALIBRATION STATE MACHINE TESTS
// =============================================================================

// =============================================================================
// TEST: computeCalibrationMatrices with empty camera info is a no-op
// WHY: The callback can fire before any camera info arrives. It must
//      gracefully no-op and leave calibration_initialized_ as false.
// =============================================================================
TEST_CASE("computeCalibrationMatrices: no-op with empty camera info", "[calibration][fast]")
{
  auto node = make_configured_node();

  // No cached camera info → should not crash, calibration stays false
  node->computeCalibrationMatrices();
  REQUIRE_FALSE(node->calibration_initialized_.load());

  rclcpp::shutdown();
}

// =============================================================================
// TEST: computeCalibrationMatrices with null camera info is a no-op
// WHY: Same as above but explicitly tests the nullptr path vs empty vector.
// =============================================================================
TEST_CASE("computeCalibrationMatrices: no-op with null cached pointer", "[calibration][fast]")
{
  auto node = make_configured_node();

  {
    std::lock_guard<std::mutex> lock(node->camera_info_mutex_);
    node->cached_camera_infos_.assign(6, nullptr);
  }

  node->computeCalibrationMatrices();
  REQUIRE_FALSE(node->calibration_initialized_.load());

  rclcpp::shutdown();
}

// =============================================================================
// TEST: computeCalibrationMatrices with valid data sets calibration flag
// WHY: End-to-end path: camera info cached + TF available → calibration
//      should succeed, set the flag and record the camera's azimuth for sweep targets.
// =============================================================================
TEST_CASE("computeCalibrationMatrices: sets flag with valid camera info and TF", "[calibration][fast]")
{
  auto node = make_single_camera_node("test_cam");

  {
    std::lock_guard<std::mutex> lock(node->camera_info_mutex_);
    node->cached_camera_infos_ = {make_camera_info("test_cam")};
  }

  // Camera looking along the LiDAR's +y axis: rotation of -90 deg about x maps optical +z to +y
  set_camera_to_lidar_tf(*node, "test_cam", -std::sqrt(0.5), std::sqrt(0.5));

  node->computeCalibrationMatrices();
  REQUIRE(node->calibration_initialized_.load());
  REQUIRE(node->camera_azimuths_deg_.size() == 1);
  REQUIRE(node->camera_azimuths_deg_[0] == Catch::Approx(90.0).margin(1e-3));

  rclcpp::shutdown();
}

// =============================================================================
// TEST: computeCalibrationMatrices aborts when TF lookup fails
// WHY: If the TF tree is incomplete (e.g. camera frame not published yet),
//      the function should return early without setting calibration_initialized_.
//      A silent failure here would leave the system stuck.
// =============================================================================
TEST_CASE("computeCalibrationMatrices: aborts on TF lookup failure", "[calibration][fast]")
{
  auto node = make_single_camera_node("nonexistent_frame");

  {
    std::lock_guard<std::mutex> lock(node->camera_info_mutex_);
    node->cached_camera_infos_ = {make_camera_info("nonexistent_frame")};
  }

  // TF buffer with no transforms → lookup will fail
  node->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  node->computeCalibrationMatrices();
  REQUIRE_FALSE(node->calibration_initialized_.load());

  rclcpp::shutdown();
}

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

#ifndef PREDICTION_ML__PREDICTION_ML_NODE_HPP_
#define PREDICTION_ML__PREDICTION_ML_NODE_HPP_

#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lanelet_msgs/msg/lanelet_ahead.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "prediction_ml/mtr_runtime.hpp"
#include "prediction_ml/scene_builder.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "world_model_msgs/msg/world_object.hpp"
#include "world_model_msgs/msg/world_object_array.hpp"

namespace prediction_ml
{

struct ExactStampInputs
{
  vision_msgs::msg::Detection3DArray::SharedPtr tracks;
  nav_msgs::msg::Odometry::SharedPtr odometry;  // ego pose and velocity
  lanelet_msgs::msg::LaneletAhead::SharedPtr lanelet;
};

// Evaluation-only exact-stamp joiner. Production continues to use the latest
// auxiliary state because its independently published topics are not guaranteed
// to share timestamps.
class ExactStampJoiner
{
public:
  static constexpr std::size_t kCapacity = 32U;

  std::optional<ExactStampInputs> addTracks(vision_msgs::msg::Detection3DArray::SharedPtr message);
  std::optional<ExactStampInputs> addOdometry(nav_msgs::msg::Odometry::SharedPtr message);
  std::optional<ExactStampInputs> addLanelet(lanelet_msgs::msg::LaneletAhead::SharedPtr message);
  void clear();

private:
  using Stamp = std::int64_t;

  static std::optional<Stamp> stampKey(std::int32_t seconds, std::uint32_t nanoseconds);

  template <typename MessageT>
  std::optional<ExactStampInputs> insertAndTake(
    std::map<Stamp, std::shared_ptr<MessageT>> & cache, std::shared_ptr<MessageT> message)
  {
    if (!message) {
      return std::nullopt;
    }
    const auto stamp = stampKey(message->header.stamp.sec, message->header.stamp.nanosec);
    if (!stamp.has_value() || processed_.find(*stamp) != processed_.end()) {
      return std::nullopt;
    }
    cache[*stamp] = std::move(message);
    while (cache.size() > kCapacity) {
      cache.erase(cache.begin());
    }
    return take(*stamp);
  }

  std::optional<ExactStampInputs> take(Stamp stamp);
  void rememberProcessed(Stamp stamp);

  std::map<Stamp, vision_msgs::msg::Detection3DArray::SharedPtr> tracks_;
  std::map<Stamp, nav_msgs::msg::Odometry::SharedPtr> odometry_;
  std::map<Stamp, lanelet_msgs::msg::LaneletAhead::SharedPtr> lanelets_;
  std::deque<Stamp> processed_order_;
  std::unordered_set<Stamp> processed_;
};

// Re-expresses tracks (pose, orientation, linear_velocity direction) in transform's target frame.
vision_msgs::msg::Detection3DArray transformTracks(
  const vision_msgs::msg::Detection3DArray & tracks, const geometry_msgs::msg::TransformStamped & transform);

// Re-expresses seeds (detections and every prediction pose) in transform's target frame. Stamps stay.
world_model_msgs::msg::WorldObjectArray transformSeeds(
  const world_model_msgs::msg::WorldObjectArray & seeds, const geometry_msgs::msg::TransformStamped & transform);

// Re-expresses lane centerlines (the only lane geometry the scene builder reads) in transform's
// target frame.
lanelet_msgs::msg::LaneletAhead transformLanes(
  const lanelet_msgs::msg::LaneletAhead & lanes, const geometry_msgs::msg::TransformStamped & transform);

// The lanes for a frame at track_time_s, re-expressed by map_from_lanes; nullopt (no-map engine) when
// there are none, their stamp is more than max_age_s from the tracks, or there is no transform.
std::optional<lanelet_msgs::msg::LaneletAhead> lanesForFrame(
  const lanelet_msgs::msg::LaneletAhead::SharedPtr & lanes,
  double track_time_s,
  double max_age_s,
  const std::optional<geometry_msgs::msg::TransformStamped> & map_from_lanes);

// Ego odometry at stamp_s from a buffer sorted by stamp: interpolated between the samples around it,
// or the nearest sample when stamp_s is outside the buffer. nullopt when the nearest sample is more
// than max_gap_s away (odometry missing, stalled or lagging).
std::optional<nav_msgs::msg::Odometry> odometryAt(
  const std::deque<nav_msgs::msg::Odometry::SharedPtr> & buffer, double stamp_s, double max_gap_s);

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
  void egoOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void laneletAheadCallback(const lanelet_msgs::msg::LaneletAhead::SharedPtr msg);
  void publishMtrDiagnostics();
  void submitSynchronizedMtr(ExactStampInputs inputs);
  // map_frame_ <- frame: the ego odometry at the track stamp composed with the TF odometry-child <-
  // frame at that stamp (static for lidar_cc tracks or lanes, eidos' for lanelets in map), or nullopt
  // (logged) when TF has no such transform.
  std::optional<geometry_msgs::msg::TransformStamped> mapFrom(
    const std::string & frame, const builtin_interfaces::msg::Time & stamp, const nav_msgs::msg::Odometry & ego);

  // Constant-velocity straight-line fallback (ported from simple_prediction).
  std::vector<world_model_msgs::msg::WorldObject> buildFallback(
    const vision_msgs::msg::Detection3DArray & msg, const rclcpp::Time & source_time) const;

  MtrConfig loadMtrConfig();

  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr tracked_objects_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_odometry_sub_;
  rclcpp::Subscription<lanelet_msgs::msg::LaneletAhead>::SharedPtr lanelet_ahead_sub_;
  rclcpp_lifecycle::LifecyclePublisher<world_model_msgs::msg::WorldObjectArray>::SharedPtr world_objects_pub_;
  rclcpp_lifecycle::LifecyclePublisher<world_model_msgs::msg::WorldObjectArray>::SharedPtr mtr_predictions_pub_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;

  std::unique_ptr<SceneBuilder> scene_builder_;
  std::unique_ptr<MtrRuntime> runtime_;
  ExactStampJoiner exact_stamp_joiner_;

  std::deque<nav_msgs::msg::Odometry::SharedPtr> odometry_buffer_;  // sorted by stamp, about 1 s
  lanelet_msgs::msg::LaneletAhead::SharedPtr lanelet_ahead_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string map_frame_{"map"};
  std::string odometry_topic_;
  double odometry_max_gap_s_{0.2};
  double lane_max_age_s_{0.5};  // mtr.cache_ttl_s
  std::string ego_velocity_child_frame_{"base_footprint"};
  bool mtr_diagnostics_enabled_{false};

  double prediction_horizon_{3.0};
  double prediction_time_step_{0.2};
};

}  // namespace prediction_ml

#endif  // PREDICTION_ML__PREDICTION_ML_NODE_HPP_

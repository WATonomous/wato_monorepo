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

#ifndef PREDICTION__PREDICTION_NODE_HPP_
#define PREDICTION__PREDICTION_NODE_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lanelet_msgs/msg/lanelet_ahead.hpp"
#include "lanelet_msgs/srv/get_lanelet_ahead.hpp"
#include "prediction/intent_classifier.hpp"
#include "prediction/trajectory_predictor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "world_model_msgs/msg/world_object.hpp"
#include "world_model_msgs/msg/world_object_array.hpp"

namespace prediction {

class PredictionNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit PredictionNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~PredictionNode() override = default;

protected:
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

private:
  void trackedObjectsCallback(
      const vision_msgs::msg::Detection3DArray::SharedPtr msg);
  void egoPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void
  laneletAheadCallback(const lanelet_msgs::msg::LaneletAhead::SharedPtr msg);

  // Temporal confidence smoothing to reduce frame-to-frame flicker.
  void applyConfidenceSmoothing(const std::string &object_id,
                                std::vector<TrajectoryHypothesis> &hypotheses);
  void pruneConfidenceHistory(const rclcpp::Time &now);

  struct SmoothedHypothesisState {
    Intent intent;
    double end_x;
    double end_y;
    double confidence;
  };

  struct SmoothedObjectState {
    std::vector<SmoothedHypothesisState> hypotheses;
    rclcpp::Time last_update;
  };

  // Subscribers
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr
      tracked_objects_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      ego_pose_sub_;
  rclcpp::Subscription<lanelet_msgs::msg::LaneletAhead>::SharedPtr
      lanelet_ahead_sub_;

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<
      world_model_msgs::msg::WorldObjectArray>::SharedPtr world_objects_pub_;

  // Core components
  std::unique_ptr<TrajectoryPredictor> trajectory_predictor_;
  std::unique_ptr<IntentClassifier> intent_classifier_;

  // Service client for per-vehicle lanelet queries
  rclcpp::Client<lanelet_msgs::srv::GetLaneletAhead>::SharedPtr
      lanelet_ahead_client_;

  // State
  geometry_msgs::msg::PoseStamped::SharedPtr ego_pose_;

  // Vehicle IDs with in-flight async lanelet requests (prevents duplicates)
  std::unordered_set<std::string> pending_vehicle_requests_;
  static constexpr size_t kMaxPendingRequests = 8;

  // Per-object history for confidence smoothing.
  std::unordered_map<std::string, SmoothedObjectState> confidence_history_;

  // Parameters
  double prediction_horizon_;
  double prediction_time_step_;
  double confidence_smoothing_alpha_;
  double confidence_match_distance_m_;
  double confidence_state_timeout_s_;
};

} // namespace prediction

#endif // PREDICTION__PREDICTION_NODE_HPP_

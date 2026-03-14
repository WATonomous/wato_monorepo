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

// Standard library headers for memory management and data structures
#include <memory>  // For std::unique_ptr and std::shared_ptr
#include <mutex>  // For std::mutex protecting async-callback shared state
#include <string>  // For std::string in object IDs and keys
#include <unordered_map>  // For storing per-object and per-vehicle state
#include <unordered_set>  // For tracking pending vehicle requests
#include <vector>  // For collections of hypotheses and objects

// ROS2 message types
#include "geometry_msgs/msg/pose_stamped.hpp"  // For ego vehicle pose
#include "lanelet_msgs/msg/lanelet_ahead.hpp"  // For reachable lanelets
#include "lanelet_msgs/srv/get_lanelet_ahead.hpp"  // For querying lanelets around vehicles
#include "vision_msgs/msg/detection3_d_array.hpp"  // For tracked object detections
#include "world_model_msgs/msg/world_object.hpp"  // For individual predicted objects
#include "world_model_msgs/msg/world_object_array.hpp"  // For predicted object arrays

// ROS2 core and lifecycle
#include "rclcpp/rclcpp.hpp"  // ROS2 C++ client library
#include "rclcpp_lifecycle/lifecycle_node.hpp"  // For lifecycle node management

// Project-specific headers
#include "prediction/intent_classifier.hpp"  // For maneuver intent detection
#include "prediction/trajectory_predictor.hpp"  // For trajectory hypothesis generation

namespace prediction
{

class PredictionNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit PredictionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PredictionNode() override = default;

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

  // Temporal confidence smoothing to reduce frame-to-frame flicker.
  void applyConfidenceSmoothing(
    const vision_msgs::msg::Detection3D & detection, std::vector<TrajectoryHypothesis> & hypotheses);
  void pruneConfidenceHistory(const rclcpp::Time & now);

  struct SmoothedHypothesisState
  {
    Intent intent;
    double end_x;
    double end_y;
    double confidence;
  };

  struct SmoothedObjectState
  {
    std::vector<SmoothedHypothesisState> hypotheses;
    bool has_observation{false};
    rclcpp::Time last_update;
  };

  // Subscribers
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr tracked_objects_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ego_pose_sub_;
  rclcpp::Subscription<lanelet_msgs::msg::LaneletAhead>::SharedPtr lanelet_ahead_sub_;

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<world_model_msgs::msg::WorldObjectArray>::SharedPtr world_objects_pub_;

  // Core components
  std::unique_ptr<TrajectoryPredictor> trajectory_predictor_;
  std::unique_ptr<IntentClassifier> intent_classifier_;

  // Service client for per-vehicle lanelet queries
  rclcpp::Client<lanelet_msgs::srv::GetLaneletAhead>::SharedPtr lanelet_ahead_client_;

  // Callback group: all subscription callbacks run in the same mutually-
  // exclusive group so they never race against each other. The async
  // service-response callback runs on a separate executor thread and
  // shares pending_vehicle_requests_ — protected by pending_requests_mutex_.
  rclcpp::CallbackGroup::SharedPtr subscription_cb_group_;

  // State
  geometry_msgs::msg::PoseStamped::SharedPtr ego_pose_;

  // Vehicle IDs with in-flight async lanelet requests (prevents duplicates).
  // Accessed from subscription callbacks AND async service-response callbacks,
  // so it is protected by pending_requests_mutex_.
  std::mutex pending_requests_mutex_;
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
  double cache_ttl_s_;  // TTL for all ID-keyed caches (TrajectoryPredictor + confidence)
  double lanelet_query_radius_;  // radius for per-vehicle lanelet service query
};

}  // namespace prediction

#endif  // PREDICTION__PREDICTION_NODE_HPP_

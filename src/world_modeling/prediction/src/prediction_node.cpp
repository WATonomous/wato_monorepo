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

#include "prediction/prediction_node.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

namespace prediction {

PredictionNode::PredictionNode(const rclcpp::NodeOptions &options)
    : LifecycleNode("prediction_node", options) {
  this->declare_parameter("prediction_horizon", 3.0);
  this->declare_parameter("prediction_time_step", 0.2);
  this->declare_parameter("confidence_smoothing_alpha", 0.35);
  this->declare_parameter("confidence_match_distance_m", 6.0);
  this->declare_parameter("confidence_state_timeout_s", 5.0);

  RCLCPP_INFO(this->get_logger(), "PredictionNode created (unconfigured)");
}

PredictionNode::CallbackReturn
PredictionNode::on_configure(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(this->get_logger(), "Configuring...");

  prediction_horizon_ = this->get_parameter("prediction_horizon").as_double();
  prediction_time_step_ =
      this->get_parameter("prediction_time_step").as_double();
  confidence_smoothing_alpha_ =
      this->get_parameter("confidence_smoothing_alpha").as_double();
  confidence_match_distance_m_ =
      this->get_parameter("confidence_match_distance_m").as_double();
  confidence_state_timeout_s_ =
      this->get_parameter("confidence_state_timeout_s").as_double();

  confidence_smoothing_alpha_ =
      std::clamp(confidence_smoothing_alpha_, 0.0, 1.0);
  confidence_match_distance_m_ = std::max(0.1, confidence_match_distance_m_);
  confidence_state_timeout_s_ = std::max(0.5, confidence_state_timeout_s_);

  RCLCPP_INFO(this->get_logger(), "Prediction horizon: %.2f seconds",
              prediction_horizon_);
  RCLCPP_INFO(this->get_logger(), "Prediction time step: %.2f seconds",
              prediction_time_step_);
  RCLCPP_INFO(this->get_logger(),
              "Confidence smoothing alpha: %.2f, match distance: %.1fm",
              confidence_smoothing_alpha_, confidence_match_distance_m_);

  world_objects_pub_ =
      this->create_publisher<world_model_msgs::msg::WorldObjectArray>(
          "world_object_seeds", 10);

  trajectory_predictor_ = std::make_unique<TrajectoryPredictor>(
      this, prediction_horizon_, prediction_time_step_);
  intent_classifier_ = std::make_unique<IntentClassifier>(this);
  lanelet_ahead_client_ =
      this->create_client<lanelet_msgs::srv::GetLaneletAhead>(
          "get_lanelet_ahead");

  // Wire the per-vehicle lanelet query as async fire-and-forget.
  // On cache miss, this fires an async service request and returns
  // immediately. The response callback populates the per-vehicle cache,
  // so the next prediction cycle will have lanelet data.
  trajectory_predictor_->setLaneletQueryFunction(
      [this](const std::string &vehicle_id,
             const geometry_msgs::msg::Point &position, double heading_rad) {
        if (!lanelet_ahead_client_ ||
            !lanelet_ahead_client_->service_is_ready()) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                               "GetLaneletAhead service not ready, "
                               "falling back to geometric prediction");
          return;
        }
        // Avoid duplicate in-flight requests for the same vehicle,
        // and cap total concurrent requests to avoid queue overflow
        if (pending_vehicle_requests_.count(vehicle_id) ||
            pending_vehicle_requests_.size() >= kMaxPendingRequests) {
          return;
        }
        pending_vehicle_requests_.insert(vehicle_id);

        auto request =
            std::make_shared<lanelet_msgs::srv::GetLaneletAhead::Request>();
        request->position = position;
        request->heading_rad = heading_rad;
        request->radius_m = 50.0;

        double vx = position.x, vy = position.y;
        lanelet_ahead_client_->async_send_request(
            request,
            [this, vehicle_id, vx, vy](
                rclcpp::Client<lanelet_msgs::srv::GetLaneletAhead>::SharedFuture
                    future) {
              pending_vehicle_requests_.erase(vehicle_id);
              auto response = future.get();
              if (response->success) {
                trajectory_predictor_->updateVehicleLaneletCache(
                    vehicle_id, response->lanelet_ahead, vx, vy);
              } else {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "GetLaneletAhead failed for vehicle '%s' at "
                    "(%.1f, %.1f): %s",
                    vehicle_id.c_str(), vx, vy,
                    response->error_message.c_str());
              }
            });
      });

  RCLCPP_INFO(this->get_logger(), "Configured successfully");
  return CallbackReturn::SUCCESS;
}

PredictionNode::CallbackReturn
PredictionNode::on_activate(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(this->get_logger(), "Activating...");

  tracked_objects_sub_ =
      this->create_subscription<vision_msgs::msg::Detection3DArray>(
          "tracks_3d", 10,
          std::bind(&PredictionNode::trackedObjectsCallback, this,
                    std::placeholders::_1));

  ego_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "ego_pose", 10,
      std::bind(&PredictionNode::egoPoseCallback, this, std::placeholders::_1));

  lanelet_ahead_sub_ =
      this->create_subscription<lanelet_msgs::msg::LaneletAhead>(
          "lanelet_ahead", 10,
          std::bind(&PredictionNode::laneletAheadCallback, this,
                    std::placeholders::_1));

  world_objects_pub_->on_activate();

  RCLCPP_INFO(this->get_logger(), "Activated successfully");
  return CallbackReturn::SUCCESS;
}

PredictionNode::CallbackReturn
PredictionNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(this->get_logger(), "Deactivating...");

  tracked_objects_sub_.reset();
  ego_pose_sub_.reset();
  lanelet_ahead_sub_.reset();
  world_objects_pub_->on_deactivate();

  RCLCPP_INFO(this->get_logger(), "Deactivated successfully");
  return CallbackReturn::SUCCESS;
}

PredictionNode::CallbackReturn
PredictionNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(this->get_logger(), "Cleaning up...");

  tracked_objects_sub_.reset();
  ego_pose_sub_.reset();
  lanelet_ahead_sub_.reset();
  world_objects_pub_.reset();

  trajectory_predictor_.reset();
  intent_classifier_.reset();

  ego_pose_.reset();
  pending_vehicle_requests_.clear();
  confidence_history_.clear();

  RCLCPP_INFO(this->get_logger(), "Cleaned up successfully");
  return CallbackReturn::SUCCESS;
}

PredictionNode::CallbackReturn
PredictionNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(this->get_logger(), "Shutting down...");

  tracked_objects_sub_.reset();
  ego_pose_sub_.reset();
  lanelet_ahead_sub_.reset();
  world_objects_pub_.reset();

  trajectory_predictor_.reset();
  intent_classifier_.reset();

  ego_pose_.reset();
  pending_vehicle_requests_.clear();
  confidence_history_.clear();

  return CallbackReturn::SUCCESS;
}

void PredictionNode::laneletAheadCallback(
    const lanelet_msgs::msg::LaneletAhead::SharedPtr msg) {
  trajectory_predictor_->setLaneletAhead(msg);
  RCLCPP_DEBUG(this->get_logger(), "Updated lanelet cache with %zu lanelets",
               msg->lanelets.size());
}

void PredictionNode::trackedObjectsCallback(
    const vision_msgs::msg::Detection3DArray::SharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Processing %zu tracked objects",
               msg->detections.size());

  pruneConfidenceHistory(this->get_clock()->now());

  world_model_msgs::msg::WorldObjectArray output;
  output.header = msg->header;

  // Track lanelet IDs reserved by predictions to prevent overlaps
  std::unordered_set<int64_t> reserved_lanelets;

  for (const auto &detection : msg->detections) {
    // Skip generating predictions for traffic lights
    bool is_traffic_light = false;
    for (const auto &result : detection.results) {
      if (result.hypothesis.class_id == "traffic_light") {
        is_traffic_light = true;
        break;
      }
    }
    if (is_traffic_light) {
      continue;
    }

    auto hypotheses = trajectory_predictor_->generateHypotheses(detection);

    if (hypotheses.empty()) {
      RCLCPP_DEBUG(this->get_logger(), "No hypotheses for object %s",
                   detection.id.c_str());
      continue;
    }

    auto features = intent_classifier_->extractFeatures(detection);
    intent_classifier_->assignProbabilities(detection, hypotheses, features);
    applyConfidenceSmoothing(detection.id, hypotheses);

    // Filter out hypotheses that use lanelets already reserved by other objects
    std::vector<TrajectoryHypothesis> filtered_hypotheses;
    for (const auto &hypothesis : hypotheses) {
      bool has_conflict = false;
      // Check if any lanelet in this hypothesis is already reserved
      for (int64_t lanelet_id : hypothesis.lanelet_ids) {
        if (reserved_lanelets.count(lanelet_id) > 0) {
          has_conflict = true;
          RCLCPP_DEBUG(this->get_logger(),
                       "Filtering hypothesis for object %s: lanelet %ld already "
                       "used by another prediction",
                       detection.id.c_str(), lanelet_id);
          break;
        }
      }
      if (!has_conflict) {
        filtered_hypotheses.push_back(hypothesis);
      }
    }

    // If all hypotheses were filtered out, keep the highest-probability one
    // to ensure every object has at least one prediction
    if (filtered_hypotheses.empty() && !hypotheses.empty()) {
      RCLCPP_DEBUG(this->get_logger(),
                   "Object %s all hypotheses filtered; keeping highest probability",
                   detection.id.c_str());
      size_t best_idx = 0;
      double best_prob = hypotheses[0].probability;
      for (size_t i = 1; i < hypotheses.size(); ++i) {
        if (hypotheses[i].probability > best_prob) {
          best_prob = hypotheses[i].probability;
          best_idx = i;
        }
      }
      filtered_hypotheses.push_back(hypotheses[best_idx]);
    }

    // Reserve the lanelets used by this object's predictions
    for (const auto &hypothesis : filtered_hypotheses) {
      for (int64_t lanelet_id : hypothesis.lanelet_ids) {
        reserved_lanelets.insert(lanelet_id);
      }
    }

    world_model_msgs::msg::WorldObject world_obj;
    world_obj.detection = detection;

    for (const auto &hypothesis : filtered_hypotheses) {
      world_model_msgs::msg::Prediction pred;
      pred.header.frame_id = msg->header.frame_id;
      pred.conf = hypothesis.probability;
      pred.poses = hypothesis.poses;
      world_obj.predictions.push_back(pred);
    }

    output.objects.push_back(world_obj);
  }

  world_objects_pub_->publish(output);
}

void PredictionNode::applyConfidenceSmoothing(
    const std::string &object_id,
    std::vector<TrajectoryHypothesis> &hypotheses) {
  if (hypotheses.empty() || object_id.empty()) {
    return;
  }

  auto &state = confidence_history_[object_id];
  std::vector<SmoothedHypothesisState> updated_states;
  updated_states.reserve(hypotheses.size());

  std::unordered_set<size_t> used_previous_indices;

  for (auto &hypothesis : hypotheses) {
    const auto &last_pose = hypothesis.poses.back().pose.position;
    const double curr_x = last_pose.x;
    const double curr_y = last_pose.y;

    size_t best_match_idx = std::numeric_limits<size_t>::max();
    double best_dist_sq = std::numeric_limits<double>::max();

    for (size_t prev_idx = 0; prev_idx < state.hypotheses.size(); ++prev_idx) {
      if (used_previous_indices.count(prev_idx) > 0) {
        continue;
      }

      const auto &prev = state.hypotheses[prev_idx];
      if (prev.intent != hypothesis.intent) {
        continue;
      }

      const double dx = curr_x - prev.end_x;
      const double dy = curr_y - prev.end_y;
      const double dist_sq = dx * dx + dy * dy;
      if (dist_sq < best_dist_sq) {
        best_dist_sq = dist_sq;
        best_match_idx = prev_idx;
      }
    }

    if (best_match_idx != std::numeric_limits<size_t>::max() &&
        best_dist_sq <=
            confidence_match_distance_m_ * confidence_match_distance_m_) {
      used_previous_indices.insert(best_match_idx);
      const double prev_conf = state.hypotheses[best_match_idx].confidence;
      hypothesis.probability =
          confidence_smoothing_alpha_ * hypothesis.probability +
          (1.0 - confidence_smoothing_alpha_) * prev_conf;
    }

    hypothesis.probability = std::clamp(hypothesis.probability, 0.0, 1.0);
    updated_states.push_back(
        {hypothesis.intent, curr_x, curr_y, hypothesis.probability});
  }

  double sum_prob = 0.0;
  for (const auto &hypothesis : hypotheses) {
    sum_prob += hypothesis.probability;
  }
  if (sum_prob > 1e-6) {
    for (auto &hypothesis : hypotheses) {
      hypothesis.probability /= sum_prob;
    }
    for (size_t i = 0; i < updated_states.size(); ++i) {
      updated_states[i].confidence = hypotheses[i].probability;
    }
  }

  state.hypotheses = std::move(updated_states);
  state.last_update = this->get_clock()->now();
}

void PredictionNode::pruneConfidenceHistory(const rclcpp::Time &now) {
  for (auto it = confidence_history_.begin();
       it != confidence_history_.end();) {
    const double age_s = (now - it->second.last_update).seconds();
    if (age_s > confidence_state_timeout_s_) {
      it = confidence_history_.erase(it);
    } else {
      ++it;
    }
  }
}

void PredictionNode::egoPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  ego_pose_ = msg;
}

} // namespace prediction

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node =
      std::make_shared<prediction::PredictionNode>(rclcpp::NodeOptions());

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  executor.spin();

  rclcpp::shutdown();
  return 0;
}

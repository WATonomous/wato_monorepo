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
#include <utility>
#include <vector>

namespace prediction
{

PredictionNode::PredictionNode(const rclcpp::NodeOptions & options)
: LifecycleNode("prediction_node", options)
{
  // Prediction horizon and temporal resolution
  this->declare_parameter("prediction_horizon", 3.0);
  this->declare_parameter("prediction_time_step", 0.2);

  // Cache and confidence smoothing
  this->declare_parameter("confidence_smoothing_alpha", 0.35);
  this->declare_parameter("confidence_match_distance_m", 6.0);
  this->declare_parameter("confidence_state_timeout_s", 5.0);
  this->declare_parameter("cache_ttl_s", 5.0);
  this->declare_parameter("vehicle_cache_invalidation_dist", 5.0);
  this->declare_parameter("lanelet_query_radius", 50.0);

  // Bicycle motion model
  this->declare_parameter("vehicle_wheelbase", 2.5);
  this->declare_parameter("max_steering_angle", 0.785);
  this->declare_parameter("max_lateral_acceleration", 4.0);
  this->declare_parameter("speed_limit", 15.0);
  this->declare_parameter("max_acceleration", 2.0);
  this->declare_parameter("max_deceleration", 4.0);
  this->declare_parameter("lookahead_distance", 3.0);

  // Constant-velocity motion model
  this->declare_parameter("position_noise_std", 0.1);
  this->declare_parameter("heading_noise_std", 0.05);

  // Speed estimation
  this->declare_parameter("speed_ema_alpha", 0.35);
  this->declare_parameter("min_dt_for_speed", 0.01);
  this->declare_parameter("max_vehicle_speed", 25.0);
  this->declare_parameter("max_pedestrian_speed", 3.0);

  // Stop detection
  this->declare_parameter("stop_sigmoid_midpoint", 0.25);
  this->declare_parameter("stop_sigmoid_steepness", 12.0);
  this->declare_parameter("stop_probability_threshold", 0.01);

  // Object classification
  this->declare_parameter("vehicle_min_length", 3.5);
  this->declare_parameter("pedestrian_max_length", 1.0);
  this->declare_parameter("cyclist_max_length", 3.5);
  this->declare_parameter("pedestrian_max_height", 2.0);

  // Default velocities
  this->declare_parameter("vehicle_default_velocity", 5.0);
  this->declare_parameter("pedestrian_default_velocity", 1.4);
  this->declare_parameter("cyclist_default_velocity", 5.0);

  // Lanelet matching
  this->declare_parameter("lanelet_match_max_distance", 10.0);
  this->declare_parameter("heading_penalty_weight", 3.0);
  this->declare_parameter("heading_rejection_threshold", 1.047);

  // Curvature-aware scoring
  this->declare_parameter("curvature_heading_tolerance_scale", 5.0);
  this->declare_parameter("curvature_heading_tolerance_max", 0.3);
  this->declare_parameter("curvature_lateral_tolerance_denom", 8.0);
  this->declare_parameter("curvature_lateral_tolerance_max", 1.0);
  this->declare_parameter("curve_offset_discount", 0.3);

  // Geometric scoring
  this->declare_parameter("heading_score_denominator", 0.2);
  this->declare_parameter("lateral_score_denominator", 4.0);

  // Lane-follow hypothesis
  this->declare_parameter("lane_follow_prior", 2.5);
  this->declare_parameter("required_distance_buffer", 10.0);

  // Smoothness and inertia
  this->declare_parameter("smoothness_penalty_exponent", 2.0);
  this->declare_parameter("smoothness_base_weight", 0.5);
  this->declare_parameter("maneuver_inertia_boost", 1.3);

  // Turn detection
  this->declare_parameter("heading_change_threshold", 0.3);
  this->declare_parameter("turn_maneuver_prior", 0.5);
  this->declare_parameter("continue_maneuver_prior", 1.5);

  // Lane change
  this->declare_parameter("lane_change_base_prior", 0.15);
  this->declare_parameter("lateral_velocity_threshold", 0.3);
  this->declare_parameter("lane_change_active_prior", 0.6);
  this->declare_parameter("intersection_suppression_factor", 0.1);
  this->declare_parameter("curvature_threshold", 0.005);
  this->declare_parameter("curvature_suppression_multiplier", 50.0);
  this->declare_parameter("curvature_suppression_minimum", 0.1);
  this->declare_parameter("position_evidence_base", 0.2);
  this->declare_parameter("position_evidence_scale", 1.8);
  this->declare_parameter("lane_change_offset_threshold", 0.3);
  this->declare_parameter("lane_change_offset_scale", 1.2);
  this->declare_parameter("lateral_offset_threshold", 0.25);
  this->declare_parameter("lateral_offset_boost", 1.25);
  this->declare_parameter("lateral_offset_suppress", 0.6);
  this->declare_parameter("heading_weight_for_lane_change", 0.4);
  this->declare_parameter("lane_change_completion_boost", 1.5);
  this->declare_parameter("lane_change_direction_switch_suppress", 0.1);
  this->declare_parameter("lane_change_blend_count", 5);

  // Geometric fallback
  this->declare_parameter("geometric_straight_probability", 0.6);
  this->declare_parameter("geometric_turn_probability", 0.1);
  this->declare_parameter("geometric_lane_change_probability", 0.1);
  this->declare_parameter("geometric_turn_radius", 10.0);
  this->declare_parameter("geometric_turn_angle_step", 0.1);
  this->declare_parameter("geometric_lane_width", 3.5);
  this->declare_parameter("geometric_lane_change_distance", 30.0);
  this->declare_parameter("geometric_path_sampling_interval", 1.0);

  // Intent classifier feature defaults
  this->declare_parameter("default_velocity", 5.0);
  this->declare_parameter("default_distance_to_intersection", 50.0);
  this->declare_parameter("default_time_in_lane", 2.0);

  // Intent probability model
  this->declare_parameter("straight_base_probability", 0.6);
  this->declare_parameter("turn_signal_discount", 0.5);
  this->declare_parameter("turn_signal_presence_weight", 0.8);
  this->declare_parameter("turn_signal_absence_weight", 0.2);
  this->declare_parameter("intersection_normalization", 50.0);
  this->declare_parameter("stop_velocity_threshold", 1.0);
  this->declare_parameter("stop_high_probability", 0.7);
  this->declare_parameter("stop_low_probability", 0.1);
  this->declare_parameter("default_fallback_probability", 0.1);

  RCLCPP_INFO(this->get_logger(), "PredictionNode created (unconfigured)");
}

PredictionNode::CallbackReturn PredictionNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Configuring...");

  prediction_horizon_ = this->get_parameter("prediction_horizon").as_double();
  prediction_time_step_ = this->get_parameter("prediction_time_step").as_double();
  confidence_smoothing_alpha_ = this->get_parameter("confidence_smoothing_alpha").as_double();
  confidence_match_distance_m_ = this->get_parameter("confidence_match_distance_m").as_double();
  confidence_state_timeout_s_ = this->get_parameter("confidence_state_timeout_s").as_double();
  cache_ttl_s_ = this->get_parameter("cache_ttl_s").as_double();
  lanelet_query_radius_ = this->get_parameter("lanelet_query_radius").as_double();

  confidence_smoothing_alpha_ = std::clamp(confidence_smoothing_alpha_, 0.0, 1.0);
  confidence_match_distance_m_ = std::max(0.1, confidence_match_distance_m_);
  confidence_state_timeout_s_ = std::max(0.5, confidence_state_timeout_s_);
  cache_ttl_s_ = std::max(1.0, cache_ttl_s_);

  RCLCPP_INFO(this->get_logger(), "Prediction horizon: %.2f seconds", prediction_horizon_);
  RCLCPP_INFO(this->get_logger(), "Prediction time step: %.2f seconds", prediction_time_step_);
  RCLCPP_INFO(
    this->get_logger(),
    "Confidence smoothing alpha: %.2f, match distance: %.1fm",
    confidence_smoothing_alpha_,
    confidence_match_distance_m_);

  // Build bicycle model config
  BicycleModelConfig bicycle_config;
  bicycle_config.max_steering_angle = this->get_parameter("max_steering_angle").as_double();
  bicycle_config.wheelbase = this->get_parameter("vehicle_wheelbase").as_double();
  bicycle_config.max_lateral_acceleration = this->get_parameter("max_lateral_acceleration").as_double();
  bicycle_config.speed_limit = this->get_parameter("speed_limit").as_double();
  bicycle_config.max_acceleration = this->get_parameter("max_acceleration").as_double();
  bicycle_config.max_deceleration = this->get_parameter("max_deceleration").as_double();
  bicycle_config.lookahead_distance = this->get_parameter("lookahead_distance").as_double();

  // Build constant-velocity model config
  ConstantVelocityModelConfig cv_config;
  cv_config.position_noise_std = this->get_parameter("position_noise_std").as_double();
  cv_config.heading_noise_std = this->get_parameter("heading_noise_std").as_double();

  // Build trajectory predictor config
  TrajectoryPredictorConfig tp_config;
  tp_config.bicycle_config = bicycle_config;
  tp_config.cv_config = cv_config;

  tp_config.speed_ema_alpha = this->get_parameter("speed_ema_alpha").as_double();
  tp_config.min_dt_for_speed = this->get_parameter("min_dt_for_speed").as_double();
  tp_config.max_vehicle_speed = this->get_parameter("max_vehicle_speed").as_double();
  tp_config.max_pedestrian_speed = this->get_parameter("max_pedestrian_speed").as_double();

  tp_config.stop_sigmoid_midpoint = this->get_parameter("stop_sigmoid_midpoint").as_double();
  tp_config.stop_sigmoid_steepness = this->get_parameter("stop_sigmoid_steepness").as_double();
  tp_config.stop_probability_threshold = this->get_parameter("stop_probability_threshold").as_double();

  tp_config.vehicle_min_length = this->get_parameter("vehicle_min_length").as_double();
  tp_config.pedestrian_max_length = this->get_parameter("pedestrian_max_length").as_double();
  tp_config.cyclist_max_length = this->get_parameter("cyclist_max_length").as_double();

  tp_config.vehicle_default_velocity = this->get_parameter("vehicle_default_velocity").as_double();
  tp_config.pedestrian_default_velocity = this->get_parameter("pedestrian_default_velocity").as_double();
  tp_config.cyclist_default_velocity = this->get_parameter("cyclist_default_velocity").as_double();

  tp_config.lanelet_match_max_distance = this->get_parameter("lanelet_match_max_distance").as_double();
  tp_config.heading_penalty_weight = this->get_parameter("heading_penalty_weight").as_double();
  tp_config.heading_rejection_threshold = this->get_parameter("heading_rejection_threshold").as_double();

  tp_config.curvature_heading_tolerance_scale = this->get_parameter("curvature_heading_tolerance_scale").as_double();
  tp_config.curvature_heading_tolerance_max = this->get_parameter("curvature_heading_tolerance_max").as_double();
  tp_config.curvature_lateral_tolerance_denom = this->get_parameter("curvature_lateral_tolerance_denom").as_double();
  tp_config.curvature_lateral_tolerance_max = this->get_parameter("curvature_lateral_tolerance_max").as_double();
  tp_config.curve_offset_discount = this->get_parameter("curve_offset_discount").as_double();

  tp_config.heading_score_denominator = this->get_parameter("heading_score_denominator").as_double();
  tp_config.lateral_score_denominator = this->get_parameter("lateral_score_denominator").as_double();

  tp_config.lane_follow_prior = this->get_parameter("lane_follow_prior").as_double();
  tp_config.required_distance_buffer = this->get_parameter("required_distance_buffer").as_double();

  tp_config.smoothness_penalty_exponent = this->get_parameter("smoothness_penalty_exponent").as_double();
  tp_config.smoothness_base_weight = this->get_parameter("smoothness_base_weight").as_double();
  tp_config.maneuver_inertia_boost = this->get_parameter("maneuver_inertia_boost").as_double();

  tp_config.heading_change_threshold = this->get_parameter("heading_change_threshold").as_double();
  tp_config.turn_maneuver_prior = this->get_parameter("turn_maneuver_prior").as_double();
  tp_config.continue_maneuver_prior = this->get_parameter("continue_maneuver_prior").as_double();

  tp_config.lane_change_base_prior = this->get_parameter("lane_change_base_prior").as_double();
  tp_config.lateral_velocity_threshold = this->get_parameter("lateral_velocity_threshold").as_double();
  tp_config.lane_change_active_prior = this->get_parameter("lane_change_active_prior").as_double();
  tp_config.intersection_suppression_factor = this->get_parameter("intersection_suppression_factor").as_double();
  tp_config.curvature_threshold = this->get_parameter("curvature_threshold").as_double();
  tp_config.curvature_suppression_multiplier = this->get_parameter("curvature_suppression_multiplier").as_double();
  tp_config.curvature_suppression_minimum = this->get_parameter("curvature_suppression_minimum").as_double();
  tp_config.position_evidence_base = this->get_parameter("position_evidence_base").as_double();
  tp_config.position_evidence_scale = this->get_parameter("position_evidence_scale").as_double();
  tp_config.lane_change_offset_threshold = this->get_parameter("lane_change_offset_threshold").as_double();
  tp_config.lane_change_offset_scale = this->get_parameter("lane_change_offset_scale").as_double();
  tp_config.lateral_offset_threshold = this->get_parameter("lateral_offset_threshold").as_double();
  tp_config.lateral_offset_boost = this->get_parameter("lateral_offset_boost").as_double();
  tp_config.lateral_offset_suppress = this->get_parameter("lateral_offset_suppress").as_double();
  tp_config.heading_weight_for_lane_change = this->get_parameter("heading_weight_for_lane_change").as_double();
  tp_config.lane_change_completion_boost = this->get_parameter("lane_change_completion_boost").as_double();
  tp_config.lane_change_direction_switch_suppress =
    this->get_parameter("lane_change_direction_switch_suppress").as_double();
  tp_config.lane_change_blend_count = this->get_parameter("lane_change_blend_count").as_int();

  tp_config.geometric_straight_probability = this->get_parameter("geometric_straight_probability").as_double();
  tp_config.geometric_turn_probability = this->get_parameter("geometric_turn_probability").as_double();
  tp_config.geometric_lane_change_probability = this->get_parameter("geometric_lane_change_probability").as_double();
  tp_config.geometric_turn_radius = this->get_parameter("geometric_turn_radius").as_double();
  tp_config.geometric_turn_angle_step = this->get_parameter("geometric_turn_angle_step").as_double();
  tp_config.geometric_lane_width = this->get_parameter("geometric_lane_width").as_double();
  tp_config.geometric_lane_change_distance = this->get_parameter("geometric_lane_change_distance").as_double();
  tp_config.geometric_path_sampling_interval = this->get_parameter("geometric_path_sampling_interval").as_double();

  tp_config.vehicle_cache_invalidation_dist = this->get_parameter("vehicle_cache_invalidation_dist").as_double();

  // Build intent classifier config
  IntentClassifierConfig ic_config;
  ic_config.default_velocity = this->get_parameter("default_velocity").as_double();
  ic_config.default_distance_to_intersection = this->get_parameter("default_distance_to_intersection").as_double();
  ic_config.default_time_in_lane = this->get_parameter("default_time_in_lane").as_double();
  ic_config.straight_base_probability = this->get_parameter("straight_base_probability").as_double();
  ic_config.turn_signal_discount = this->get_parameter("turn_signal_discount").as_double();
  ic_config.turn_signal_presence_weight = this->get_parameter("turn_signal_presence_weight").as_double();
  ic_config.turn_signal_absence_weight = this->get_parameter("turn_signal_absence_weight").as_double();
  ic_config.intersection_normalization = this->get_parameter("intersection_normalization").as_double();
  ic_config.stop_velocity_threshold = this->get_parameter("stop_velocity_threshold").as_double();
  ic_config.stop_high_probability = this->get_parameter("stop_high_probability").as_double();
  ic_config.stop_low_probability = this->get_parameter("stop_low_probability").as_double();
  ic_config.default_fallback_probability = this->get_parameter("default_fallback_probability").as_double();

  // All subscription callbacks share a mutually-exclusive callback group so
  // they never run concurrently with each other. The async service-response
  // callback is NOT in this group (it uses the default group), so shared
  // state between them (pending_vehicle_requests_, vehicle_lanelet_cache_)
  // is protected by mutexes.
  subscription_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  world_objects_pub_ = this->create_publisher<world_model_msgs::msg::WorldObjectArray>("world_object_seeds", 10);

  trajectory_predictor_ =
    std::make_unique<TrajectoryPredictor>(this, prediction_horizon_, prediction_time_step_, tp_config);
  intent_classifier_ = std::make_unique<IntentClassifier>(this, ic_config);
  lanelet_ahead_client_ = this->create_client<lanelet_msgs::srv::GetLaneletAhead>("get_lanelet_ahead");

  // Wire the per-vehicle lanelet query as async fire-and-forget.
  // On cache miss, this fires an async service request and returns
  // immediately. The response callback populates the per-vehicle cache,
  // so the next prediction cycle will have lanelet data.
  trajectory_predictor_->setLaneletQueryFunction(
    [this](const std::string & vehicle_id, const geometry_msgs::msg::Point & position, double heading_rad) {
      if (!lanelet_ahead_client_ || !lanelet_ahead_client_->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          5000,
          "GetLaneletAhead service not ready, "
          "falling back to geometric prediction");
        return;
      }
      // Avoid duplicate in-flight requests for the same vehicle,
      // and cap total concurrent requests to avoid queue overflow.
      // Lock: pending_vehicle_requests_ is shared with the async
      // service-response callback which runs on a different thread.
      {
        std::lock_guard<std::mutex> lock(pending_requests_mutex_);
        if (pending_vehicle_requests_.count(vehicle_id) || pending_vehicle_requests_.size() >= kMaxPendingRequests) {
          return;
        }
        pending_vehicle_requests_.insert(vehicle_id);
      }

      auto request = std::make_shared<lanelet_msgs::srv::GetLaneletAhead::Request>();
      request->position = position;
      request->heading_rad = heading_rad;
      request->radius_m = lanelet_query_radius_;

      double vx = position.x, vy = position.y;
      lanelet_ahead_client_->async_send_request(
        request, [this, vehicle_id, vx, vy](rclcpp::Client<lanelet_msgs::srv::GetLaneletAhead>::SharedFuture future) {
          {
            std::lock_guard<std::mutex> lock(pending_requests_mutex_);
            pending_vehicle_requests_.erase(vehicle_id);
          }
          auto response = future.get();
          if (response->success) {
            trajectory_predictor_->updateVehicleLaneletCache(vehicle_id, response->lanelet_ahead, vx, vy);
          } else {
            RCLCPP_WARN_THROTTLE(
              this->get_logger(),
              *this->get_clock(),
              5000,
              "GetLaneletAhead failed for vehicle '%s' at "
              "(%.1f, %.1f): %s",
              vehicle_id.c_str(),
              vx,
              vy,
              response->error_message.c_str());
          }
        });
    });

  RCLCPP_INFO(this->get_logger(), "Configured successfully");
  return CallbackReturn::SUCCESS;
}

PredictionNode::CallbackReturn PredictionNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Activating...");

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = subscription_cb_group_;

  tracked_objects_sub_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
    "tracks_3d", 10, std::bind(&PredictionNode::trackedObjectsCallback, this, std::placeholders::_1), sub_opts);

  ego_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "ego_pose", 10, std::bind(&PredictionNode::egoPoseCallback, this, std::placeholders::_1), sub_opts);

  lanelet_ahead_sub_ = this->create_subscription<lanelet_msgs::msg::LaneletAhead>(
    "lanelet_ahead", 10, std::bind(&PredictionNode::laneletAheadCallback, this, std::placeholders::_1), sub_opts);

  world_objects_pub_->on_activate();

  RCLCPP_INFO(this->get_logger(), "Activated successfully");
  return CallbackReturn::SUCCESS;
}

PredictionNode::CallbackReturn PredictionNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating...");

  tracked_objects_sub_.reset();
  ego_pose_sub_.reset();
  lanelet_ahead_sub_.reset();
  world_objects_pub_->on_deactivate();

  RCLCPP_INFO(this->get_logger(), "Deactivated successfully");
  return CallbackReturn::SUCCESS;
}

PredictionNode::CallbackReturn PredictionNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up...");

  tracked_objects_sub_.reset();
  ego_pose_sub_.reset();
  lanelet_ahead_sub_.reset();
  world_objects_pub_.reset();

  trajectory_predictor_.reset();
  intent_classifier_.reset();

  ego_pose_.reset();
  {
    std::lock_guard<std::mutex> lock(pending_requests_mutex_);
    pending_vehicle_requests_.clear();
  }
  confidence_history_.clear();

  RCLCPP_INFO(this->get_logger(), "Cleaned up successfully");
  return CallbackReturn::SUCCESS;
}

PredictionNode::CallbackReturn PredictionNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down...");

  tracked_objects_sub_.reset();
  ego_pose_sub_.reset();
  lanelet_ahead_sub_.reset();
  world_objects_pub_.reset();

  trajectory_predictor_.reset();
  intent_classifier_.reset();

  ego_pose_.reset();
  {
    std::lock_guard<std::mutex> lock(pending_requests_mutex_);
    pending_vehicle_requests_.clear();
  }
  confidence_history_.clear();

  return CallbackReturn::SUCCESS;
}

void PredictionNode::laneletAheadCallback(const lanelet_msgs::msg::LaneletAhead::SharedPtr msg)
{
  trajectory_predictor_->setLaneletAhead(msg);
  RCLCPP_DEBUG(this->get_logger(), "Updated lanelet cache with %zu lanelets", msg->lanelets.size());
}

void PredictionNode::trackedObjectsCallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Processing %zu tracked objects", msg->detections.size());

  const rclcpp::Time now = this->get_clock()->now();
  pruneConfidenceHistory(now);
  trajectory_predictor_->pruneStaleCaches(now, cache_ttl_s_);

  world_model_msgs::msg::WorldObjectArray output;
  output.header = msg->header;

  // Track lanelet IDs reserved by predictions to prevent overlaps
  std::unordered_set<int64_t> reserved_lanelets;

  for (const auto & detection : msg->detections) {
    // Skip generating predictions for traffic lights
    bool is_traffic_light = false;
    for (const auto & result : detection.results) {
      if (result.hypothesis.class_id == "traffic_light") {
        is_traffic_light = true;
        break;
      }
    }
    if (is_traffic_light) {
      world_model_msgs::msg::WorldObject traffic_light_obj;
      traffic_light_obj.detection = detection;
      output.objects.push_back(traffic_light_obj);
      continue;
    }

    auto hypotheses = trajectory_predictor_->generateHypotheses(detection);

    if (hypotheses.empty()) {
      RCLCPP_DEBUG(this->get_logger(), "No hypotheses for object %s", detection.id.c_str());
      continue;
    }

    auto features = intent_classifier_->extractFeatures(detection);
    intent_classifier_->assignProbabilities(detection, hypotheses, features);
    applyConfidenceSmoothing(detection, hypotheses);

    // Filter out hypotheses that use lanelets already reserved by other objects
    std::vector<TrajectoryHypothesis> filtered_hypotheses;
    for (const auto & hypothesis : hypotheses) {
      bool has_conflict = false;
      // Check if any lanelet in this hypothesis is already reserved
      for (int64_t lanelet_id : hypothesis.lanelet_ids) {
        if (reserved_lanelets.count(lanelet_id) > 0) {
          has_conflict = true;
          RCLCPP_DEBUG(
            this->get_logger(),
            "Filtering hypothesis for object %s: lanelet %ld already "
            "used by another prediction",
            detection.id.c_str(),
            lanelet_id);
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
      RCLCPP_DEBUG(
        this->get_logger(), "Object %s all hypotheses filtered; keeping highest probability", detection.id.c_str());
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
    for (const auto & hypothesis : filtered_hypotheses) {
      for (int64_t lanelet_id : hypothesis.lanelet_ids) {
        reserved_lanelets.insert(lanelet_id);
      }
    }

    world_model_msgs::msg::WorldObject world_obj;
    world_obj.detection = detection;

    for (const auto & hypothesis : filtered_hypotheses) {
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
  const vision_msgs::msg::Detection3D & detection, std::vector<TrajectoryHypothesis> & hypotheses)
{
  const std::string & object_id = detection.id;
  if (hypotheses.empty() || object_id.empty()) {
    return;
  }

  const rclcpp::Time now = this->get_clock()->now();
  auto & state = confidence_history_[object_id];

  state.has_observation = true;

  std::vector<SmoothedHypothesisState> updated_states;
  updated_states.reserve(hypotheses.size());

  std::unordered_set<size_t> used_previous_indices;

  for (auto & hypothesis : hypotheses) {
    const auto & last_pose = hypothesis.poses.back().pose.position;
    const double curr_x = last_pose.x;
    const double curr_y = last_pose.y;

    size_t best_match_idx = std::numeric_limits<size_t>::max();
    double best_dist_sq = std::numeric_limits<double>::max();

    for (size_t prev_idx = 0; prev_idx < state.hypotheses.size(); ++prev_idx) {
      if (used_previous_indices.count(prev_idx) > 0) {
        continue;
      }

      const auto & prev = state.hypotheses[prev_idx];
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

    if (
      best_match_idx != std::numeric_limits<size_t>::max() &&
      best_dist_sq <= confidence_match_distance_m_ * confidence_match_distance_m_)
    {
      used_previous_indices.insert(best_match_idx);
      const double prev_conf = state.hypotheses[best_match_idx].confidence;
      hypothesis.probability =
        confidence_smoothing_alpha_ * hypothesis.probability + (1.0 - confidence_smoothing_alpha_) * prev_conf;
    }

    hypothesis.probability = std::clamp(hypothesis.probability, 0.0, 1.0);
    updated_states.push_back({hypothesis.intent, curr_x, curr_y, hypothesis.probability});
  }

  double sum_prob = 0.0;
  for (const auto & hypothesis : hypotheses) {
    sum_prob += hypothesis.probability;
  }
  if (sum_prob > 1e-6) {
    for (auto & hypothesis : hypotheses) {
      hypothesis.probability /= sum_prob;
    }
    for (size_t i = 0; i < updated_states.size(); ++i) {
      updated_states[i].confidence = hypotheses[i].probability;
    }
  }

  state.hypotheses = std::move(updated_states);
  state.last_update = now;
}

void PredictionNode::pruneConfidenceHistory(const rclcpp::Time & now)
{
  for (auto it = confidence_history_.begin(); it != confidence_history_.end();) {
    const double age_s = (now - it->second.last_update).seconds();
    if (age_s > confidence_state_timeout_s_) {
      it = confidence_history_.erase(it);
    } else {
      ++it;
    }
  }
}

void PredictionNode::egoPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  ego_pose_ = msg;
}

}  // namespace prediction

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<prediction::PredictionNode>(rclcpp::NodeOptions());

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  executor.spin();

  rclcpp::shutdown();
  return 0;
}

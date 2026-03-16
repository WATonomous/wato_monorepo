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

/**
 * @file trajectory_predictor.hpp
 * @brief Trajectory generation for seed WorldObjects.
 *
 * Generates lanelet-aware trajectory predictions for tracked objects.
 * Uses lanelet centerlines as reference paths and the bicycle model
 * for physically-realistic trajectory propagation.
 */

#ifndef PREDICTION__TRAJECTORY_PREDICTOR_HPP_
#define PREDICTION__TRAJECTORY_PREDICTOR_HPP_

#include <deque>
#include <utility>

// Standard library headers for functional programming and data structures
#include <functional>  // For std::function used in callback definitions
#include <memory>  // For std::unique_ptr and std::shared_ptr
#include <mutex>  // For std::mutex protecting cross-thread caches
#include <optional>  // For std::optional return from queryVehicleLanelets
#include <string>  // For std::string in vehicle IDs and keys
#include <unordered_map>  // For caching lanelet and position data
#include <vector>  // For collections of trajectories, centerline points

// ROS2 message types
#include "geometry_msgs/msg/pose_stamped.hpp"  // For trajectory poses with timestamps
#include "lanelet_msgs/msg/lanelet.hpp"  // For lanelet geometry and centerline data
#include "lanelet_msgs/msg/lanelet_ahead.hpp"  // For reachable lanelets from a position
#include "vision_msgs/msg/detection3_d.hpp"  // For detected object information

// Project-specific headers
#include "prediction/motion_models.hpp"  // For bicycle and constant velocity models
#include "rclcpp/rclcpp.hpp"  // ROS2 C++ client library
#include "rclcpp_lifecycle/lifecycle_node.hpp"  // For lifecycle node management

namespace prediction
{

/**
 * @brief Object types for prediction
 */
enum class ObjectType
{
  VEHICLE,
  PEDESTRIAN,
  CYCLIST,
  UNKNOWN
};

/**
 * @brief Intent types for trajectory hypotheses
 */
enum class Intent
{
  CONTINUE_STRAIGHT,
  TURN_LEFT,
  TURN_RIGHT,
  LANE_CHANGE_LEFT,
  LANE_CHANGE_RIGHT,
  STOP,
  UNKNOWN
};

/**
 * @brief Single trajectory hypothesis (sequence of waypoints)
 */
struct TrajectoryHypothesis
{
  std_msgs::msg::Header header;
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  Intent intent;
  double probability;
  std::vector<int64_t> lanelet_ids;  // Lanelets used in this prediction path
};

/**
 * @brief Read-only view of lanelet data for a single prediction call.
 *
 * Built on the stack from either per-vehicle or ego-vehicle lanelet caches
 * and passed through the entire hypothesis-generation call chain, replacing
 * the former mutable global lanelet swap (setTemporaryLaneletData / restore).
 */
struct LaneletContext
{
  const lanelet_msgs::msg::LaneletAhead & data;
  const std::unordered_map<int64_t, size_t> & id_to_index;
};

/**
 * @brief Configuration for the trajectory predictor
 */
struct TrajectoryPredictorConfig
{
  // Speed estimation
  double speed_ema_alpha = 0.35;
  double min_dt_for_speed = 0.01;
  double max_vehicle_speed = 25.0;
  double max_pedestrian_speed = 3.0;

  // Stop detection
  double stop_sigmoid_midpoint = 0.25;
  double stop_sigmoid_steepness = 12.0;
  double stop_probability_threshold = 0.01;

  // Object classification
  double vehicle_min_length = 3.5;
  double pedestrian_max_length = 1.0;
  double cyclist_max_length = 3.5;

  // Default velocities
  double vehicle_default_velocity = 5.0;
  double pedestrian_default_velocity = 1.4;
  double cyclist_default_velocity = 5.0;

  // Lanelet matching
  double lanelet_match_max_distance = 10.0;
  double heading_penalty_weight = 3.0;
  double heading_rejection_threshold = 1.047;  // M_PI / 3.0

  // Curvature-aware scoring
  double curvature_heading_tolerance_scale = 5.0;
  double curvature_heading_tolerance_max = 0.3;
  double curvature_lateral_tolerance_denom = 8.0;
  double curvature_lateral_tolerance_max = 1.0;
  double curve_offset_discount = 0.3;

  // Geometric scoring
  double heading_score_denominator = 0.2;
  double lateral_score_denominator = 4.0;

  // Lane-follow hypothesis
  double lane_follow_prior = 2.5;
  double required_distance_buffer = 10.0;

  // Smoothness and inertia
  double smoothness_penalty_exponent = 2.0;
  double smoothness_base_weight = 0.5;
  double maneuver_inertia_boost = 1.3;

  // Turn detection
  double heading_change_threshold = 0.3;
  double turn_maneuver_prior = 0.5;
  double continue_maneuver_prior = 1.5;

  // Lane change
  double lane_change_base_prior = 0.15;
  double lateral_velocity_threshold = 0.3;
  double lane_change_active_prior = 0.6;
  double intersection_suppression_factor = 0.1;
  double curvature_threshold = 0.005;
  double curvature_suppression_multiplier = 50.0;
  double curvature_suppression_minimum = 0.1;
  double position_evidence_base = 0.2;
  double position_evidence_scale = 1.8;
  double lane_change_offset_threshold = 0.3;
  double lane_change_offset_scale = 1.2;
  double lateral_offset_threshold = 0.25;
  double lateral_offset_boost = 1.25;
  double lateral_offset_suppress = 0.6;
  double heading_weight_for_lane_change = 0.4;
  double lane_change_completion_boost = 1.5;
  double lane_change_direction_switch_suppress = 0.1;
  int lane_change_blend_count = 5;

  // Geometric fallback
  double geometric_straight_probability = 0.6;
  double geometric_turn_probability = 0.1;
  double geometric_lane_change_probability = 0.1;
  double geometric_turn_radius = 10.0;
  double geometric_turn_angle_step = 0.1;
  double geometric_lane_width = 3.5;
  double geometric_lane_change_distance = 30.0;
  double geometric_path_sampling_interval = 1.0;

  // Cache
  double vehicle_cache_invalidation_dist = 5.0;

  // Lateral velocity estimation
  double lateral_velocity_min_dt = 0.01;
  double lateral_velocity_max_dt = 2.0;

  // Bicycle model config (forwarded)
  BicycleModelConfig bicycle_config;
  ConstantVelocityModelConfig cv_config;
};

/**
 * @brief Timestamped position for detection history tracking
 */
struct TimestampedPosition
{
  geometry_msgs::msg::Point position;
  double timestamp;  // Seconds since epoch
};

/**
 * @brief Generates trajectory hypotheses for tracked objects
 *
 * Uses lanelet centerlines as reference paths and physics-based motion models
 * to generate trajectory hypotheses that follow the actual road geometry.
 */
class TrajectoryPredictor
{
public:
  /**
   * @brief Parameters for vehicle prediction (loaded from params.yaml)
   */
  struct VehicleParams
  {
    double default_speed;  // m/s - fallback when no velocity history
    double max_speed;  // m/s - clamp upper bound
  };

  /**
   * @brief Parameters for pedestrian prediction (loaded from params.yaml)
   */
  struct PedestrianParams
  {
    double default_speed;  // m/s - fallback when no velocity history
    double max_speed;  // m/s - clamp upper bound
  };

  /**
   * @brief Parameters for cyclist prediction (loaded from params.yaml)
   */
  struct CyclistParams
  {
    double lanelet_proximity_threshold;  // meters
    double min_speed;  // m/s
    double max_speed;  // m/s
    int max_lanelet_search_depth;  // max successive lanelets to follow
    double lanelet_confidence;  // total confidence for lanelet-based hypotheses
    double cv_fallback_confidence;  // confidence for constant-velocity fallback
    double straight_boost;  // weight multiplier for straight-ahead paths
    double turn_angle_threshold;  // radians - threshold for turn detection
  };

  /**
   * @brief Construct a trajectory predictor with model and scoring
   * configuration.
   * @param node Lifecycle node used for logging and ROS time access.
   * @param prediction_horizon Prediction horizon in seconds.
   * @param time_step Time resolution between predicted poses in seconds.
   * @param config Tuning parameters for classification, scoring, and motion
   * models.
   * @param vehicle_params Vehicle-specific parameters (from params.yaml)
   * @param pedestrian_params Pedestrian-specific parameters (from params.yaml)
   * @param cyclist_params Cyclist-specific parameters (from params.yaml)
   * @param lanelet_handler Optional LaneletHandler for map queries

   */
  TrajectoryPredictor(
    rclcpp_lifecycle::LifecycleNode * node,
    double prediction_horizon,
    double time_step,
    const VehicleParams & vehicle_params,
    const PedestrianParams & pedestrian_params,
    const CyclistParams & cyclist_params,
    const TrajectoryPredictorConfig & config = {});

  /**
   * @brief Generate trajectory hypotheses for a single detection.
   *
   * Selects an object model, attempts lanelet-aware prediction when possible,
   * and falls back to geometric propagation if lane context is unavailable.
   *
   * @param detection Incoming tracked object detection.
   * @return List of trajectory hypotheses with intents and probabilities.
   */
  std::vector<TrajectoryHypothesis> generateHypotheses(
    const vision_msgs::msg::Detection3D & detection, double timestamp);

  /**
   * @brief Classify detection into a predictor object type.
   * @param detection Incoming tracked object detection.
   * @return Coarse object type used to choose motion model and priors.
   */
  ObjectType classifyObjectType(const vision_msgs::msg::Detection3D & detection) const;

  /**
   * @brief Generate hypotheses for vehicle objects
   * @param velocity Computed velocity, or nullopt if insufficient history
   */
  std::vector<TrajectoryHypothesis> generateVehicleHypotheses(
    const vision_msgs::msg::Detection3D & detection, std::optional<double> velocity);

  /**
   * @brief Generate hypotheses for pedestrian objects
   * @param velocity Computed velocity, or nullopt if insufficient history
   * @brief Update cached lanelet data for lanelet-aware prediction
   */
  void setLaneletAhead(const lanelet_msgs::msg::LaneletAhead::SharedPtr & msg);

  /**
   * @brief Callback type for requesting lanelets around an arbitrary position.
   *
   * Fire-and-forget: the callback fires an async service request.
   * Results are delivered later via updateVehicleLaneletCache().
   */
  using LaneletQueryFn =
    std::function<void(const std::string & vehicle_id, const geometry_msgs::msg::Point & position, double heading_rad)>;

  /**
   * @brief Set the per-vehicle lanelet query function.
   *
   * When the ego-vehicle lanelet cache does not cover a detected vehicle,
   * this function is called to fetch lanelets around that vehicle's position.
   */
  void setLaneletQueryFunction(LaneletQueryFn fn);

  /**
   * @brief Update the per-vehicle lanelet cache with async query results.
   * Thread-safe: protected by vehicle_cache_mutex_.
   */
  void updateVehicleLaneletCache(
    const std::string & vehicle_id, const lanelet_msgs::msg::LaneletAhead & data, double x, double y);

  /**
   * @brief Check whether ego lanelet cache currently contains lanelets.
   * @return True when lanelet-aware prediction can potentially run.
   */
  bool hasLaneletData() const
  {
    return !lanelet_cache_.lanelets.empty();
  }

  /**
   * @brief Prune all ID-keyed caches, removing entries older than ttl_s.
   * Called once per prediction cycle to bound memory growth.
   */
  void pruneStaleCaches(const rclcpp::Time & now, double ttl_s);

  /**
   * @brief Estimate vehicle speed from position history.
   *
   * Tracks each detection ID's position over time and computes speed
   * from displacement between the current and previous observation.
   * Falls back to the bbox-length heuristic when no history exists.
   */
  double estimateSpeed(const vision_msgs::msg::Detection3D & detection);

private:
  /**
   * @brief Generate lanelet-aware vehicle hypotheses from a local lanelet
   * context.
   */
  std::vector<TrajectoryHypothesis> generateLaneletVehicleHypotheses(
    const vision_msgs::msg::Detection3D & detection, double speed, const LaneletContext & ctx);

  /**
   * @brief Generate geometric vehicle hypotheses when lanelet data is
   * unavailable.
   */
  std::vector<TrajectoryHypothesis> generateGeometricVehicleHypotheses(
    const vision_msgs::msg::Detection3D & detection, double speed);

  /**
   * @brief Generate pedestrian hypotheses using pedestrian-oriented motion
   * assumptions.
   */
  std::vector<TrajectoryHypothesis> generatePedestrianHypotheses(
    const vision_msgs::msg::Detection3D & detection, std::optional<double> velocity);

  /**
   * @brief Generate hypotheses for cyclist objects (hybrid model)
   * @param velocity Computed velocity, or nullopt if insufficient history
   */
  std::vector<TrajectoryHypothesis> generateCyclistHypotheses(
    const vision_msgs::msg::Detection3D & detection, std::optional<double> velocity);

  /**
   * @brief Update detection history and compute velocity
   * @param object_id Unique object identifier
   * @param position Current position
   * @param timestamp Current timestamp
   * @return Computed velocity magnitude (m/s), or nullopt if insufficient
   * history
   */
  std::optional<double> updateHistoryAndComputeVelocity(
    const std::string & object_id, const geometry_msgs::msg::Point & position, double timestamp);

  /**
   * @brief Get all possible lanelet paths (for multi-hypothesis generation)
   * @param start_lanelet Starting lanelet
   * @param max_depth Maximum number of successive lanelets to follow
   * @return Vector of (path_points, intent) pairs for each possible route
   */
  std::vector<std::pair<std::vector<Eigen::Vector2d>, Intent>> getAllLaneletPaths(
    const lanelet_msgs::msg::Lanelet & start_lanelet, int max_depth, const LaneletContext & ctx) const;

  /**
   * @brief Build a single constant-velocity fallback hypothesis
   * @param state Kinematic state for trajectory generation
   * @param velocity Current velocity for logging
   * @param reason Descriptive reason for fallback (for logging)
   * @return Vector containing a single CV hypothesis with probability 1.0
   */
  std::vector<TrajectoryHypothesis> buildCvFallback(const KinematicState & state, double velocity, const char * reason);

  // Lanelet helpers
  struct LaneletMatch
  {
    const lanelet_msgs::msg::Lanelet * lanelet;
    size_t closest_centerline_idx;
    double lateral_offset;
    double heading_diff;
  };

  /**
   * @brief Match a vehicle state to the best lanelet candidate in context.
   */
  LaneletMatch findVehicleLanelet(const KinematicState & state, const LaneletContext & ctx) const;

  /**
   * @brief Extract a forward centerline path through lanelet successors.
   */
  std::vector<Eigen::Vector2d> extractForwardPath(
    const lanelet_msgs::msg::Lanelet & lanelet,
    size_t start_idx,
    double required_distance,
    const LaneletContext & ctx) const;

  /**
   * @brief Build a lane-change path from current lanelet toward a target
   * lanelet.
   */
  std::vector<Eigen::Vector2d> extractLaneChangePath(
    const lanelet_msgs::msg::Lanelet & current_lanelet,
    size_t start_idx,
    int64_t target_lane_id,
    double required_distance,
    const LaneletContext & ctx) const;

  /// Extract lanelet IDs used in a forward path through successors
  std::vector<int64_t> extractForwardPathLaneletIds(
    const lanelet_msgs::msg::Lanelet & lanelet, double required_distance, const LaneletContext & ctx) const;

  /// Extract lanelet IDs used in a lane change path
  std::vector<int64_t> extractLaneChangePathLaneletIds(
    const lanelet_msgs::msg::Lanelet & current_lanelet,
    int64_t target_lane_id,
    double required_distance,
    const LaneletContext & ctx) const;

  /**
   * @brief Lookup lanelet pointer by ID using precomputed index map.
   */
  const lanelet_msgs::msg::Lanelet * findLaneletById(int64_t id, const LaneletContext & ctx) const;

  /**
   * @brief Compute polyline arc length for a 2D path.
   */
  double computePathLength(const std::vector<Eigen::Vector2d> & path) const;

  /**
   * @brief Compute path heading at a centerline index.
   */
  double computeHeadingAtIndex(const std::vector<geometry_msgs::msg::Point> & centerline, size_t idx) const;

  /**
   * @brief Compute geometric compatibility score for a maneuver candidate.
   */
  double computeGeometricScore(double heading_diff, double lateral_offset, double maneuver_prior) const;

  /// Compute path curvature at a given centerline index (1/radius)
  double computeCurvatureAtIndex(const std::vector<geometry_msgs::msg::Point> & centerline, size_t idx) const;

  /// Compute trajectory smoothness penalty (mean absolute curvature)
  double computeTrajectorySmoothness(const std::vector<geometry_msgs::msg::PoseStamped> & poses) const;

  /// Estimate lateral velocity (rate of change of lateral offset)
  double estimateLateralVelocity(const std::string & vehicle_id, double lateral_offset);

  // Per-vehicle lateral offset history for lateral velocity estimation
  struct LateralOffsetStamped
  {
    double lateral_offset;
    rclcpp::Time stamp;
  };

  std::unordered_map<std::string, LateralOffsetStamped> lateral_offset_history_;

  // Per-vehicle previous intent for maneuver inertia
  struct IntentStamped
  {
    Intent intent;
    rclcpp::Time stamp;
  };

  std::unordered_map<std::string, IntentStamped> previous_intent_;

  rclcpp_lifecycle::LifecycleNode * node_;
  double prediction_horizon_;
  double time_step_;
  TrajectoryPredictorConfig config_;

  std::unique_ptr<BicycleModel> bicycle_model_;
  std::unique_ptr<ConstantVelocityModel> constant_velocity_model_;

  // Per-type parameters (const after construction)
  const VehicleParams vehicle_params_;
  const PedestrianParams pedestrian_params_;
  const CyclistParams cyclist_params_;

  // Detection history for velocity computation (object_id -> history)
  // Protected by mutex for thread-safe access from multi-threaded executor
  mutable std::mutex history_mutex_;
  std::unordered_map<std::string, std::deque<TimestampedPosition>> detection_history_;

  // History parameters
  static constexpr double kHistoryMaxAge = 2.0;  // seconds
  static constexpr size_t kHistoryMaxSize = 10;  // entries per object
  // Cached ego-vehicle lanelet data (from lanelet_ahead subscription)
  lanelet_msgs::msg::LaneletAhead lanelet_cache_;
  std::unordered_map<int64_t, size_t> lanelet_id_to_index_;

  // Optional per-vehicle lanelet query callback
  LaneletQueryFn lanelet_query_fn_;

  // Per-vehicle lanelet cache: keyed by detection ID, stores queried lanelet
  // data so we don't re-query the service every cycle for the same vehicle.
  // Protected by vehicle_cache_mutex_ (written from async service response
  // callback, read from subscription callback thread).
  struct VehicleLaneletEntry
  {
    lanelet_msgs::msg::LaneletAhead lanelet_ahead;
    std::unordered_map<int64_t, size_t> id_to_index;
    double last_x;
    double last_y;
    rclcpp::Time last_update;
  };

  mutable std::mutex vehicle_cache_mutex_;
  std::unordered_map<std::string, VehicleLaneletEntry> vehicle_lanelet_cache_;

  // Position history for velocity estimation (keyed by detection ID)
  struct PositionStamped
  {
    double x;
    double y;
    double speed;  // Raw instantaneous speed
    double smoothed_speed;  // EMA-filtered speed
    rclcpp::Time stamp;
  };

  std::unordered_map<std::string, PositionStamped> position_history_;

public:
  /**
   * @brief Probability that a vehicle is stopped, given its smoothed speed.
   *
   * Pure sigmoid — no internal state.  Returns ~1.0 for speed ≈ 0,
   * 0.5 at 0.5 m/s, and ~0 above ~1.5 m/s.
   */
  double computeStopProbability(double speed) const;

  /**
   * @brief Query lanelets for a specific vehicle, using per-vehicle cache.
   *
   * Returns a copy of the cached entry if available and fresh, otherwise
   * fires an async query and returns std::nullopt.
   * Thread-safe: protected by vehicle_cache_mutex_.
   */
  std::optional<VehicleLaneletEntry> queryVehicleLanelets(
    const std::string & vehicle_id, const geometry_msgs::msg::Point & position, double heading_rad);
};

}  // namespace prediction

#endif  // PREDICTION__TRAJECTORY_PREDICTOR_HPP_

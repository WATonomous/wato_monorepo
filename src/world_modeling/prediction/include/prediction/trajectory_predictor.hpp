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

// Standard library headers for functional programming and data structures
#include <functional>    // For std::function used in callback definitions
#include <memory>        // For std::unique_ptr and std::shared_ptr
#include <string>        // For std::string in vehicle IDs and keys
#include <unordered_map> // For caching lanelet and position data
#include <vector>        // For collections of trajectories, centerline points

// ROS2 message types
#include "geometry_msgs/msg/pose_stamped.hpp" // For trajectory poses with timestamps
#include "lanelet_msgs/msg/lanelet.hpp" // For lanelet geometry and centerline data
#include "lanelet_msgs/msg/lanelet_ahead.hpp" // For reachable lanelets from a position
#include "vision_msgs/msg/detection3_d.hpp" // For detected object information

// Project-specific headers
#include "prediction/motion_models.hpp" // For bicycle and constant velocity models
#include "rclcpp/rclcpp.hpp"            // ROS2 C++ client library
#include "rclcpp_lifecycle/lifecycle_node.hpp" // For lifecycle node management

namespace prediction {

/**
 * @brief Object types for prediction
 */
enum class ObjectType { VEHICLE, PEDESTRIAN, CYCLIST, UNKNOWN };

/**
 * @brief Intent types for trajectory hypotheses
 */
enum class Intent {
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
struct TrajectoryHypothesis {
  std_msgs::msg::Header header;
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  Intent intent;
  double probability;
  std::vector<int64_t> lanelet_ids; // Lanelets used in this prediction path
};

/**
 * @brief Generates trajectory hypotheses for tracked objects
 *
 * Uses lanelet centerlines as reference paths and physics-based motion models
 * to generate trajectory hypotheses that follow the actual road geometry.
 */
class TrajectoryPredictor {
public:
  TrajectoryPredictor(rclcpp_lifecycle::LifecycleNode *node,
                      double prediction_horizon, double time_step);

  std::vector<TrajectoryHypothesis>
  generateHypotheses(const vision_msgs::msg::Detection3D &detection);

  ObjectType classifyObjectType(const vision_msgs::msg::Detection3D &detection);

  /**
   * @brief Update cached lanelet data for lanelet-aware prediction
   */
  void setLaneletAhead(const lanelet_msgs::msg::LaneletAhead::SharedPtr &msg);

  /**
   * @brief Callback type for requesting lanelets around an arbitrary position.
   *
   * Fire-and-forget: the callback fires an async service request.
   * Results are delivered later via updateVehicleLaneletCache().
   */
  using LaneletQueryFn = std::function<void(
      const std::string &vehicle_id, const geometry_msgs::msg::Point &position,
      double heading_rad)>;

  /**
   * @brief Set the per-vehicle lanelet query function.
   *
   * When the ego-vehicle lanelet cache does not cover a detected vehicle,
   * this function is called to fetch lanelets around that vehicle's position.
   */
  void setLaneletQueryFunction(LaneletQueryFn fn);

  /**
   * @brief Update the per-vehicle lanelet cache with async query results.
   */
  void updateVehicleLaneletCache(const std::string &vehicle_id,
                                 const lanelet_msgs::msg::LaneletAhead &data,
                                 double x, double y);

  /**
   * @brief Temporarily set lanelet data for a single prediction, then restore
   * previous cache
   */
  void setTemporaryLaneletData(const lanelet_msgs::msg::LaneletAhead &data);
  void restoreLaneletData();

  bool hasLaneletData() const { return !lanelet_cache_.lanelets.empty(); }

  /**
   * @brief Estimate vehicle speed from position history.
   *
   * Tracks each detection ID's position over time and computes speed
   * from displacement between the current and previous observation.
   * Falls back to the bbox-length heuristic when no history exists.
   */
  double estimateSpeed(const vision_msgs::msg::Detection3D &detection);

private:
  // Lanelet-aware hypothesis generation
  std::vector<TrajectoryHypothesis> generateLaneletVehicleHypotheses(
      const vision_msgs::msg::Detection3D &detection, double speed);

  // Geometric fallback (no lanelet data)
  std::vector<TrajectoryHypothesis> generateGeometricVehicleHypotheses(
      const vision_msgs::msg::Detection3D &detection, double speed);

  std::vector<TrajectoryHypothesis>
  generatePedestrianHypotheses(const vision_msgs::msg::Detection3D &detection);

  std::vector<TrajectoryHypothesis>
  generateCyclistHypotheses(const vision_msgs::msg::Detection3D &detection);

  // Lanelet helpers
  struct LaneletMatch {
    const lanelet_msgs::msg::Lanelet *lanelet;
    size_t closest_centerline_idx;
    double lateral_offset;
    double heading_diff;
  };

  LaneletMatch findVehicleLanelet(const KinematicState &state) const;

  std::vector<Eigen::Vector2d>
  extractForwardPath(const lanelet_msgs::msg::Lanelet &lanelet,
                     size_t start_idx, double required_distance) const;

  std::vector<Eigen::Vector2d>
  extractLaneChangePath(const lanelet_msgs::msg::Lanelet &current_lanelet,
                        size_t start_idx, int64_t target_lane_id,
                        double required_distance) const;

  /// Extract lanelet IDs used in a forward path through successors
  std::vector<int64_t>
  extractForwardPathLaneletIds(const lanelet_msgs::msg::Lanelet &lanelet,
                               double required_distance) const;

  /// Extract lanelet IDs used in a lane change path
  std::vector<int64_t> extractLaneChangePathLaneletIds(
      const lanelet_msgs::msg::Lanelet &current_lanelet, int64_t target_lane_id,
      double required_distance) const;

  const lanelet_msgs::msg::Lanelet *findLaneletById(int64_t id) const;

  double computePathLength(const std::vector<Eigen::Vector2d> &path) const;

  double computeHeadingAtIndex(
      const std::vector<geometry_msgs::msg::Point> &centerline,
      size_t idx) const;

  double computeGeometricScore(double heading_diff, double lateral_offset,
                               double maneuver_prior) const;

  /// Compute path curvature at a given centerline index (1/radius)
  double computeCurvatureAtIndex(
      const std::vector<geometry_msgs::msg::Point> &centerline,
      size_t idx) const;

  /// Compute trajectory smoothness penalty (mean absolute curvature)
  double computeTrajectorySmoothness(
      const std::vector<geometry_msgs::msg::PoseStamped> &poses) const;

  /// Estimate lateral velocity (rate of change of lateral offset)
  double estimateLateralVelocity(const std::string &vehicle_id,
                                 double lateral_offset);

  // Per-vehicle lateral offset history for lateral velocity estimation
  struct LateralOffsetStamped {
    double lateral_offset;
    rclcpp::Time stamp;
  };
  std::unordered_map<std::string, LateralOffsetStamped> lateral_offset_history_;

  // Per-vehicle previous intent for maneuver inertia
  std::unordered_map<std::string, Intent> previous_intent_;

  rclcpp_lifecycle::LifecycleNode *node_;
  double prediction_horizon_;
  double time_step_;

  std::unique_ptr<BicycleModel> bicycle_model_;
  std::unique_ptr<ConstantVelocityModel> constant_velocity_model_;

  // Cached lanelet data
  lanelet_msgs::msg::LaneletAhead lanelet_cache_;
  std::unordered_map<int64_t, size_t> lanelet_id_to_index_;

  // Backup for temporary lanelet overrides
  lanelet_msgs::msg::LaneletAhead lanelet_cache_backup_;
  std::unordered_map<int64_t, size_t> lanelet_id_to_index_backup_;

  // Optional per-vehicle lanelet query callback
  LaneletQueryFn lanelet_query_fn_;

  // Per-vehicle lanelet cache: keyed by detection ID, stores queried lanelet
  // data so we don't re-query the service every cycle for the same vehicle
  struct VehicleLaneletEntry {
    lanelet_msgs::msg::LaneletAhead lanelet_ahead;
    double last_x;
    double last_y;
  };
  std::unordered_map<std::string, VehicleLaneletEntry> vehicle_lanelet_cache_;
  static constexpr double kVehicleCacheInvalidationDist = 5.0;

  // Position history for velocity estimation (keyed by detection ID)
  struct PositionStamped {
    double x;
    double y;
    double speed;
    rclcpp::Time stamp;
  };
  std::unordered_map<std::string, PositionStamped> position_history_;

  /**
   * @brief Query lanelets for a specific vehicle, using per-vehicle cache.
   *
   * Returns cached data if the vehicle hasn't moved far, otherwise queries
   * the service and updates the cache.
   */
  lanelet_msgs::msg::LaneletAhead
  queryVehicleLanelets(const std::string &vehicle_id,
                       const geometry_msgs::msg::Point &position,
                       double heading_rad);
};

} // namespace prediction

#endif // PREDICTION__TRAJECTORY_PREDICTOR_HPP_

// Copyright 2021 Tier IV, Inc.
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

#ifndef motion_forecasting__motion_forecasting_NODE_HPP_
#define motion_forecasting__motion_forecasting_NODE_HPP_

#include "motion_forecasting/path_generator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "autoware/universe_utils/geometry/geometry.hpp"
#include "autoware/universe_utils/ros/update_param.hpp"

#include <rclcpp/rclcpp.hpp>
#include <autoware/universe_utils/ros/debug_publisher.hpp>
#include <autoware/universe_utils/ros/published_time_publisher.hpp>
#include <autoware/universe_utils/ros/transform_listener.hpp>
#include <autoware/universe_utils/ros/uuid_helper.hpp>
#include <autoware/universe_utils/system/stop_watch.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_routing/Forward.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <algorithm>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace motion_forecasting
{
struct LateralKinematicsToLanelet
{
  double dist_from_left_boundary;
  double dist_from_right_boundary;
  double left_lateral_velocity;
  double right_lateral_velocity;
  double filtered_left_lateral_velocity;
  double filtered_right_lateral_velocity;
};

enum class Maneuver {
  UNINITIALIZED = 0,
  LANE_FOLLOW = 1,
  LEFT_LANE_CHANGE = 2,
  RIGHT_LANE_CHANGE = 3,
};

struct ObjectData
{
  std_msgs::msg::Header header;
  lanelet::ConstLanelets current_lanelets;
  lanelet::ConstLanelets future_possible_lanelets;
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Twist twist;
  double time_delay;
  // for lane change prediction
  std::unordered_map<lanelet::ConstLanelet, LateralKinematicsToLanelet> lateral_kinematics_set;
  Maneuver one_shot_maneuver{Maneuver::UNINITIALIZED};
  Maneuver output_maneuver{
    Maneuver::UNINITIALIZED};  // output maneuver considering previous one shot maneuvers
};

struct CrosswalkUserData
{
  std_msgs::msg::Header header;
  autoware_perception_msgs::msg::TrackedObject tracked_object;
};

struct LaneletData
{
  lanelet::Lanelet lanelet;
  float probability;
};

struct PredictedRefPath
{
  float probability;
  double speed_limit;
  PosePath path;
  Maneuver maneuver;
};

struct PredictionTimeHorizon
{
  // NOTE(Mamoru Sobue): motorcycle belongs to "vehicle" and bicycle to "pedestrian"
  double vehicle;
  double pedestrian;
  double unknown;
};

using LaneletsData = std::vector<LaneletData>;
using ManeuverProbability = std::unordered_map<Maneuver, float>;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjectKinematics;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::PredictedPath;
using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjectKinematics;
using autoware_perception_msgs::msg::TrackedObjects;
using autoware_perception_msgs::msg::TrafficLightElement;
using autoware_perception_msgs::msg::TrafficLightGroup;
using autoware_perception_msgs::msg::TrafficLightGroupArray;
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware::universe_utils::StopWatch;
using tier4_debug_msgs::msg::StringStamped;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
class MapBasedPredictionNode : public rclcpp::Node
{
public:
  explicit MapBasedPredictionNode(const rclcpp::NodeOptions & node_options);

private:
  // ROS Publisher and Subscriber
  rclcpp::Publisher<PredictedObjects>::SharedPtr pub_objects_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_markers_;
  rclcpp::Subscription<TrackedObjects>::SharedPtr sub_objects_;
  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<TrafficLightGroupArray>::SharedPtr sub_traffic_signals_;

  // debug publisher
  std::unique_ptr<autoware::universe_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<autoware::universe_utils::DebugPublisher> processing_time_publisher_;

  // Object History
  std::unordered_map<std::string, std::deque<ObjectData>> road_users_history;
  std::map<std::pair<std::string, lanelet::Id>, rclcpp::Time> stopped_times_against_green_;
  std::unordered_map<std::string, std::deque<CrosswalkUserData>> crosswalk_users_history_;
  std::unordered_map<std::string, std::string> known_matches_;

  // Lanelet Map Pointers
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr_;
  std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr_;

  std::unordered_map<lanelet::Id, TrafficLightGroup> traffic_signal_id_map_;

  // parameter update
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onParam(
    const std::vector<rclcpp::Parameter> & parameters);

  // Pose Transform Listener
  autoware::universe_utils::TransformListener transform_listener_{this};

  // Path Generator
  std::shared_ptr<PathGenerator> path_generator_;

  // Crosswalk Entry Points
  lanelet::ConstLanelets crosswalks_;

  // Parameters
  bool enable_delay_compensation_;
  PredictionTimeHorizon prediction_time_horizon_;
  double lateral_control_time_horizon_;
  double prediction_time_horizon_rate_for_validate_lane_length_;
  double prediction_sampling_time_interval_;
  double min_velocity_for_motion_forecasting_;
  double min_crosswalk_user_velocity_;
  double max_crosswalk_user_delta_yaw_threshold_for_lanelet_;
  double debug_accumulated_time_;
  double dist_threshold_for_searching_lanelet_;
  double delta_yaw_threshold_for_searching_lanelet_;
  double sigma_lateral_offset_;
  double sigma_yaw_angle_deg_;
  double object_buffer_time_length_;
  double history_time_length_;
  std::string lane_change_detection_method_;
  double dist_threshold_to_bound_;
  double time_threshold_to_bound_;
  double cutoff_freq_of_velocity_lpf_;
  double dist_ratio_threshold_to_left_bound_;
  double dist_ratio_threshold_to_right_bound_;
  double diff_dist_threshold_to_left_bound_;
  double diff_dist_threshold_to_right_bound_;
  int num_continuous_state_transition_;
  bool consider_only_routable_neighbours_;
  double reference_path_resolution_;

  bool check_lateral_acceleration_constraints_;
  double max_lateral_accel_;
  double min_acceleration_before_curve_;

  bool use_vehicle_acceleration_;
  double speed_limit_multiplier_;
  double acceleration_exponential_half_life_;

  bool use_crosswalk_signal_;
  double threshold_velocity_assumed_as_stopping_;
  std::vector<double> distance_set_for_no_intention_to_walk_;
  std::vector<double> timeout_set_for_no_intention_to_walk_;

  bool match_lost_and_appeared_crosswalk_users_;
  bool remember_lost_crosswalk_users_;

  std::unique_ptr<autoware::universe_utils::PublishedTimePublisher> published_time_publisher_;

  // Member Functions
  void mapCallback(const LaneletMapBin::ConstSharedPtr msg);
  void trafficSignalsCallback(const TrafficLightGroupArray::ConstSharedPtr msg);
  void objectsCallback(const TrackedObjects::ConstSharedPtr in_objects);

  bool doesPathCrossAnyFence(const PredictedPath & predicted_path);
  bool doesPathCrossFence(
    const PredictedPath & predicted_path, const lanelet::ConstLineString3d & fence_line);
  lanelet::BasicLineString2d convertToFenceLine(const lanelet::ConstLineString3d & fence);
  bool isIntersecting(
    const geometry_msgs::msg::Point & point1, const geometry_msgs::msg::Point & point2,
    const lanelet::ConstPoint3d & point3, const lanelet::ConstPoint3d & point4);

  PredictedObjectKinematics convertToPredictedKinematics(
    const TrackedObjectKinematics & tracked_object);

  PredictedObject convertToPredictedObject(const TrackedObject & tracked_object);

  PredictedObject getPredictedObjectAsCrosswalkUser(const TrackedObject & object);

  void removeStaleTrafficLightInfo(const TrackedObjects::ConstSharedPtr in_objects);

  LaneletsData getCurrentLanelets(const TrackedObject & object);
  bool checkCloseLaneletCondition(
    const std::pair<double, lanelet::Lanelet> & lanelet, const TrackedObject & object);
  float calculateLocalLikelihood(
    const lanelet::Lanelet & current_lanelet, const TrackedObject & object) const;
  void updateObjectData(TrackedObject & object);

  void updateRoadUsersHistory(
    const std_msgs::msg::Header & header, const TrackedObject & object,
    const LaneletsData & current_lanelets_data);
  void updateCrosswalkUserHistory(
    const std_msgs::msg::Header & header, const TrackedObject & object,
    const std::string & object_id);
  std::string tryMatchNewObjectToDisappeared(
    const std::string & object_id, std::unordered_map<std::string, TrackedObject> & current_users);
  std::vector<PredictedRefPath> getPredictedReferencePath(
    const TrackedObject & object, const LaneletsData & current_lanelets_data,
    const double object_detected_time, const double time_horizon);
  Maneuver predictObjectManeuver(
    const TrackedObject & object, const LaneletData & current_lanelet_data,
    const double object_detected_time);
  geometry_msgs::msg::Pose compensateTimeDelay(
    const geometry_msgs::msg::Pose & delayed_pose, const geometry_msgs::msg::Twist & twist,
    const double dt) const;
  double calcRightLateralOffset(
    const lanelet::ConstLineString2d & boundary_line, const geometry_msgs::msg::Pose & search_pose);
  double calcLeftLateralOffset(
    const lanelet::ConstLineString2d & boundary_line, const geometry_msgs::msg::Pose & search_pose);
  ManeuverProbability calculateManeuverProbability(
    const Maneuver & predicted_maneuver, const lanelet::routing::LaneletPaths & left_paths,
    const lanelet::routing::LaneletPaths & right_paths,
    const lanelet::routing::LaneletPaths & center_paths);

  void addReferencePaths(
    const TrackedObject & object, const lanelet::routing::LaneletPaths & candidate_paths,
    const float path_probability, const ManeuverProbability & maneuver_probability,
    const Maneuver & maneuver, std::vector<PredictedRefPath> & reference_paths,
    const double speed_limit = 0.0);
  std::vector<PosePath> convertPathType(const lanelet::routing::LaneletPaths & paths);

  void updateFuturePossibleLanelets(
    const TrackedObject & object, const lanelet::routing::LaneletPaths & paths);

  bool isDuplicated(
    const std::pair<double, lanelet::ConstLanelet> & target_lanelet,
    const LaneletsData & lanelets_data);
  bool isDuplicated(
    const PredictedPath & predicted_path, const std::vector<PredictedPath> & predicted_paths);
  std::optional<lanelet::Id> getTrafficSignalId(const lanelet::ConstLanelet & way_lanelet);
  std::optional<TrafficLightElement> getTrafficSignalElement(const lanelet::Id & id);
  bool calcIntentionToCrossWithTrafficSignal(
    const TrackedObject & object, const lanelet::ConstLanelet & crosswalk,
    const lanelet::Id & signal_id);

  visualization_msgs::msg::Marker getDebugMarker(
    const TrackedObject & object, const Maneuver & maneuver, const size_t obj_num);

  Maneuver predictObjectManeuverByTimeToLaneChange(
    const TrackedObject & object, const LaneletData & current_lanelet_data,
    const double object_detected_time);
  Maneuver predictObjectManeuverByLatDiffDistance(
    const TrackedObject & object, const LaneletData & current_lanelet_data,
    const double object_detected_time);

  // NOTE: This function is copied from the motion_velocity_smoother package.
  // TODO(someone): Consolidate functions and move them to a common
  inline std::vector<double> calcTrajectoryCurvatureFrom3Points(
    const TrajectoryPoints & trajectory, size_t idx_dist)
  {
    using autoware::universe_utils::calcCurvature;
    using autoware::universe_utils::getPoint;

    if (trajectory.size() < 3) {
      const std::vector<double> k_arr(trajectory.size(), 0.0);
      return k_arr;
    }

    // if the idx size is not enough, change the idx_dist
    const auto max_idx_dist = static_cast<size_t>(std::floor((trajectory.size() - 1) / 2.0));
    idx_dist = std::max(1ul, std::min(idx_dist, max_idx_dist));

    if (idx_dist < 1) {
      throw std::logic_error("idx_dist less than 1 is not expected");
    }

    // calculate curvature by circle fitting from three points
    std::vector<double> k_arr(trajectory.size(), 0.0);

    for (size_t i = 1; i + 1 < trajectory.size(); i++) {
      double curvature = 0.0;
      const auto p0 = getPoint(trajectory.at(i - std::min(idx_dist, i)));
      const auto p1 = getPoint(trajectory.at(i));
      const auto p2 = getPoint(trajectory.at(i + std::min(idx_dist, trajectory.size() - 1 - i)));
      try {
        curvature = calcCurvature(p0, p1, p2);
      } catch (std::exception const & e) {
        // ...code that handles the error...
        RCLCPP_WARN(rclcpp::get_logger("motion_forecasting"), "%s", e.what());
        if (i > 1) {
          curvature = k_arr.at(i - 1);  // previous curvature
        } else {
          curvature = 0.0;
        }
      }
      k_arr.at(i) = curvature;
    }
    // copy curvatures for the last and first points;
    k_arr.at(0) = k_arr.at(1);
    k_arr.back() = k_arr.at((trajectory.size() - 2));

    return k_arr;
  }

  inline TrajectoryPoints toTrajectoryPoints(const PredictedPath & path, const double velocity)
  {
    TrajectoryPoints out_trajectory;
    std::for_each(
      path.path.begin(), path.path.end(), [&out_trajectory, velocity](const auto & pose) {
        TrajectoryPoint p;
        p.pose = pose;
        p.longitudinal_velocity_mps = velocity;
        out_trajectory.push_back(p);
      });
    return out_trajectory;
  };

  inline bool isLateralAccelerationConstraintSatisfied(
    const TrajectoryPoints & trajectory, const double delta_time)
  {
    constexpr double epsilon = 1E-6;
    if (delta_time < epsilon) throw std::invalid_argument("delta_time must be a positive value");

    if (trajectory.size() < 3) return true;
    const double max_lateral_accel_abs = std::fabs(max_lateral_accel_);

    double arc_length = 0.0;
    for (size_t i = 1; i < trajectory.size(); ++i) {
      const auto current_pose = trajectory.at(i).pose;
      const auto next_pose = trajectory.at(i - 1).pose;
      // Compute distance between poses
      const double delta_s = std::hypot(
        next_pose.position.x - current_pose.position.x,
        next_pose.position.y - current_pose.position.y);
      arc_length += delta_s;

      // Compute change in heading
      tf2::Quaternion q_current, q_next;
      tf2::convert(current_pose.orientation, q_current);
      tf2::convert(next_pose.orientation, q_next);
      double delta_theta = q_current.angleShortestPath(q_next);
      // Handle wrap-around
      if (delta_theta > M_PI) {
        delta_theta -= 2.0 * M_PI;
      } else if (delta_theta < -M_PI) {
        delta_theta += 2.0 * M_PI;
      }

      const double yaw_rate = std::max(std::abs(delta_theta / delta_time), 1.0E-5);

      const double current_speed = std::abs(trajectory.at(i).longitudinal_velocity_mps);
      // Compute lateral acceleration
      const double lateral_acceleration = std::abs(current_speed * yaw_rate);
      if (lateral_acceleration < max_lateral_accel_abs) continue;
      const double v_curvature_max = std::sqrt(max_lateral_accel_abs / yaw_rate);
      const double t =
        (v_curvature_max - current_speed) / min_acceleration_before_curve_;  // acc is negative
      const double distance_to_slow_down =
        current_speed * t + 0.5 * min_acceleration_before_curve_ * std::pow(t, 2);
      if (distance_to_slow_down > arc_length) return false;
    }
    return true;
  };
};
}  // namespace motion_forecasting

#endif  // motion_forecasting__motion_forecasting_NODE_HPP_

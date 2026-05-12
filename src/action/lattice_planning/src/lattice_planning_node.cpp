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

#include "lattice_planning/lattice_planning_node.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

LatticePlanningNode::LatticePlanningNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("lattice_planning_node", options)
{
  RCLCPP_INFO(this->get_logger(), "Lattice Planner ROS 2 lifecycle node created");
  RCLCPP_INFO(this->get_logger(), "Current state: %s", this->get_current_state().label().c_str());

  // Subscription topics
  lanelet_ahead_topic = this->declare_parameter("lanelet_ahead_topic", "lanelet_ahead");
  odom_topic = this->declare_parameter("odom_topic", "odom");
  bt_topic = this->declare_parameter("bt_topic", "execute_behaviour");

  // Publishing topics
  final_path_topic = this->declare_parameter("path_topic", "path");
  available_paths_topic = this->declare_parameter("available_paths_topic", "available_paths");

  // Control Frequency
  control_rate_hz_ = this->declare_parameter("control_rate_hz", 2.0);

  // Path generation parameters
  //  - Corridor -
  num_lane_switch_horizons = this->declare_parameter("num_lane_switch_horizons", 3);
  lane_switch_lookahead_distances =
    this->declare_parameter("lookahead_distances", std::vector<double>{10.0, 15.0, 20.0});
  centreline_horizon = this->declare_parameter("centreline_horizon", 30.0);
  centreline_velocity_scale = this->declare_parameter("centreline_velocity_scale", 1.0);

  //  - Cost Function -
  cf_params.lateral_movement_weight = this->declare_parameter("cost_function.lateral_movement_weight", 2.0);
  cf_params.physical_limits_weight = this->declare_parameter("cost_function.physical_limits_weight", 4.0);
  cf_params.preferred_lane_cost = this->declare_parameter("cost_function.preferred_lane_cost", 20.0);
  cf_params.unknown_occupancy_cost = this->declare_parameter("cost_function.unknown_occupancy_cost", 50.0);
  cf_params.max_curvature_change = this->declare_parameter("cost_function.max_curvature_change", 0.1);

  //  - Path Generation -
  PathGenParams pg_params;
  pg_params.max_iterations = this->declare_parameter("path_gen.max_iterations", 20);
  pg_params.steps = this->declare_parameter("path_gen.path_steps", 20);
  pg_params.tolerance = this->declare_parameter("path_gen.convergence_tolerance", 0.25);
  pg_params.newton_damping = this->declare_parameter("path_gen.newton_damping", 0.7);
  pg_params.max_step_size = this->declare_parameter("path_gen.max_step_size", 1.0);

  // Spiral Coefficient Equation Constants
  SpiralCoeffConstants constants;
  constants.c1_k0_coeff = this->declare_parameter("spiral_coeffs.c1_k0_coeff", -11.0);
  constants.c1_k1_coeff = this->declare_parameter("spiral_coeffs.c1_k1_coeff", 18.0);
  constants.c1_k2_coeff = this->declare_parameter("spiral_coeffs.c1_k2_coeff", -9.0);
  constants.c1_k3_coeff = this->declare_parameter("spiral_coeffs.c1_k3_coeff", 2.0);
  constants.c1_divisor = this->declare_parameter("spiral_coeffs.c1_divisor", 2.0);

  constants.c2_k0_coeff = this->declare_parameter("spiral_coeffs.c2_k0_coeff", 18.0);
  constants.c2_k1_coeff = this->declare_parameter("spiral_coeffs.c2_k1_coeff", -45.0);
  constants.c2_k2_coeff = this->declare_parameter("spiral_coeffs.c2_k2_coeff", 36.0);
  constants.c2_k3_coeff = this->declare_parameter("spiral_coeffs.c2_k3_coeff", -9.0);
  constants.c2_divisor = this->declare_parameter("spiral_coeffs.c2_divisor", 2.0);

  constants.c3_k0_coeff = this->declare_parameter("spiral_coeffs.c3_k0_coeff", -9.0);
  constants.c3_k1_coeff = this->declare_parameter("spiral_coeffs.c3_k1_coeff", 27.0);
  constants.c3_k2_coeff = this->declare_parameter("spiral_coeffs.c3_k2_coeff", -27.0);
  constants.c3_k3_coeff = this->declare_parameter("spiral_coeffs.c3_k3_coeff", 9.0);
  constants.c3_divisor = this->declare_parameter("spiral_coeffs.c3_divisor", 2.0);

  core_ = std::make_unique<LatticePlanningCore>(constants, pg_params);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LatticePlanningNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring Lattice Planning node");
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(final_path_topic, 10);
  available_paths_pub_ = this->create_publisher<lattice_planning_msgs::msg::PathArray>(available_paths_topic, 10);

  const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
  publish_timer_ = this->create_wall_timer(period, std::bind(&LatticePlanningNode::plan_and_publish_path, this));
  publish_timer_->cancel();

  RCLCPP_INFO(this->get_logger(), "Node configured successfully");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LatticePlanningNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Activating Lattice Planning node");
  lanelet_ahead_sub_ = create_subscription<lanelet_msgs::msg::LaneletAhead>(
    lanelet_ahead_topic, 10, std::bind(&LatticePlanningNode::lanelet_update_callback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 10, std::bind(&LatticePlanningNode::update_vehicle_odom, this, std::placeholders::_1));
  bt_sub_ = create_subscription<behaviour_msgs::msg::ExecuteBehaviour>(
    bt_topic, 10, std::bind(&LatticePlanningNode::set_preferred_lanes, this, std::placeholders::_1));
  path_pub_->on_activate();
  available_paths_pub_->on_activate();
  publish_timer_->reset();
  RCLCPP_INFO(this->get_logger(), "Node Activated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LatticePlanningNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating Lattice Planning node");
  lanelet_ahead_sub_.reset();
  bt_sub_.reset();
  publish_timer_->cancel();
  RCLCPP_INFO(this->get_logger(), "Node deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LatticePlanningNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up Lattice Planning node");
  lanelet_ahead_sub_.reset();
  bt_sub_.reset();
  publish_timer_.reset();
  RCLCPP_INFO(this->get_logger(), "Node cleaned up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LatticePlanningNode::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down Lattice Planning node");
  lanelet_ahead_sub_.reset();
  bt_sub_.reset();
  publish_timer_.reset();
  RCLCPP_INFO(this->get_logger(), "Node shut down");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void LatticePlanningNode::update_vehicle_odom(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  geometry_msgs::msg::PoseStamped ps;
  geometry_msgs::msg::Quaternion q;
  geometry_msgs::msg::Point p;
  PathPoint fp;
  double yaw;

  ps.header = msg->header;
  ps.pose = msg->pose.pose;
  p = ps.pose.position;
  q = ps.pose.orientation;
  yaw = core_->normalise_angle(tf2::getYaw(q));

  if (!car_frenet_point) {
    fp = PathPoint{p.x, p.y, yaw, 0.0};
  } else {
    double ds = core_->get_euc_dist(p.x, p.y, car_pose->pose.position.x, car_pose->pose.position.y);
    double kappa = ds > 0.001 ? (yaw - car_frenet_point->theta) / ds : 0.0;
    fp = PathPoint{p.x, p.y, yaw, kappa};
  }
  car_pose = ps;
  car_frenet_point = fp;
  car_tang_velocity = fabs(msg->twist.twist.linear.x);
}

void LatticePlanningNode::set_preferred_lanes(const behaviour_msgs::msg::ExecuteBehaviour::ConstSharedPtr & msg)
{
  preferred_lanelets.clear();
  for (auto id : msg->preferred_lanelet_ids) {
    preferred_lanelets[id] = 1;
  }
}

void LatticePlanningNode::lanelet_update_callback(const lanelet_msgs::msg::LaneletAhead::ConstSharedPtr & msg)
{
  std::unordered_map<int64_t, lanelet_msgs::msg::Lanelet> lanelets;

  const int64_t curr_id = msg->current_lanelet_id;

  for (const auto & ll : msg->lanelets) {
    lanelets[ll.id] = ll;
  }

  corridor_terminals.clear();
  ego_centrelines_.clear();

  const std::vector<std::vector<int64_t>> id_order = get_id_order(curr_id, lanelets);

  // Compute velocity-scaled horizon once; shared by all ego-lane variants.
  double horizon = centreline_horizon;
  if (car_tang_velocity.has_value() && car_tang_velocity.value() > 0.0) {
    horizon = std::max(centreline_horizon, centreline_horizon * car_tang_velocity.value() * centreline_velocity_scale);
  }

  for (const auto & lane : id_order) {
    if (lane.empty()) continue;

    if (lane.front() == curr_id) {
      // ------------------------------------------------------------------
      // Ego lane (including fork variants): collect centreline up to horizon.
      //
      // All lanes whose sequence starts from curr_id are ego-lane options —
      // this includes the straight-ahead lane AND every fork variant that
      // get_id_order creates when the ego lane has multiple successors.
      // We store each independently so the costmap can score them all.
      // ------------------------------------------------------------------

      std::vector<geometry_msgs::msg::Point> centreline_pts;
      std::vector<int64_t> contributing_ids;
      bool closest_found = false;
      double arc_length = 0.0;
      std::optional<geometry_msgs::msg::Point> prev_pt;

      for (const int64_t ll_id : lane) {
        if (lanelets.count(ll_id) == 0) continue;
        const auto & centerline = lanelets.at(ll_id).centerline;
        // it should only be false if ll_id is not the curr_id
        bool ll_contributed = (ll_id == curr_id);

        for (const auto & pt : centerline) {
          if (!closest_found) {
            if (point_ahead_of_car(pt)) {
              closest_found = true;
            } else {
              continue;
            }
          }

          if (prev_pt.has_value()) {
            arc_length += core_->get_euc_dist(prev_pt->x, prev_pt->y, pt.x, pt.y);
          }

          if (arc_length > horizon) break;

          // Skip near-duplicate points at lanelet boundaries
          if (!centreline_pts.empty()) {
            double dd = core_->get_euc_dist(centreline_pts.back().x, centreline_pts.back().y, pt.x, pt.y);
            if (dd < 0.01) continue;
          }

          centreline_pts.push_back(pt);
          prev_pt = pt;

          // Record this lanelet the first time it contributes a point
          if (!ll_contributed) {
            contributing_ids.push_back(ll_id);
            ll_contributed = true;
          }
        }

        if (arc_length > horizon) break;
      }

      if (!centreline_pts.empty()) {
        ego_centrelines_.emplace_back(std::move(centreline_pts), std::move(contributing_ids));
      }

    } else {
      // ------------------------------------------------------------------
      // Adjacent lane (left or right): generate corridor terminal points
      // at each lookahead horizon for spiral-based lane-change paths.
      // ------------------------------------------------------------------

      int curr_horizon = 0;
      bool closest_found = false;
      double arc_length = 0.0;
      const geometry_msgs::msg::Point * prev_pt = nullptr;

      for (const int64_t ll_id : lane) {
        if (lanelets.count(ll_id) == 0) continue;
        const auto & centerline = lanelets.at(ll_id).centerline;

        for (size_t pt_idx = 0; pt_idx < centerline.size(); pt_idx++) {
          if (curr_horizon >= num_lane_switch_horizons) break;

          const auto & pt = centerline[pt_idx];

          if (!closest_found) {
            if (point_ahead_of_car(pt)) {
              closest_found = true;
            } else {
              continue;
            }
          }

          if (arc_length < lane_switch_lookahead_distances[curr_horizon]) {
            if (prev_pt) {
              arc_length += core_->get_euc_dist(prev_pt->x, prev_pt->y, pt.x, pt.y);
            }
            prev_pt = &pt;
          } else {
            PathPoint t_pt = create_terminal_point(pt, pt_idx, centerline, prev_pt);
            corridor_terminals.push_back({t_pt, ll_id});
            curr_horizon++;
          }
        }
        if (curr_horizon >= num_lane_switch_horizons) break;
      }
    }
  }
}

void LatticePlanningNode::plan_and_publish_path()
{
  std::vector<Path> paths;

  if (!car_frenet_point) {
    RCLCPP_WARN(get_logger(), "Car frenet point not available");
    return;
  }

  // Generate spiral lane-change paths toward each corridor terminal.
  // Corridor terminals only exist for adjacent (non-ego) lanes.
  for (size_t i = 0; i < corridor_terminals.size(); i++) {
    const auto & terminal = corridor_terminals[i];
    std::vector<PathPoint> ft_path = core_->generate_path(car_frenet_point.value(), terminal.first);

    if (ft_path.empty()) {
      RCLCPP_DEBUG(get_logger(), "Path generation failed for corridor terminal %zu", i);
    } else {
      paths.push_back(Path{ft_path, {terminal.second}, 0, 0});
    }
  }

  // Ego-lane centreline paths — full traversal ID list populated above
  for (const auto & [centreline_pts, ll_ids] : ego_centrelines_) {
    std::vector<PathPoint> cl_pts = centreline_to_path_points(centreline_pts);
    if (!cl_pts.empty()) {
      paths.push_back(Path{cl_pts, ll_ids, 0, 0});
    }
  }

  if (paths.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "No valid paths generated (corridor terminals: %zu, ego centrelines: %zu)",
      corridor_terminals.size(),
      ego_centrelines_.size());
    return;
  }

  Path lowest_cost = core_->get_lowest_cost_path(paths, preferred_lanelets, cf_params);
  publish_final_path(lowest_cost);
  publish_available_paths(paths);
}

// ---------------------------------------------------------------------------
// Conversion helper
// ---------------------------------------------------------------------------

std::vector<PathPoint> LatticePlanningNode::centreline_to_path_points(
  const std::vector<geometry_msgs::msg::Point> & centreline)
{
  std::vector<PathPoint> path_points;
  path_points.reserve(centreline.size());

  for (size_t i = 0; i < centreline.size(); i++) {
    const auto & pt = centreline[i];

    // Heading: forward difference; backward at the last point
    double angle = 0.0;
    if (i + 1 < centreline.size()) {
      angle = core_->get_angle_from_pts(pt.x, pt.y, centreline[i + 1].x, centreline[i + 1].y);
    } else if (i > 0) {
      angle = core_->get_angle_from_pts(centreline[i - 1].x, centreline[i - 1].y, pt.x, pt.y);
    }

    // Curvature: dθ/ds via finite difference
    double curvature = 0.0;
    if (i > 0) {
      const auto & prev = centreline[i - 1];
      double prev_angle = core_->get_angle_from_pts(prev.x, prev.y, pt.x, pt.y);
      double ds = core_->get_euc_dist(prev.x, prev.y, pt.x, pt.y);
      if (ds > 0.001) {
        curvature = core_->normalise_angle(angle - prev_angle) / ds;
      }
    }

    path_points.push_back(PathPoint{pt.x, pt.y, angle, curvature});
  }

  return path_points;
}

// ---------------------------------------------------------------------------
// Remaining helpers (unchanged)
// ---------------------------------------------------------------------------

std::vector<std::vector<int64_t>> LatticePlanningNode::get_id_order(
  int64_t curr_id, const std::unordered_map<int64_t, lanelet_msgs::msg::Lanelet> & ll_map)
{
  std::vector<std::vector<int64_t>> id_order;

  auto it_curr = ll_map.find(curr_id);
  if (it_curr == ll_map.end()) return id_order;

  const auto & curr_ll = it_curr->second;
  const int64_t first_lanelet_id[3] = {curr_ll.left_lane_id, curr_ll.id, curr_ll.right_lane_id};

  for (auto id : first_lanelet_id) {
    if (id >= 0) {
      id_order.push_back({id});
    }
  }

  for (size_t lane_idx = 0; lane_idx < id_order.size(); ++lane_idx) {
    while (true) {
      if (id_order[lane_idx].empty()) {
        RCLCPP_ERROR(get_logger(), "Empty lane at index %zu", lane_idx);
        break;
      }

      int64_t current_id = id_order[lane_idx].back();

      auto it = ll_map.find(current_id);
      if (it == ll_map.end()) break;

      const auto & succs = it->second.successor_ids;
      if (succs.empty()) break;

      int64_t succ_id = succs.front();
      if (succ_id < 0 || succ_id == current_id || ll_map.count(succ_id) < 1) break;

      id_order[lane_idx].push_back(succ_id);

      // Forks on ego-lane sequences only: each extra successor becomes a new
      // sequence that shares the same history up to the fork point.
      if (succs.size() > 1 && id_order[lane_idx].front() == curr_id) {
        for (size_t i = 1; i < succs.size(); ++i) {
          if (succs[i] >= 0 && succs[i] != current_id && ll_map.count(succs[i]) > 0) {
            std::vector<int64_t> split_lane = id_order[lane_idx];
            if (!split_lane.empty()) {
              split_lane.back() = succs[i];
              id_order.push_back(split_lane);
            }
          }
        }
      }
    }
  }
  return id_order;
}

bool LatticePlanningNode::point_ahead_of_car(const geometry_msgs::msg::Point & pt)
{
  const auto & car_pos = car_pose->pose.position;
  const auto & car_quat = car_pose->pose.orientation;

  double car_yaw = core_->normalise_angle(tf2::getYaw(car_quat));
  double car_forward_x = std::cos(car_yaw);
  double car_forward_y = std::sin(car_yaw);

  double to_point_x = pt.x - car_pos.x;
  double to_point_y = pt.y - car_pos.y;

  double dot = car_forward_x * to_point_x + car_forward_y * to_point_y;
  return dot > 0.0;
}

PathPoint LatticePlanningNode::create_terminal_point(
  const geometry_msgs::msg::Point & pt,
  size_t pt_idx,
  const std::vector<geometry_msgs::msg::Point> & centerline,
  const geometry_msgs::msg::Point * prev_pt)
{
  double angle = 0.0;
  if ((pt_idx + 1) < centerline.size()) {
    const auto & next_pt = centerline[pt_idx + 1];
    angle = core_->get_angle_from_pts(pt.x, pt.y, next_pt.x, next_pt.y);
  } else if (prev_pt) {
    angle = core_->get_angle_from_pts(prev_pt->x, prev_pt->y, pt.x, pt.y);
  }

  double curvature = 0.0;
  if (prev_pt) {
    double prev_angle = core_->get_angle_from_pts(prev_pt->x, prev_pt->y, pt.x, pt.y);
    double ds = core_->get_euc_dist(prev_pt->x, prev_pt->y, pt.x, pt.y);
    if (ds > 0.001) {
      double dtheta = core_->normalise_angle(angle - prev_angle);
      curvature = dtheta / ds;
    }
  }

  return PathPoint{pt.x, pt.y, angle, curvature};
}

void LatticePlanningNode::publish_final_path(const Path & path)
{
  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = this->now();
  path_msg.header.frame_id = "map";

  // Prepend a point 0.5m behind the first path point so pure pursuit
  // always has the path starting behind the vehicle and doesn't creep.
  if (!path.path.empty()) {
    constexpr double PATH_START_OFFSET = 2.0;
    const auto & first = path.path.front();
    geometry_msgs::msg::PoseStamped behind;
    behind.header = path_msg.header;
    behind.pose.position.x = first.x - PATH_START_OFFSET * std::cos(first.theta);
    behind.pose.position.y = first.y - PATH_START_OFFSET * std::sin(first.theta);
    behind.pose.position.z = 0.0;
    tf2::Quaternion q_behind;
    q_behind.setRPY(0, 0, first.theta);
    behind.pose.orientation = tf2::toMsg(q_behind);
    path_msg.poses.push_back(behind);
  }

  for (const auto & pt : path.path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = pt.x;
    pose.pose.position.y = pt.y;
    pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, pt.theta);
    pose.pose.orientation = tf2::toMsg(q);

    path_msg.poses.push_back(pose);
  }

  path_pub_->publish(path_msg);
}

void LatticePlanningNode::publish_available_paths(const std::vector<Path> & paths)
{
  lattice_planning_msgs::msg::PathArray available_paths;

  if (paths.empty()) {
    RCLCPP_ERROR(get_logger(), "Available paths list is empty");
    return;
  }

  for (const auto & path : paths) {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "map";

    for (const auto & pt : path.path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = pt.x;
      pose.pose.position.y = pt.y;
      pose.pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0, 0, pt.theta);
      pose.pose.orientation = tf2::toMsg(q);

      path_msg.poses.push_back(pose);
    }
    available_paths.paths.push_back(path_msg);
    available_paths.costs.push_back(path.cost);
  }
  available_paths_pub_->publish(available_paths);
}

RCLCPP_COMPONENTS_REGISTER_NODE(LatticePlanningNode)

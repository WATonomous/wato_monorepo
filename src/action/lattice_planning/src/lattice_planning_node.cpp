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

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

LatticePlanningNode::LatticePlanningNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("local_planner_node", options)
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
  publish_rate_hz_ = this->declare_parameter("publish_rate_hz", 10.0);
  min_path_length_ = this->declare_parameter("min_path_length", 35.0);

  // Lane follow mode
  lane_follow_mode_ = this->declare_parameter("lane_follow_mode", true);
  lane_change_lookahead_s_m_ = this->declare_parameter(
    "lane_change_lookahead_distances", std::vector<double>{8.0, 12.0, 16.0});

  // Path generation parameters
  //  - Corridor -
  num_horizons = this->declare_parameter("num_horizons", 3);
  lookahead_s_m = this->declare_parameter("lookahead_distances", std::vector<double>{10.0, 15.0, 20.0});

  //  - Cost Funcation -
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

  // Sprial Coefficient Equation Constants
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
  auto period = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / publish_rate_hz_));
  publish_timer_ = this->create_wall_timer(period, std::bind(&LatticePlanningNode::plan_and_publish_path, this));
  RCLCPP_INFO(this->get_logger(), "Node Activated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LatticePlanningNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating Lattice Planning node");
  publish_timer_.reset();
  lanelet_ahead_sub_.reset();
  bt_sub_.reset();
  RCLCPP_INFO(this->get_logger(), "Node deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LatticePlanningNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up Lattice Planning node");
  lanelet_ahead_sub_.reset();
  bt_sub_.reset();
  RCLCPP_INFO(this->get_logger(), "Node cleaned up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LatticePlanningNode::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down Lattice Planning node");
  lanelet_ahead_sub_.reset();
  bt_sub_.reset();
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
}

void LatticePlanningNode::set_preferred_lanes(const behaviour_msgs::msg::ExecuteBehaviour::ConstSharedPtr & msg)
{
  bt_behaviour_ = msg->behaviour;
  preferred_lanelets.clear();
  for (auto id : msg->preferred_lanelet_ids) {
    preferred_lanelets[id] = 1;
  }
}

void LatticePlanningNode::lanelet_update_callback(const lanelet_msgs::msg::LaneletAhead::ConstSharedPtr & msg)
{
  std::unordered_map<int64_t, lanelet_msgs::msg::Lanelet> lanelets;
  std::vector<std::vector<int64_t>> id_order;

  const int64_t curr_id = msg->current_lanelet_id;

  // Build map of all lanelets in msg
  for (const auto & ll : msg->lanelets) {
    lanelets[ll.id] = ll;
  }

  // Cache for centerline extension
  cached_lanelets_ = lanelets;
  cached_current_lanelet_id_ = curr_id;

  // Clear previous terminals
  corridor_terminals.clear();

  // Get the order of ids for each lane
  id_order = get_id_order(curr_id, lanelets);

  // Search lanes for terminal points at horizons
  for (size_t lane_idx = 0; lane_idx < id_order.size(); lane_idx++) {
    const auto & lane = id_order[lane_idx];

    int curr_horizon = 0;
    bool closest_found = false;
    const geometry_msgs::msg::Point * prev_pt = nullptr;
    const auto & car_pos = car_pose->pose.position;

    for (const int64_t ll_id : lane) {
      const auto & centerline = lanelets.at(ll_id).centerline;

      for (size_t pt_idx = 0; pt_idx < centerline.size(); pt_idx++) {
        if (curr_horizon >= num_horizons) break;

        const auto & pt = centerline[pt_idx];

        if (!closest_found) {
          if (point_ahead_of_car(pt)) {
            closest_found = true;
          } else {
            continue;
          }
        }

        double euc_dist = core_->get_euc_dist(car_pos.x, car_pos.y, pt.x, pt.y);

        if (euc_dist >= lookahead_s_m[curr_horizon]) {
          // Interpolate to exact Euclidean lookahead distance
          if (prev_pt) {
            double prev_dist = core_->get_euc_dist(car_pos.x, car_pos.y, prev_pt->x, prev_pt->y);
            double frac = (lookahead_s_m[curr_horizon] - prev_dist) / (euc_dist - prev_dist);
            frac = std::clamp(frac, 0.0, 1.0);

            geometry_msgs::msg::Point interp_pt;
            interp_pt.x = prev_pt->x + frac * (pt.x - prev_pt->x);
            interp_pt.y = prev_pt->y + frac * (pt.y - prev_pt->y);
            interp_pt.z = prev_pt->z + frac * (pt.z - prev_pt->z);

            PathPoint t_pt = create_terminal_point(interp_pt, pt_idx, centerline, prev_pt);
            corridor_terminals.push_back({t_pt, ll_id});
          } else {
            PathPoint t_pt = create_terminal_point(pt, pt_idx, centerline, nullptr);
            corridor_terminals.push_back({t_pt, ll_id});
          }
          curr_horizon++;
        } else {
          prev_pt = &pt;
        }
      }
      if (curr_horizon >= num_horizons) break;
    }
  }
  plan_and_publish_path();
}

bool LatticePlanningNode::is_lane_change_requested() const
{
  return bt_behaviour_.find("lane change") != std::string::npos;
}

void LatticePlanningNode::plan_and_publish_path()
{
  if (!car_frenet_point) {
    RCLCPP_WARN(get_logger(), "Car frenet point not available");
    return;
  }

  if (lane_follow_mode_ && !is_lane_change_requested()) {
    plan_lane_follow();
  } else {
    plan_lane_change();
  }
}

void LatticePlanningNode::plan_lane_follow()
{
  // Build path directly from centerline — no spiral optimization
  Path path;
  path.target_lanelet_id = cached_current_lanelet_id_;
  path.cost = 0.0;

  // Start with current vehicle position
  path.path.push_back(car_frenet_point.value());

  // Extend with centerline to min_path_length
  extend_path_with_centerline(path);

  publish_final_path(path);
  publish_available_paths({path});
}

void LatticePlanningNode::plan_lane_change()
{
  // Use multi-horizon lookaheads for lane change spiral candidates
  // Temporarily swap lookahead distances if lane_change_lookaheads are configured
  std::vector<double> original_lookaheads = lookahead_s_m;
  int original_num_horizons = num_horizons;

  if (!lane_change_lookahead_s_m_.empty()) {
    lookahead_s_m = lane_change_lookahead_s_m_;
    num_horizons = static_cast<int>(lane_change_lookahead_s_m_.size());

    // Re-generate corridor terminals with new horizons
    // Trigger recomputation by re-processing cached lanelets
    if (cached_current_lanelet_id_ >= 0 && !cached_lanelets_.empty()) {
      corridor_terminals.clear();
      auto id_order = get_id_order(cached_current_lanelet_id_, cached_lanelets_);

      for (size_t lane_idx = 0; lane_idx < id_order.size(); lane_idx++) {
        const auto & lane = id_order[lane_idx];
        int curr_horizon = 0;
        bool closest_found = false;
        const geometry_msgs::msg::Point * prev_pt = nullptr;
        const auto & car_pos = car_pose->pose.position;

        for (const int64_t ll_id : lane) {
          const auto & centerline = cached_lanelets_.at(ll_id).centerline;
          for (size_t pt_idx = 0; pt_idx < centerline.size(); pt_idx++) {
            if (curr_horizon >= num_horizons) break;
            const auto & pt = centerline[pt_idx];
            if (!closest_found) {
              if (point_ahead_of_car(pt)) {
                closest_found = true;
              } else {
                continue;
              }
            }
            double euc_dist = core_->get_euc_dist(car_pos.x, car_pos.y, pt.x, pt.y);
            if (euc_dist >= lookahead_s_m[curr_horizon]) {
              if (prev_pt) {
                double prev_dist = core_->get_euc_dist(car_pos.x, car_pos.y, prev_pt->x, prev_pt->y);
                double frac = (lookahead_s_m[curr_horizon] - prev_dist) / (euc_dist - prev_dist);
                frac = std::clamp(frac, 0.0, 1.0);
                geometry_msgs::msg::Point interp_pt;
                interp_pt.x = prev_pt->x + frac * (pt.x - prev_pt->x);
                interp_pt.y = prev_pt->y + frac * (pt.y - prev_pt->y);
                interp_pt.z = prev_pt->z + frac * (pt.z - prev_pt->z);
                PathPoint t_pt = create_terminal_point(interp_pt, pt_idx, centerline, prev_pt);
                corridor_terminals.push_back({t_pt, ll_id});
              } else {
                PathPoint t_pt = create_terminal_point(pt, pt_idx, centerline, nullptr);
                corridor_terminals.push_back({t_pt, ll_id});
              }
              curr_horizon++;
            } else {
              prev_pt = &pt;
            }
          }
          if (curr_horizon >= num_horizons) break;
        }
      }
    }
  }

  // Generate spiral paths for each terminal (original logic)
  std::vector<Path> paths;
  for (size_t i = 0; i < corridor_terminals.size(); i++) {
    auto & terminal = corridor_terminals[i];
    std::vector<PathPoint> ft_path = core_->generate_path(car_frenet_point.value(), terminal.first);
    if (!ft_path.empty()) {
      Path path{ft_path, terminal.second, 0, 0};
      paths.push_back(path);
    }
  }

  // Restore original parameters
  lookahead_s_m = original_lookaheads;
  num_horizons = original_num_horizons;

  if (paths.empty()) {
    RCLCPP_WARN(get_logger(), "No valid lane change paths, falling back to lane follow");
    plan_lane_follow();
    return;
  }

  Path lowest_cost = core_->get_lowest_cost_path(paths, preferred_lanelets, cf_params);
  extend_path_with_centerline(lowest_cost);
  publish_final_path(lowest_cost);
  publish_available_paths(paths);
}

void LatticePlanningNode::extend_path_with_centerline(Path & path)
{
  if (path.path.empty() || cached_current_lanelet_id_ < 0) return;

  // Compute current path arc length
  double path_len = 0.0;
  for (size_t i = 1; i < path.path.size(); ++i) {
    path_len += core_->get_euc_dist(
      path.path[i - 1].x, path.path[i - 1].y, path.path[i].x, path.path[i].y);
  }
  if (path_len >= min_path_length_) return;

  // Build ego lane sequence
  auto id_order = get_id_order(cached_current_lanelet_id_, cached_lanelets_);
  if (id_order.empty()) return;

  // Find the ego lane
  const std::vector<int64_t> * ego_lane = nullptr;
  for (const auto & lane : id_order) {
    if (!lane.empty() && lane.front() == cached_current_lanelet_id_) {
      ego_lane = &lane;
      break;
    }
  }
  if (!ego_lane) ego_lane = &id_order[0];

  // Find the closest centerline point to the end of the spiral
  const auto & last_pt = path.path.back();
  double min_dist = std::numeric_limits<double>::max();
  size_t best_ll_idx = 0;
  size_t best_pt_idx = 0;
  bool found = false;

  for (size_t li = 0; li < ego_lane->size(); ++li) {
    auto it = cached_lanelets_.find((*ego_lane)[li]);
    if (it == cached_lanelets_.end()) continue;
    const auto & cl = it->second.centerline;
    for (size_t pi = 0; pi < cl.size(); ++pi) {
      double d = core_->get_euc_dist(last_pt.x, last_pt.y, cl[pi].x, cl[pi].y);
      if (d < min_dist) {
        min_dist = d;
        best_ll_idx = li;
        best_pt_idx = pi;
        found = true;
      }
    }
  }
  if (!found) return;

  // Append centerline points after the closest point until we reach min_path_length
  bool started = false;
  for (size_t li = best_ll_idx; li < ego_lane->size(); ++li) {
    auto it = cached_lanelets_.find((*ego_lane)[li]);
    if (it == cached_lanelets_.end()) break;
    const auto & cl = it->second.centerline;

    size_t start_pi = started ? 0 : best_pt_idx + 1;
    started = true;

    for (size_t pi = start_pi; pi < cl.size(); ++pi) {
      const auto & pt = cl[pi];
      double seg = core_->get_euc_dist(
        path.path.back().x, path.path.back().y, pt.x, pt.y);
      if (seg < 0.1) continue;  // skip near-duplicate points

      // Compute heading from previous point
      double angle = core_->get_angle_from_pts(path.path.back().x, path.path.back().y, pt.x, pt.y);
      path.path.push_back(PathPoint{pt.x, pt.y, angle, 0.0});
      path_len += seg;

      if (path_len >= min_path_length_) return;
    }
  }
}

std::vector<std::vector<int64_t>> LatticePlanningNode::get_id_order(
  int64_t curr_id, const std::unordered_map<int64_t, lanelet_msgs::msg::Lanelet> & ll_map)
{
  std::vector<std::vector<int64_t>> id_order;

  auto it_curr = ll_map.find(curr_id);
  if (it_curr == ll_map.end()) return id_order;

  const auto & curr_ll = it_curr->second;
  const int64_t first_lanelet_id[3] = {curr_ll.left_lane_id, curr_ll.id, curr_ll.right_lane_id};

  // Initialize starting lanes, tracking which index is the ego lane
  size_t ego_lane_idx = 0;
  for (auto id : first_lanelet_id) {
    if (id >= 0) {
      if (id == curr_id) ego_lane_idx = id_order.size();
      id_order.push_back({id});
    }
  }

  // Use index-based loop to allow safe addition during iteration
  for (size_t lane_idx = 0; lane_idx < id_order.size(); ++lane_idx) {
    while (true) {
      // Check empty at the start of each iteration
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

      // Add first successor
      id_order[lane_idx].push_back(succ_id);

      // Handle splits for ego lane only
      if (succs.size() > 1 && lane_idx == ego_lane_idx) {
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

  // use dot product to find if point lies ahead or behind the car
  double dot = car_forward_x * to_point_x + car_forward_y * to_point_y;

  return dot > 0.0;
}

PathPoint LatticePlanningNode::create_terminal_point(
  const geometry_msgs::msg::Point & pt,
  size_t pt_idx,
  const std::vector<geometry_msgs::msg::Point> & centerline,
  const geometry_msgs::msg::Point * prev_pt)
{
  // Calculate heading
  double angle = 0.0;
  if ((pt_idx + 1) < centerline.size()) {
    // Use forward difference
    const auto & next_pt = centerline[pt_idx + 1];
    angle = core_->get_angle_from_pts(pt.x, pt.y, next_pt.x, next_pt.y);
  } else if (prev_pt) {
    // Use backward difference
    angle = core_->get_angle_from_pts(prev_pt->x, prev_pt->y, pt.x, pt.y);
  }

  // Calculate curvature
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

  for (const auto & pt : path.path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = pt.x;
    pose.pose.position.y = pt.y;
    pose.pose.position.z = 0.0;

    // Convert heading to quaternion
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

  for (auto path : paths) {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "map";

    for (const auto & pt : path.path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = pt.x;
      pose.pose.position.y = pt.y;
      pose.pose.position.z = 0.0;

      // Convert heading to quaternion
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

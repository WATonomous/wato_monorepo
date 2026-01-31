#pragma once

#include <iostream>
#include <vector>
#include <optional>

#include "local_planning/local_planner_core.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "lanelet_msgs/msg/route_ahead.hpp"


struct FrenetPath{
  std::vector<FrenetPoint> path;
  int target_lanelet_id;
  double lateral_dist_from_goal_lane;
  double cost;
};

class LocalPlannerNode : public rclcpp_lifecycle::LifecycleNode
{
public:

  explicit LocalPlannerNode(const rclcpp::NodeOptions & options);

  static constexpr auto route_ahead_topic = "/world_modeling/lanelet/route_ahead";
  static constexpr auto odom_topic = "/ego/odom";

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  
  LocalPlannerCore core_;

  // corridor construction
  static constexpr int num_horizons = 1;
  static constexpr double lookahead_s_m[num_horizons] = {5.0};
  static constexpr int lateral_samples = 3;

  FrenetPoint corridor_terminals[num_horizons][lateral_samples] = {};

  std::optional<geometry_msgs::msg::PoseStamped> car_pose;
  std::optional<FrenetPoint> car_frenet_point;

  double distance_along_first_lanelet(const lanelet_msgs::msg::Lanelet & ll);

  void create_corridor(const lanelet_msgs::msg::RouteAhead::ConstSharedPtr & msg);
  void update_vehicle_odom(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void timer_callback();
  void publish_paths_vis(std::vector<FrenetPath> paths);
  
  rclcpp::Subscription<lanelet_msgs::msg::RouteAhead>::SharedPtr route_ahead_sub_;
  rclcpp::Subscription<lanelet_msgs::msg::RouteAhead>::SharedPtr lanelet_ahead_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_vis_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  
};
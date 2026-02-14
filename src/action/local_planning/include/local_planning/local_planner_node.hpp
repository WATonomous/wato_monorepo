#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <optional>

#include "local_planning/local_planner_core.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

#include "local_planning_msgs/msg/path_array.hpp"

#include "lanelet_msgs/msg/route_ahead.hpp"
#include "lanelet_msgs/msg/lanelet_ahead.hpp"

#include "behaviour_msgs/msg/execute_behaviour.hpp"


class LocalPlannerNode : public rclcpp_lifecycle::LifecycleNode
{
public:

  LocalPlannerNode(const rclcpp::NodeOptions & options);

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
  
  // path generation functions
  bool point_ahead_of_car(const geometry_msgs::msg::Point & pt);
  
  void plan_and_publish_path();

  PathPoint create_terminal_point(
    const geometry_msgs::msg::Point& pt,
    size_t pt_idx,
    const std::vector<geometry_msgs::msg::Point>& centerline,
     const geometry_msgs::msg::Point* prev_pt
  );

  std::vector<std::vector<int64_t>> get_id_order(
    int64_t curr_id, 
    const std::unordered_map<int64_t, 
    lanelet_msgs::msg::Lanelet> & ll_map
  ); 
  

  // subscriber callbacks
  void lanelet_update_callback(const lanelet_msgs::msg::LaneletAhead::ConstSharedPtr & msg);
  void update_vehicle_odom(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void set_preferred_lanes(const behaviour_msgs::msg::ExecuteBehaviour::ConstSharedPtr & msg);
  
  // publisher wrappers
  void publish_final_path(const Path & path);
  void publish_planned_paths_vis(const std::vector<Path> & paths);
  void publish_final_path_vis(const Path & path);
  void publish_available_paths(const std::vector<Path> & paths);
  
  LocalPlannerCore core_;

  // subscription topic names
  std::string lanelet_ahead_topic, odom_topic, bt_topic;
  
  // publisher topic names
  std::string planned_paths_vis_topic, final_path_vis_topic, final_path_topic, available_paths_topic;

  // parameter structs
  CostFunctionParams cf_params;
  PathGenParams pg_params;

  // corridor construction
  int num_horizons;
  std::vector<double> lookahead_s_m; // in metres
  std::vector<std::pair<PathPoint, int64_t>> corridor_terminals;

  std::optional<geometry_msgs::msg::PoseStamped> car_pose;
  std::optional<PathPoint> car_frenet_point;
  std::unordered_map<int64_t, int> preferred_lanelets;

  // subscribers
  rclcpp::Subscription<lanelet_msgs::msg::RouteAhead>::SharedPtr route_ahead_sub_;
  rclcpp::Subscription<lanelet_msgs::msg::LaneletAhead>::SharedPtr lanelet_ahead_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<behaviour_msgs::msg::ExecuteBehaviour>::SharedPtr bt_sub_; 

  // publishers
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr planned_path_vis_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr final_path_vis_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<local_planning_msgs::msg::PathArray>::SharedPtr available_paths_pub_;
 
};
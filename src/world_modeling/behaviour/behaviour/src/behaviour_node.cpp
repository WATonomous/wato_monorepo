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

#include "behaviour/behaviour_node.hpp"

#include <chrono>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "behaviour/utils/types.hpp"

namespace behaviour
{

/**
 * @brief Constructor for BehaviourNode.
 */

BehaviourNode::BehaviourNode(const rclcpp::NodeOptions & options)
: Node("behaviour_node", options)
{
  // Declare parameters
  this->declare_parameter("bt_tree_file", "main_tree.xml");
  this->declare_parameter("rate_hz", 10.0);
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("base_frame", "base_link");
  this->declare_parameter("enable_console_logging", false);
  this->declare_parameter("traffic_light_state_hypothesis_index", 1);
  this->declare_parameter("world_objects_hypothesis_index", 0);
  this->declare_parameter("bt.left_lane_change_areas", std::vector<std::string>{"left_lane_change_corridor"});
  this->declare_parameter("bt.right_lane_change_areas", std::vector<std::string>{"right_lane_change_corridor"});
  this->declare_parameter("bt.intersection_wall_of_doom_width", 5.0);
  this->declare_parameter("bt.intersection_wall_of_doom_length", 1.0);
  this->declare_parameter("bt.stop_sign_ego_stop_line_threshold_m", 3.0);
  this->declare_parameter("bt.ego_stopped_velocity_threshold", 0.1);
  this->declare_parameter("bt.intersection_lookahead_m", 100.0);
  this->declare_parameter("bt.goal_reached_mode", "lanelet");
  this->declare_parameter("bt.goal_reached_threshold_m", 1.0);
  this->declare_parameter("service_timeout_ms", 6000);
  this->declare_parameter("wait_for_service_timeout_ms", 60000);

  // Init TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

/**
 * @brief Separate init function to pass node to behaviour tree after construction.
 */

void BehaviourNode::init()
{
  load_params();
  rebuild_tree();

  auto tick_period = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / rate_hz_));
  // timer to tick the behaviour tree
  tick_tree_timer_ = this->create_wall_timer(tick_period, std::bind(&BehaviourNode::tick_tree_callback, this));

  reset_bt_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "reset_bt", std::bind(&BehaviourNode::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

  // subscribers
  goal_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "goal_point", 10, std::bind(&BehaviourNode::goal_point_callback, this, std::placeholders::_1));

  current_lane_context_sub_ = this->create_subscription<lanelet_msgs::msg::CurrentLaneContext>(
    "lane_context", 10, std::bind(&BehaviourNode::lane_context_callback, this, std::placeholders::_1));

  route_ahead_sub_ = this->create_subscription<lanelet_msgs::msg::RouteAhead>(
    "route_ahead", 10, std::bind(&BehaviourNode::route_ahead_callback, this, std::placeholders::_1));

  lanelets_ahead_sub_ = this->create_subscription<lanelet_msgs::msg::LaneletAhead>(
    "lanelet_ahead", 10, std::bind(&BehaviourNode::lanelet_ahead_callback, this, std::placeholders::_1));

  ego_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "ego/odom", rclcpp::QoS(10), std::bind(&BehaviourNode::odom_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "BehaviourNode has been fully initialized.");
}

void BehaviourNode::load_params()
{
  rate_hz_ = this->get_parameter("rate_hz").as_double();
  map_frame_ = this->get_parameter("map_frame").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();

  // params
  enable_console_logging_ = this->get_parameter("enable_console_logging").as_bool();
  traffic_light_state_hypothesis_index_ =
    static_cast<std::size_t>(this->get_parameter("traffic_light_state_hypothesis_index").as_int());
  world_objects_hypothesis_index_ =
    static_cast<std::size_t>(this->get_parameter("world_objects_hypothesis_index").as_int());
  left_lane_change_areas_ = this->get_parameter("bt.left_lane_change_areas").as_string_array();
  right_lane_change_areas_ = this->get_parameter("bt.right_lane_change_areas").as_string_array();
  stop_line_wall_width_ = this->get_parameter("bt.intersection_wall_of_doom_width").as_double();
  stop_line_wall_length_ = this->get_parameter("bt.intersection_wall_of_doom_length").as_double();
  stop_sign_ego_stop_line_threshold_m_ = this->get_parameter("bt.stop_sign_ego_stop_line_threshold_m").as_double();
  ego_stopped_velocity_threshold_ = this->get_parameter("bt.ego_stopped_velocity_threshold").as_double();
  intersection_lookahead_m_ = this->get_parameter("bt.intersection_lookahead_m").as_double();
  goal_reached_mode_ = this->get_parameter("bt.goal_reached_mode").as_string();
  goal_reached_threshold_m_ = this->get_parameter("bt.goal_reached_threshold_m").as_double();

  // tree file path
  const std::string package_share_directory = ament_index_cpp::get_package_share_directory("behaviour");
  const std::string bt_xml_name = this->get_parameter("bt_tree_file").as_string();
  tree_file_path_ = (std::filesystem::path(package_share_directory) / "trees" / bt_xml_name).string();
}

void BehaviourNode::rebuild_tree()
{
  tree_ = std::make_shared<BehaviourTree>(this->shared_from_this(), tree_file_path_, enable_console_logging_);

  // Static BT configuration that would normally be rebuilt on node startup.
  tree_->updateBlackboard("tf_buffer", tf_buffer_);
  tree_->updateBlackboard("map_frame", map_frame_);
  tree_->updateBlackboard("base_frame", base_frame_);
  tree_->updateBlackboard("world_objects_hypothesis_index", world_objects_hypothesis_index_);
  tree_->updateBlackboard("traffic_light_state_hypothesis_index", traffic_light_state_hypothesis_index_);
  tree_->updateBlackboard("overtake.stage", types::OvertakeStage::IDLE);
  tree_->updateBlackboard("bt.left_lane_change_areas", left_lane_change_areas_);
  tree_->updateBlackboard("bt.right_lane_change_areas", right_lane_change_areas_);
  tree_->updateBlackboard("bt.intersection_wall_of_doom_width", stop_line_wall_width_);
  tree_->updateBlackboard("bt.intersection_wall_of_doom_length", stop_line_wall_length_);
  tree_->updateBlackboard("bt.stop_sign_ego_stop_line_threshold_m", stop_sign_ego_stop_line_threshold_m_);
  tree_->updateBlackboard("bt.ego_stopped_velocity_threshold", ego_stopped_velocity_threshold_);
  tree_->updateBlackboard("bt.intersection_lookahead_m", intersection_lookahead_m_);
  tree_->updateBlackboard("bt.goal_reached_mode", goal_reached_mode_);
  tree_->updateBlackboard("bt.goal_reached_threshold_m", goal_reached_threshold_m_);
}

void BehaviourNode::reset_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;

  try {
    load_params();
    rebuild_tree();

    response->success = true;
    response->message = "Behaviour tree reset complete.";
    RCLCPP_INFO(this->get_logger(), "Behaviour tree reset via reset_bt service.");
  } catch (const std::exception & e) {
    response->success = false;
    response->message = std::string("Failed to reset behaviour tree: ") + e.what();
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
  }
}

void BehaviourNode::goal_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  tree_->updateBlackboard("requested_gp", msg);
  RCLCPP_INFO(this->get_logger(), "New goal received: x=%.2f, y=%.2f", msg->point.x, msg->point.y);
}

void BehaviourNode::lane_context_callback(const lanelet_msgs::msg::CurrentLaneContext::SharedPtr msg)
{
  tree_->updateBlackboard("current_lane_ctx", msg);
}

void BehaviourNode::route_ahead_callback(const lanelet_msgs::msg::RouteAhead::SharedPtr msg)
{
  auto route_ahead_index_map = std::make_shared<std::unordered_map<int64_t, std::size_t>>();
  route_ahead_index_map->reserve(msg->lanelets.size());
  for (std::size_t i = 0; i < msg->lanelets.size(); ++i) {
    (*route_ahead_index_map)[msg->lanelets[i].id] = i;
  }

  tree_->updateBlackboard("route_ahead", msg);
  tree_->updateBlackboard("route_ahead_index_map", route_ahead_index_map);
}

void BehaviourNode::lanelet_ahead_callback(const lanelet_msgs::msg::LaneletAhead::SharedPtr msg)
{
  auto lanelets_ahead_index_map = std::make_shared<std::unordered_map<int64_t, std::size_t>>();
  lanelets_ahead_index_map->reserve(msg->lanelets.size());
  for (std::size_t i = 0; i < msg->lanelets.size(); ++i) {
    (*lanelets_ahead_index_map)[msg->lanelets[i].id] = i;
  }

  tree_->updateBlackboard("lanelets_ahead", msg);
  tree_->updateBlackboard("lanelets_ahead_index_map", lanelets_ahead_index_map);
}

void BehaviourNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  tree_->updateBlackboard("ego_odom", msg);
}

/**
 * @brief Ticks the behavior tree.
 */
void BehaviourNode::tick_tree_callback()
{
  try {
    // tick tree
    tree_->tick();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Behavior Tree tick failed: %s", e.what());
  }
}

}  // namespace behaviour

/**
 * @brief Main entry point for the behaviour node.
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<behaviour::BehaviourNode>();
  node->init();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

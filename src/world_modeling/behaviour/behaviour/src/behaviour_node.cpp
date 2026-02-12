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
#include <memory>
#include <string>
#include <vector>

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
  this->declare_parameter("bt.traffic_light_state_hypothesis_index", 1);
  this->declare_parameter("world_objects_hypothesis_index", 0);
  this->declare_parameter("bt.left_lane_change_areas", std::vector<std::string>{"left_lane_change_corridor"});
  this->declare_parameter("bt.right_lane_change_areas", std::vector<std::string>{"right_lane_change_corridor"});
  this->declare_parameter("bt.intersection_wall_of_doom_width", 5.0);
  this->declare_parameter("bt.intersection_wall_of_doom_length", 1.0);
  this->declare_parameter("bt.ego_stopped_velocity_threshold", 0.1);
  this->declare_parameter("bt.intersection_lookahead_m", 40.0);
  this->declare_parameter("bt.traffic_control_element_passed_lanelet_threshold", 2);
  this->declare_parameter("bt.traffic_control_element_handled_lanelet_threshold", 1);
  this->declare_parameter("bt.stop_sign_car_detection_threshold_m", 8.0);
  this->declare_parameter("bt.goal_reached_threshold_m", 1.0);
  this->declare_parameter("get_shortest_route_timeout_ms", 6000);
  this->declare_parameter("set_route_timeout_ms", 6000);
  this->declare_parameter("get_lanelets_by_reg_elem_timeout_ms", 5000);
  this->declare_parameter("wall_service_timeout_ms", 5000);

  // Init TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

/**
   * @brief Separate init function to pass node to behaviour tree after construction.
   */

void BehaviourNode::init()
{
  map_frame_ = this->get_parameter("map_frame").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();
  double tick_rate_hz = this->get_parameter("rate_hz").as_double();
  bool enable_console_logging = this->get_parameter("enable_console_logging").as_bool();
  int traffic_light_state_hypothesis_index = this->get_parameter("bt.traffic_light_state_hypothesis_index").as_int();
  int world_objects_hypothesis_index = this->get_parameter("world_objects_hypothesis_index").as_int();
  std::vector<std::string> left_lane_change_areas = this->get_parameter("bt.left_lane_change_areas").as_string_array();
  std::vector<std::string> right_lane_change_areas =this->get_parameter("bt.right_lane_change_areas").as_string_array();
  double stop_line_wall_width = this->get_parameter("bt.intersection_wall_of_doom_width").as_double();
  double stop_line_wall_length = this->get_parameter("bt.intersection_wall_of_doom_length").as_double();
  double ego_stopped_velocity_threshold = this->get_parameter("bt.ego_stopped_velocity_threshold").as_double();
  double intersection_lookahead_m = this->get_parameter("bt.intersection_lookahead_m").as_double();
  int traffic_control_passed_threshold =
    this->get_parameter("bt.traffic_control_element_passed_lanelet_threshold").as_int();
  int traffic_control_handled_threshold =
    this->get_parameter("bt.traffic_control_element_handled_lanelet_threshold").as_int();
  double stop_sign_car_detection_threshold_m =
    this->get_parameter("bt.stop_sign_car_detection_threshold_m").as_double();
  double goal_reached_threshold_m = this->get_parameter("bt.goal_reached_threshold_m").as_double();

  // behaviour tree file path
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("behaviour");
  std::string bt_xml_name = this->get_parameter("bt_tree_file").as_string();
  std::filesystem::path tree_path = std::filesystem::path(package_share_directory) / "trees" / bt_xml_name;

  // create the tree
  tree_ = std::make_shared<BehaviourTree>(this->shared_from_this(), tree_path.string(), enable_console_logging);

  // set TF and frames on blackboard to be used by BT nodes for performing transforms between points
  tree_->updateBlackboard("tf_buffer", tf_buffer_);
  tree_->updateBlackboard("map_frame", map_frame_);
  tree_->updateBlackboard("base_frame", base_frame_);

  // xml specific values
  tree_->updateBlackboard("bt.traffic_light_state_hypothesis_index", traffic_light_state_hypothesis_index);
  tree_->updateBlackboard("bt.left_lane_change_areas", left_lane_change_areas);
  tree_->updateBlackboard("bt.right_lane_change_areas", right_lane_change_areas);
  tree_->updateBlackboard("bt.intersection_wall_of_doom_width", stop_line_wall_width);
  tree_->updateBlackboard("bt.intersection_wall_of_doom_length", stop_line_wall_length);
  tree_->updateBlackboard("bt.ego_stopped_velocity_threshold", ego_stopped_velocity_threshold);
  tree_->updateBlackboard("bt.intersection_lookahead_m", intersection_lookahead_m);
  tree_->updateBlackboard("bt.traffic_control_element_passed_lanelet_threshold", traffic_control_passed_threshold);
  tree_->updateBlackboard("bt.traffic_control_element_handled_lanelet_threshold", traffic_control_handled_threshold);
  tree_->updateBlackboard("bt.stop_sign_car_detection_threshold_m", stop_sign_car_detection_threshold_m);
  tree_->updateBlackboard("bt.goal_reached_threshold_m", goal_reached_threshold_m);

  // stores for world state publishers
  dynamic_objects_store_ = std::make_shared<behaviour::DynamicObjectStore>(world_objects_hypothesis_index);
  area_occupancy_store_ = std::make_shared<behaviour::AreaOccupancyStore>();

  auto tick_period = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / tick_rate_hz));
  // timer to tick the behaviour tree
  tick_tree_timer_ = this->create_wall_timer(tick_period, std::bind(&BehaviourNode::tickTreeTimerCallback, this));

  // subscribers
  goal_point_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
    "goal_point", 10, [this](const geometry_msgs::msg::Point::SharedPtr msg) {
      tree_->updateBlackboard("goal_point", msg);
      RCLCPP_INFO(this->get_logger(), "New goal received: x=%.2f, y=%.2f", msg->x, msg->y);
    });

  current_lane_context_sub_ = this->create_subscription<lanelet_msgs::msg::CurrentLaneContext>(
    "lane_context", 10, [this](const lanelet_msgs::msg::CurrentLaneContext::SharedPtr msg) {
      tree_->updateBlackboard("current_lane_ctx", msg);
    });

  dynamic_objects_sub_ = this->create_subscription<world_model_msgs::msg::WorldObjectArray>(
    "world_objects", rclcpp::QoS(10), [this](world_model_msgs::msg::WorldObjectArray::ConstSharedPtr msg) {
      dynamic_objects_store_->update(msg);
    });

  area_occupancy_sub_ = this->create_subscription<world_model_msgs::msg::AreaOccupancy>(
    "area_occupancy", rclcpp::QoS(10), [this](world_model_msgs::msg::AreaOccupancy::ConstSharedPtr msg) {
      area_occupancy_store_->update(msg);
    });

  ego_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "ego/odom", rclcpp::QoS(10), [this](nav_msgs::msg::Odometry::SharedPtr msg) {
      tree_->updateBlackboard("ego_odom", msg);
    });

  RCLCPP_INFO(this->get_logger(), "BehaviourNode has been fully initialized.");
}

/**
   * @brief Ticks the behavior tree.
   */
void BehaviourNode::tickTreeTimerCallback()
{
  try {
    // send latest world state snapshots to blackboard
    tree_->updateBlackboard("dynamic_objects.snapshot", dynamic_objects_store_->snapshot());
    tree_->updateBlackboard("area_occupancy.snapshot", area_occupancy_store_->snapshot());

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

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

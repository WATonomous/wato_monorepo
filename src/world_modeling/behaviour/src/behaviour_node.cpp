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

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "behaviour/dynamic_object_store.hpp"
#include "behaviour/interfaces/subscribers/dynamic_objects_subscriber.hpp"
#include "behaviour/interfaces/subscribers/goal_point_subscriber.hpp"
#include "behaviour/interfaces/subscribers/lane_context_subscriber.hpp"
#include "behaviour/interfaces/subscribers/odometry_subscriber.hpp"
#include "behaviour/interfaces/timers/ego_pose_timer.hpp"
#include "behaviour/interfaces/timers/tick_tree_timer.hpp"

namespace behaviour
{
BehaviourNode::BehaviourNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("behaviour_node", options)
{
  this->declare_parameter("bt_tree_file", "main_tree.xml");
  this->declare_parameter("rate_hz", 10.0);
  this->declare_parameter("ego_pose_rate_hz", 20.0);
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("base_frame", "base_link");

  RCLCPP_INFO(this->get_logger(), "BehaviourNode created (unconfigured)");
}

BehaviourNode::~BehaviourNode()
{
  on_deactivate(get_current_state());
  on_cleanup(get_current_state());
}

BehaviourNode::CallbackReturn BehaviourNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Configuring...");

  dynamic_object_store_ = std::make_shared<DynamicObjectStore>();

  map_frame_ = this->get_parameter("map_frame").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();

  // Initialize TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create ROS node for BT ROS action/service nodes (workaround for behaviortree_ros2)
  ros_node_ = std::make_shared<rclcpp::Node>("behaviour_ros_node", rclcpp::NodeOptions().use_global_arguments(false));

  // Build tree path
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("behaviour");
  std::string bt_xml_name = this->get_parameter("bt_tree_file").as_string();
  std::filesystem::path tree_path = std::filesystem::path(package_share_directory) / "trees" / bt_xml_name;

  // Create BehaviourTree (registers nodes, creates blackboard, loads XML)
  tree_ = std::make_shared<BehaviourTree>(
    std::dynamic_pointer_cast<rclcpp_lifecycle::LifecycleNode>(this->shared_from_this()),
    ros_node_,
    tree_path.string(),
    dynamic_object_store_);

  // Create interfaces (they receive tree_ to update blackboard)
  createInterfaces();

  RCLCPP_INFO(this->get_logger(), "Configured successfully");
  return CallbackReturn::SUCCESS;
}

void BehaviourNode::createInterfaces()
{
  interfaces_.clear();

  double rate_hz = this->get_parameter("rate_hz").as_double();
  double ego_pose_rate_hz = this->get_parameter("ego_pose_rate_hz").as_double();

  auto ego_pose_period = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / ego_pose_rate_hz));
  auto tick_period = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / rate_hz));

  // subbers
  interfaces_.push_back(std::make_unique<interfaces::LaneContextSubscriber>(this, tree_));
  interfaces_.push_back(std::make_unique<interfaces::GoalPointSubscriber>(this, tree_));
  interfaces_.push_back(std::make_unique<interfaces::DynamicObjectsSubscriber>(this, dynamic_object_store_));

  // TODO(wato): Add OdometrySubscriber here once implemented the publisher
  // interfaces_.push_back(std::make_unique<interfaces::OdometrySubscriber>(this, tree_));

  // timers
  interfaces_.push_back(
    std::make_unique<interfaces::EgoPoseTimer>(this, tf_buffer_, map_frame_, base_frame_, tree_, ego_pose_period));
  interfaces_.push_back(std::make_unique<interfaces::TickTreeTimer>(this, tree_, tick_period));
}

BehaviourNode::CallbackReturn BehaviourNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Activating...");

  for (auto & interface : interfaces_) {
    interface->activate();
  }

  RCLCPP_INFO(this->get_logger(), "Activated successfully");
  return CallbackReturn::SUCCESS;
}

BehaviourNode::CallbackReturn BehaviourNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating...");

  for (auto & interface : interfaces_) {
    interface->deactivate();
  }

  RCLCPP_INFO(this->get_logger(), "Deactivated successfully");
  return CallbackReturn::SUCCESS;
}

BehaviourNode::CallbackReturn BehaviourNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up...");

  interfaces_.clear();
  tree_.reset();
  ros_node_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();

  RCLCPP_INFO(this->get_logger(), "Cleaned up successfully");
  return CallbackReturn::SUCCESS;
}

BehaviourNode::CallbackReturn BehaviourNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  on_deactivate(get_current_state());
  on_cleanup(get_current_state());

  RCLCPP_INFO(this->get_logger(), "Shutting down...");
  return CallbackReturn::SUCCESS;
}

}  // namespace behaviour

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<behaviour::BehaviourNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  if (node->get_ros_node()) {
    executor.add_node(node->get_ros_node());
  }

  executor.spin();
  rclcpp::shutdown();
  return 0;
}

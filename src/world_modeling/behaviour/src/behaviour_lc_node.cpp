
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

#include "behaviour/behaviour_lc_node.hpp"

#include <chrono>

#include <ament_index_cpp/get_package_share_directory.hpp>

// actions
#include "behaviour/nodes/actions/determine_maneuver_action.hpp"
#include "behaviour/nodes/actions/execute_behaviour_action.hpp"

// nodes
#include "behaviour/nodes/determine_reg_elem_node.hpp"

// services
#include "behaviour/nodes/services/get_lanelets_by_reg_elem_service.hpp"
#include "behaviour/nodes/services/get_objects_by_lanelet_service.hpp"
#include "behaviour/nodes/services/get_route_service.hpp"

// conditions
#include "behaviour/nodes/conditions/is_ego_state_condition.hpp"
#include "behaviour/nodes/conditions/is_maneuver_condition.hpp"
#include "behaviour/nodes/conditions/is_route_valid_condition.hpp"
#include "behaviour/nodes/conditions/prediction_condition.hpp"
#include "behaviour/nodes/conditions/proximity_condition.hpp"

// decorators
#include "behaviour/nodes/decorators/rate_controller.hpp"

// TODO(wato): make behaviour_tree_ros2 accept lifecycle node without using a secondhand node(see line 42)
// What was already tried and failed:
// - proxy node / aux node (a rclcpp::Node inside a lifecycle node)
// - static casting ros node over this->shared_from_this() (static_pointer_cast<rclcpp::Node>(this->shared_from_this()))

// Potential solution: change the bt.ros2 library to accept the a lifecycle node as the node handler for ros action wrappers (action clients, service clients)

BehaviourNode::BehaviourNode(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("behaviour_lifecycle_node", options)
{
  this->declare_parameter("bt_tree_file", "main_tree.xml");
  this->declare_parameter("rate_hz", 10.0);

  ros_node_ = std::make_shared<rclcpp::Node>(
      "behaviour_ros_node",
      rclcpp::NodeOptions().use_global_arguments(
          false)); // this is a workaround, the bt library expects a ros node, and not a lifecycle node

  // Register nodes wihtout the need for ros params
  factory_.registerNodeType<behaviour::RateController>("RateController");

  RCLCPP_INFO(this->get_logger(), "BehaviourNode created.");
}

BehaviourNode::~BehaviourNode()
{
  on_deactivate(get_current_state());
  on_cleanup(get_current_state());
}

BehaviourNode::CallbackReturn BehaviourNode::on_configure(const rclcpp_lifecycle::State &)
{
  current_lane_context_sub_ = this->create_subscription<world_modeling_msgs::msg::CurrentLaneContext>(
      "current_lane_context", 10, std::bind(&BehaviourNode::current_lane_context_callback, this, std::placeholders::_1));

  // Register ros bt nodes that need ros param
  BT::RosNodeParams params;
  params.nh = ros_node_; // this is the workaround, the bt library expects a ros node, and not a lifecycle node

  factory_.registerNodeType<behaviour::SampleActionNode>("SampleAction", params);

  // Set up the blackboard for the behavior tree
  blackboard_ = BT::Blackboard::create();
  blackboard_->set<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node", this->shared_from_this());

  std::string package_share_directory = ament_index_cpp::get_package_share_directory("behaviour");
  std::string bt_xml_name = this->get_parameter("bt_tree_file").as_string();
  std::filesystem::path tree_path = std::filesystem::path(package_share_directory) / "trees" / bt_xml_name;

  RCLCPP_INFO(this->get_logger(), "Loading Behavior Tree from: %s", tree_path.c_str());

  tree_ = factory_.createTreeFromFile(tree_path.string(), blackboard_);

  // Running a timer to run this at a stable rate
  // This enables us to run the executor with just a spin at the upper level
  std::chrono::milliseconds rate(int32_t(1000.0 / this->get_parameter("rate_hz").as_double()));
  timer_ = this->create_wall_timer(rate, std::bind(&BehaviourNode::timer_callback, this));

  // start with the timer cancelled
  RCLCPP_INFO(this->get_logger(), "Stopping Timer from running");
  timer_->cancel();

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

BehaviourNode::CallbackReturn BehaviourNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Starting the Timer and running Ticks");
  timer_->reset();
  return CallbackReturn::SUCCESS;
}

BehaviourNode::CallbackReturn BehaviourNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Stopping Timer");
  timer_->cancel();

  // We can wait until Cancel as well by doing timer_->is_cancelled()

  return CallbackReturn::SUCCESS;
}

BehaviourNode::CallbackReturn BehaviourNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning Up");
  blackboard_.reset();
  timer_.reset();
  return CallbackReturn::SUCCESS;
}

BehaviourNode::CallbackReturn BehaviourNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  on_deactivate(get_current_state());
  on_cleanup(get_current_state());

  RCLCPP_INFO(this->get_logger(), "Shutting Down");
  return CallbackReturn::SUCCESS;
}

void BehaviourNode::register_nodes()
{
  BT::RosNodeParams params;
  params.nh = ros_node_;

  // actions
  factory_.registerNodeType<DetermineManeuverAction>("DetermineManeuver");
  factory_.registerNodeType<ExecuteBehaviourAction>("ExecuteBehaviour", params);
  factory_.registerNodeType<DetermineRegElemNode>("DetermineRegElem");

  // services
  factory_.registerNodeType<GetLaneletsByRegElemService>("GetLaneletsByRegElem", params);
  factory_.registerNodeType<GetRouteService>("GetRoute", params);

  // conditions
  factory_.registerNodeType<IsRouteValidCondition>("IsRouteValid");
  factory_.registerNodeType<IsManeuverCondition>("IsManeuver");
  factory_.registerNodeType<IsEgoStateCondition>("IsEgoState");
  factory_.registerNodeType<PredictionCondition>("Prediction");
  factory_.registerNodeType<ProximityCondition>("Proximity");

  // decorators
  factory_.registerNodeType<RateController>(
      "RateController"); // this is a placeholder for now, will be replaced with a real rate controller
}

void BehaviourNode::timer_callback()
{
  RCLCPP_INFO(this->get_logger(), "Ticking the tree");
  tree_.tickOnce();
  return;
}

void BehaviourNode::current_lane_context_callback(const world_modeling_msgs::msg::CurrentLaneContext::SharedPtr msg)
{
  blackboard_->set<world_modeling_msgs::msg::CurrentLaneContext::SharedPtr>("current_lane_context", msg);
  blackboard_->set<int64_t>("current_lanelet_id", msg->current_lanelet.id);
}

// for when bt will be a component
// #include "rclcpp_components/register_node_macro.hpp"

// RCLCPP_COMPONENTS_REGISTER_NODE(BehaviourNode)

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BehaviourNode>();

  // // This is the same as rclcpp::spin(node);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  if (node->get_ros_node())
  {
    executor.add_node(node->get_ros_node()); // node handler for bt ros nodes
  }

  executor.spin();

  rclcpp::shutdown();
  return 0;
}

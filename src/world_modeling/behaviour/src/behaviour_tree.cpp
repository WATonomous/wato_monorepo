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

#include "behaviour/behaviour_tree.hpp"

#include <memory>
#include <string>
#include <utility>

#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/bt_service_node.hpp>

#include "behaviour/dynamic_object_store.hpp"
#include "behaviour/utils/utils.hpp"

// Actions
#include "behaviour/nodes/actions/determine_lane_behaviour.hpp"
#include "behaviour/nodes/actions/determine_reg_elem_node.hpp"
#include "behaviour/nodes/actions/execute_behaviour_action.hpp"
#include "behaviour/nodes/actions/get_objects_by_lanelet_action.hpp"
#include "behaviour/nodes/actions/get_objects_by_lanelets_action.hpp"
#include "behaviour/nodes/actions/get_reg_elem_context_action.hpp"
#include "behaviour/nodes/actions/get_traffic_light_state_action.hpp"
#include "behaviour/nodes/actions/reg_elem_reached_action.hpp"

// Conditions
#include "behaviour/nodes/conditions/comparator_condition.hpp"
#include "behaviour/nodes/conditions/error_message_condition.hpp"
#include "behaviour/nodes/conditions/goal_reached_condition.hpp"
#include "behaviour/nodes/conditions/goal_updated_condition.hpp"
#include "behaviour/nodes/conditions/has_goal_condition.hpp"
#include "behaviour/nodes/conditions/reg_elem_type_condition.hpp"
#include "behaviour/nodes/conditions/safe_prediction_condition.hpp"
#include "behaviour/nodes/conditions/safe_proximity_condition.hpp"
#include "behaviour/nodes/conditions/turn_to_go_condition.hpp"

// Decorators
#include "behaviour/nodes/decorators/rate_controller.hpp"

// Services
#include "behaviour/nodes/services/get_lanelets_by_reg_elem_service.hpp"
#include "behaviour/nodes/services/get_route_service.hpp"
#include "behaviour/nodes/services/set_route_service.hpp"

// #include "behaviour/nodes/services/set_wall_service.hpp"

namespace behaviour
{

BehaviourTree::BehaviourTree(
  rclcpp_lifecycle::LifecycleNode::SharedPtr lifecycle_node,
  rclcpp::Node::SharedPtr ros_node,
  const std::string & tree_file_path,
  std::shared_ptr<DynamicObjectStore> store)
: lifecycle_node_(std::move(lifecycle_node))
, ros_node_(std::move(ros_node))
, store_(std::move(store))
{
  registerNodes();
  buildTree(tree_file_path);
}

void BehaviourTree::registerNodes()
{
  if (!store_) {
    throw std::runtime_error("DynamicObjectStore is null!");
  }

  if (!ros_node_) {
    throw std::runtime_error("Ros node is null!");
  }

  BT::RosNodeParams params;
  params.nh = ros_node_;

  // TODO(wato): Figure out how to register enums with btcpp
  // factory_.registerScriptingEnums<types::LaneBehaviour>();

  // Decorators
  factory_.registerNodeType<RateController>("RateController");

  // Actions
  factory_.registerNodeType<DetermineLaneBehaviourAction>("DetermineLaneBehaviour");
  factory_.registerNodeType<DetermineRegElemAction>("DetermineRegElem");
  factory_.registerNodeType<ExecuteBehaviourAction>("ExecuteBehaviour", params);
  factory_.registerNodeType<GetObjectsByLaneletAction>("GetObjectsByLanelet");
  factory_.registerNodeType<GetObjectsByLaneletsAction>("GetObjectsByLanelets");
  factory_.registerNodeType<GetRegElemContextAction>("GetRegElemContext");
  factory_.registerNodeType<GetTrafficLightStateAction>("GetTrafficLightState");
  factory_.registerNodeType<RegElemReachedAction>("RegElemReached");

  // Services
  factory_.registerNodeType<GetLaneletsByRegElemService>("GetLaneletsByRegElem", params);
  factory_.registerNodeType<GetRouteService>("GetRoute", params);
  factory_.registerNodeType<SetRouteService>("SetRoute", params);
  // factory_.registerNodeType<SetWallService>("SetWall", params);

  // Conditions
  factory_.registerNodeType<ComparatorCondition>("Comparator");
  factory_.registerNodeType<ErrorMessageCondition>("ErrorMessageCondition");
  factory_.registerNodeType<GoalReachedConditon>("GoalReached");
  factory_.registerNodeType<GoalUpdatedCondition>("GoalUpdated");
  factory_.registerNodeType<RegElemTypeCondition>("RegElemType");
  factory_.registerNodeType<SafePredictionCondition>("SafePrediction");
  factory_.registerNodeType<SafeProximityCondition>("SafeProximity");
  factory_.registerNodeType<TurnToGoCondition>("TurnToGo");
  factory_.registerNodeType<HasGoalCondition>("HasGoal");
}

void BehaviourTree::buildTree(const std::string & tree_file_path)
{
  blackboard_ = BT::Blackboard::create();
  blackboard_->set<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node", lifecycle_node_);

  RCLCPP_INFO(lifecycle_node_->get_logger(), "Loading Behavior Tree from: %s", tree_file_path.c_str());
  tree_ = factory_.createTreeFromFile(tree_file_path, blackboard_);
}

void BehaviourTree::tick()
{
  tree_.tickOnce();
}

}  // namespace behaviour

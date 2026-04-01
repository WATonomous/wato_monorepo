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

#ifndef BEHAVIOUR__NODES__COMMON__REGISTRAR_HPP_
#define BEHAVIOUR__NODES__COMMON__REGISTRAR_HPP_

#include <behaviortree_cpp/bt_factory.h>

#include <chrono>
#include <stdexcept>

#include <behaviortree_ros2/ros_node_params.hpp>
#include <rclcpp/rclcpp.hpp>

#include "behaviour/nodes/node_registrar_base.hpp"

// actions
#include "behaviour/nodes/common/actions/clear_pending_goal_point_action.hpp"
#include "behaviour/nodes/common/actions/despawn_wall_service.hpp"
#include "behaviour/nodes/common/actions/execute_behaviour_publisher.hpp"
#include "behaviour/nodes/common/actions/get_area_occupancy_service.hpp"
#include "behaviour/nodes/common/actions/get_area_occupancy_subscriber.hpp"
#include "behaviour/nodes/common/actions/get_forward_lanelet_action.hpp"
#include "behaviour/nodes/common/actions/get_lanelet_by_id_action.hpp"
#include "behaviour/nodes/common/actions/get_lanelet_by_relation_action.hpp"
#include "behaviour/nodes/common/actions/get_shortest_route_service.hpp"
#include "behaviour/nodes/common/actions/get_world_objects_service.hpp"
#include "behaviour/nodes/common/actions/get_world_objects_subscriber.hpp"
#include "behaviour/nodes/common/actions/log_failure_action.hpp"
#include "behaviour/nodes/common/actions/set_new_goal_action.hpp"
#include "behaviour/nodes/common/actions/set_pending_goal_action.hpp"
#include "behaviour/nodes/common/actions/set_route_service.hpp"
#include "behaviour/nodes/common/actions/spawn_wall_service.hpp"
#include "behaviour/nodes/common/actions/speed_behaviour_publisher.hpp"

// conditions
#include "behaviour/nodes/common/conditions/ego_on_lanelet_condition.hpp"
#include "behaviour/nodes/common/conditions/ego_on_route_condition.hpp"
#include "behaviour/nodes/common/conditions/ego_stopped_condition.hpp"
#include "behaviour/nodes/common/conditions/empty_lanelets_condition.hpp"
#include "behaviour/nodes/common/conditions/global_route_exist_condition.hpp"
#include "behaviour/nodes/common/conditions/goal_exist_condition.hpp"
#include "behaviour/nodes/common/conditions/goal_lanelet_exist_condition.hpp"
#include "behaviour/nodes/common/conditions/goal_reached_condition.hpp"
#include "behaviour/nodes/common/conditions/is_area_occupied_condition.hpp"
#include "behaviour/nodes/common/conditions/is_error_message_condition.hpp"
#include "behaviour/nodes/common/conditions/is_lanelet_turn_direction_condition.hpp"
#include "behaviour/nodes/common/conditions/wall_exist_condition.hpp"
#include "behaviour/nodes/common/conditions/world_objects_contains_condition.hpp"

// decorators
#include "behaviour/nodes/common/decorators/rate_controller.hpp"

class CommonNodeRegistrar : public NodeRegistrarBase
{
public:
  explicit CommonNodeRegistrar(const rclcpp::Node::SharedPtr & node)
  : NodeRegistrarBase(node)
  {}

  void register_nodes(BT::BehaviorTreeFactory & factory, const BT::RosNodeParams & params) override
  {
    int service_timeout = node_->get_parameter("service_timeout_ms").as_int();
    int wait_for_service_timeout = node_->get_parameter("wait_for_service_timeout_ms").as_int();

    BT::RosNodeParams service_params = params;
    service_params.server_timeout = std::chrono::milliseconds(service_timeout);
    service_params.wait_for_server_timeout = std::chrono::milliseconds(wait_for_service_timeout);

    // if ever we need to set different timeouts for different services
    BT::RosNodeParams get_shortest_route_params = service_params;
    BT::RosNodeParams set_route_params = service_params;
    BT::RosNodeParams get_area_occupancy_params = service_params;
    BT::RosNodeParams get_objects_params = service_params;
    BT::RosNodeParams spawn_wall_params = service_params;
    BT::RosNodeParams despawn_wall_params = service_params;

    // actions
    factory.registerNodeType<behaviour::GetShortestRouteService>("GetShortestRoute", get_shortest_route_params);
    factory.registerNodeType<behaviour::SetRouteService>("SetRoute", set_route_params);
    factory.registerNodeType<behaviour::GetAreaOccupancyService>("GetAreaOccupancy", get_area_occupancy_params);
    factory.registerNodeType<behaviour::GetWorldObjectsService>("GetWorldObjects", get_objects_params);
    factory.registerNodeType<behaviour::SpawnWallService>("SpawnWall", spawn_wall_params);
    factory.registerNodeType<behaviour::DespawnWallService>("DespawnWall", despawn_wall_params);
    factory.registerNodeType<behaviour::GetLaneletByIdAction>("GetLaneletById", logger_.get_child("GetLaneletById"));
    factory.registerNodeType<behaviour::GetForwardLaneletAction>(
      "GetForwardLanelet", logger_.get_child("GetForwardLanelet"));
    factory.registerNodeType<behaviour::GetLaneletByRelationAction>(
      "GetLaneletByRelation", logger_.get_child("GetLaneletByRelation"));
    factory.registerNodeType<behaviour::ClearPendingGoalPointAction>(
      "ClearPendingGoalPoint", logger_.get_child("ClearPendingGoalPoint"));
    factory.registerNodeType<behaviour::LogFailureAction>("LogFailure", logger_.get_child("LogFailure"));
    factory.registerNodeType<behaviour::SetNewGoalAction>("SetNewGoal", logger_.get_child("SetNewGoal"));
    factory.registerNodeType<behaviour::SetPendingGoalAction>("SetPendingGoal", logger_.get_child("SetPendingGoal"));

    factory.registerNodeType<behaviour::GetWorldObjectsSubscriber>("GetWorldObjectsSub", service_params);
    factory.registerNodeType<behaviour::GetAreaOccupancySubscriber>("GetAreaOccupancySub", service_params);

    factory.registerNodeType<behaviour::ExecuteBehaviourPublisher>("ExecuteBehaviour", service_params);
    factory.registerNodeType<behaviour::SpeedBehaviourPublisher>("SpeedBehaviour", service_params);

    // conditions
    factory.registerNodeType<behaviour::IsErrorMessageCondition>("IsErrorMessage", logger_.get_child("IsErrorMessage"));
    factory.registerNodeType<behaviour::WallIdExistCondition>("WallIdExist", logger_.get_child("WallIdExist"));
    factory.registerNodeType<behaviour::GoalReachedCondition>("GoalReached", logger_.get_child("GoalReached"));
    factory.registerNodeType<behaviour::GoalExistCondition>("GoalExist", logger_.get_child("GoalExist"));
    factory.registerNodeType<behaviour::GoalLaneletExistCondition>(
      "GoalLaneletExist", logger_.get_child("GoalLaneletExist"));
    factory.registerNodeType<behaviour::EmptyLaneletsCondition>("EmptyLanelets", logger_.get_child("EmptyLanelets"));
    factory.registerNodeType<behaviour::GlobalRouteExistCondition>(
      "GlobalRouteExist", logger_.get_child("GlobalRouteExist"));
    factory.registerNodeType<behaviour::EgoOnRouteCondition>("EgoOnRoute", logger_.get_child("EgoOnRoute"));
    factory.registerNodeType<behaviour::EgoOnLaneletCondition>("IsEgoOnLanelet", logger_.get_child("IsEgoOnLanelet"));
    factory.registerNodeType<behaviour::EgoStoppedCondition>("EgoStopped", logger_.get_child("EgoStopped"));
    factory.registerNodeType<behaviour::IsAreaOccupiedCondition>("IsAreaOccupied", logger_.get_child("IsAreaOccupied"));
    factory.registerNodeType<behaviour::WorldObjectsContainsCondition>(
      "WorldObjectsContains", logger_.get_child("WorldObjectsContains"));
    factory.registerNodeType<behaviour::IsLaneletTurnDirectionCondition>(
      "IsLaneletTurnDirection", logger_.get_child("IsLaneletTurnDirection"));

    // decorators
    factory.registerNodeType<behaviour::RateController>("RateController");
  }
};

#endif  // BEHAVIOUR__NODES__COMMON__REGISTRAR_HPP_

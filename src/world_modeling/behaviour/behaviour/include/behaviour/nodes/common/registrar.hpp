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

#include <stdexcept>

#include <behaviortree_ros2/ros_node_params.hpp>

#include "behaviour/nodes/node_registrar_base.hpp"

// actions
#include "behaviour/nodes/common/actions/despawn_wall_service.hpp"
#include "behaviour/nodes/common/actions/execute_behaviour_publisher.hpp"
#include "behaviour/nodes/common/actions/get_area_occupancy_service.hpp"
#include "behaviour/nodes/common/actions/get_area_occupancy_subscriber.hpp"
#include "behaviour/nodes/common/actions/get_lanelet_by_id_action.hpp"
#include "behaviour/nodes/common/actions/get_lanelet_by_relation_action.hpp"
#include "behaviour/nodes/common/actions/get_shortest_route_service.hpp"
#include "behaviour/nodes/common/actions/get_world_objects_service.hpp"
#include "behaviour/nodes/common/actions/get_world_objects_subscriber.hpp"
#include "behaviour/nodes/common/actions/set_route_service.hpp"
#include "behaviour/nodes/common/actions/spawn_wall_service.hpp"

// conditions
#include "behaviour/nodes/common/conditions/ego_on_lanelet_condition.hpp"
#include "behaviour/nodes/common/conditions/ego_on_route_condition.hpp"
#include "behaviour/nodes/common/conditions/ego_stopped_condition.hpp"
#include "behaviour/nodes/common/conditions/global_route_exist_condition.hpp"
#include "behaviour/nodes/common/conditions/goal_exist_condition.hpp"
#include "behaviour/nodes/common/conditions/goal_lanelet_exist_condition.hpp"
#include "behaviour/nodes/common/conditions/goal_reached_condition.hpp"
#include "behaviour/nodes/common/conditions/is_area_occupied_condition.hpp"
#include "behaviour/nodes/common/conditions/is_error_message_condition.hpp"
#include "behaviour/nodes/common/conditions/wall_exist_condition.hpp"
#include "behaviour/nodes/common/conditions/world_objects_contains_condition.hpp"

// decorators
#include "behaviour/nodes/common/decorators/rate_controller.hpp"

class CommonNodeRegistrar : public NodeRegistrarBase
{
public:
  void register_nodes(BT::BehaviorTreeFactory & factory, const BT::RosNodeParams & params) override
  {
    BT::RosNodeParams get_shortest_route_params = params;
    BT::RosNodeParams set_route_params = params;
    BT::RosNodeParams get_area_occupancy_params = params;
    BT::RosNodeParams get_objects_params = params;
    BT::RosNodeParams wall_service = params;

    // Read timeout values from node parameters
    auto node = params.nh.lock();
    if (!node) {
      throw std::runtime_error("ROS node expired in CommonNodeRegistrar");
    }
    auto logger = node->get_logger();
    int get_shortest_route_timeout = node->get_parameter("get_shortest_route_timeout_ms").as_int();
    int set_route_timeout = node->get_parameter("set_route_timeout_ms").as_int();
    int get_area_occupancy_timeout = node->get_parameter("get_area_occupancy_timeout_ms").as_int();
    int get_world_objects_enriched_timeout = node->get_parameter("get_world_objects_enriched_timeout_ms").as_int();
    int wall_timeout = node->get_parameter("wall_service_timeout_ms").as_int();

    get_shortest_route_params.server_timeout = std::chrono::milliseconds(get_shortest_route_timeout);
    set_route_params.server_timeout = std::chrono::milliseconds(set_route_timeout);
    get_area_occupancy_params.server_timeout = std::chrono::milliseconds(get_area_occupancy_timeout);
    get_objects_params.server_timeout = std::chrono::milliseconds(get_world_objects_enriched_timeout);
    wall_service.server_timeout = std::chrono::milliseconds(wall_timeout);

    // actions
    factory.registerNodeType<behaviour::GetShortestRouteService>("GetShortestRoute", get_shortest_route_params);
    factory.registerNodeType<behaviour::SetRouteService>("SetRoute", set_route_params);
    factory.registerNodeType<behaviour::GetAreaOccupancyService>("GetAreaOccupancy", get_area_occupancy_params);
    factory.registerNodeType<behaviour::GetWorldObjectsService>("GetWorldObjects", get_objects_params);
    factory.registerNodeType<behaviour::SpawnWallService>("SpawnWall", wall_service);
    factory.registerNodeType<behaviour::DespawnWallService>("DespawnWall", wall_service);
    factory.registerNodeType<behaviour::GetLaneletByIdAction>("GetLaneletById", logger.get_child("GetLaneletById"));
    factory.registerNodeType<behaviour::GetLaneletByRelationAction>(
      "GetLaneletByRelation", logger.get_child("GetLaneletByRelation"));

    factory.registerNodeType<behaviour::GetWorldObjectsSubscriber>("GetWorldObjectsSub", params);
    factory.registerNodeType<behaviour::GetAreaOccupancySubscriber>("GetAreaOccupancySub", params);

    factory.registerNodeType<behaviour::ExecuteBehaviourPublisher>("ExecuteBehaviour", params);

    // conditions
    factory.registerNodeType<behaviour::IsErrorMessageCondition>("IsErrorMessage", logger.get_child("IsErrorMessage"));
    factory.registerNodeType<behaviour::WallIdExistCondition>("WallIdExist", logger.get_child("WallIdExist"));
    factory.registerNodeType<behaviour::GoalReachedCondition>("GoalReached", logger.get_child("GoalReached"));
    factory.registerNodeType<behaviour::GoalExistCondition>("GoalExist", logger.get_child("GoalExist"));
    factory.registerNodeType<behaviour::GoalLaneletExistCondition>(
      "GoalLaneletExist", logger.get_child("GoalLaneletExist"));
    factory.registerNodeType<behaviour::GlobalRouteExistCondition>(
      "GlobalRouteExist", logger.get_child("GlobalRouteExist"));
    factory.registerNodeType<behaviour::EgoOnRouteCondition>("EgoOnRoute", logger.get_child("EgoOnRoute"));
    factory.registerNodeType<behaviour::EgoOnLaneletCondition>("IsEgoOnLanelet", logger.get_child("IsEgoOnLanelet"));
    factory.registerNodeType<behaviour::EgoStoppedCondition>("EgoStopped", logger.get_child("EgoStopped"));
    factory.registerNodeType<behaviour::IsAreaOccupiedCondition>("IsAreaOccupied", logger.get_child("IsAreaOccupied"));
    factory.registerNodeType<behaviour::WorldObjectsContainsCondition>(
      "WorldObjectsContains", logger.get_child("WorldObjectsContains"));

    // decorators
    factory.registerNodeType<behaviour::RateController>("RateController");
  }
};

#endif  // BEHAVIOUR__NODES__COMMON__REGISTRAR_HPP_

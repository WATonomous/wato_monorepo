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

#include <behaviortree_ros2/ros_node_params.hpp>

#include "behaviour/nodes/node_registrar_base.hpp"

// actions
#include "behaviour/nodes/common/actions/despawn_wall_service.hpp"
#include "behaviour/nodes/common/actions/execute_behaviour_publisher.hpp"
#include "behaviour/nodes/common/actions/get_lanelets_by_reg_elem_service.hpp"
#include "behaviour/nodes/common/actions/get_shortest_route_service.hpp"
#include "behaviour/nodes/common/actions/set_route_service.hpp"
#include "behaviour/nodes/common/actions/spawn_wall_service.hpp"

// conditions
#include "behaviour/nodes/common/conditions/ego_on_route_condition.hpp"
#include "behaviour/nodes/common/conditions/ego_stopped_condition.hpp"
#include "behaviour/nodes/common/conditions/global_route_exist_condition.hpp"
#include "behaviour/nodes/common/conditions/goal_exist_condition.hpp"
#include "behaviour/nodes/common/conditions/goal_reached_condition.hpp"
#include "behaviour/nodes/common/conditions/is_error_message_condition.hpp"
#include "behaviour/nodes/common/conditions/wall_exist_condition.hpp"

// decorators
#include "behaviour/nodes/common/decorators/rate_controller.hpp"

class CommonNodeRegistrar : public NodeRegistrarBase
{
public:
  void register_nodes(BT::BehaviorTreeFactory &factory, const BT::RosNodeParams &params) override
  {
    BT::RosNodeParams get_shortest_route_params = params;
    BT::RosNodeParams set_route_params = params;
    BT::RosNodeParams get_lanelets_by_reg_elem_params = params;
    BT::RosNodeParams wall_service = params;

    // Read timeout values from node parameters
    auto node = params.nh.lock();
    if (!node)
    {
      throw std::runtime_error("ROS node expired in CommonNodeRegistrar");
    }
    int get_shortest_route_timeout = node->get_parameter("get_shortest_route_timeout_ms").as_int();
    int set_route_timeout = node->get_parameter("set_route_timeout_ms").as_int();
    int get_lanelets_timeout = node->get_parameter("get_lanelets_by_reg_elem_timeout_ms").as_int();
    int wall_timeout = node->get_parameter("wall_service_timeout_ms").as_int();

    get_shortest_route_params.server_timeout = std::chrono::milliseconds(get_shortest_route_timeout);
    set_route_params.server_timeout = std::chrono::milliseconds(set_route_timeout);
    get_lanelets_by_reg_elem_params.server_timeout = std::chrono::milliseconds(get_lanelets_timeout);
    wall_service.server_timeout = std::chrono::milliseconds(wall_timeout);

    // actions
    factory.registerNodeType<behaviour::GetShortestRouteService>("GetShortestRoute", get_shortest_route_params);
    factory.registerNodeType<behaviour::SetRouteService>("SetRoute", set_route_params);
    factory.registerNodeType<behaviour::GetLaneletsByRegElemService>(
        "GetLaneletsByRegElem", get_lanelets_by_reg_elem_params);
    factory.registerNodeType<behaviour::SpawnWallService>("SpawnWall", wall_service);
    factory.registerNodeType<behaviour::DespawnWallService>("DespawnWall", wall_service);
    factory.registerNodeType<behaviour::ExecuteBehaviourPublisher>("ExecuteBehaviour", params);

    // conditions
    factory.registerNodeType<behaviour::IsErrorMessageCondition>("IsErrorMessage");
    factory.registerNodeType<behaviour::WallIdExistCondition>("WallIdExist");
    factory.registerNodeType<behaviour::GoalReachedCondition>("GoalReached");
    factory.registerNodeType<behaviour::GoalExistCondition>("GoalExist");
    factory.registerNodeType<behaviour::GlobalRouteExistCondition>("GlobalRouteExist");
    factory.registerNodeType<behaviour::EgoOnRouteCondition>("EgoOnRoute");
    factory.registerNodeType<behaviour::EgoStoppedCondition>("EgoStopped");

    // decorators
    factory.registerNodeType<behaviour::RateController>("RateController");
  }
};

#endif // BEHAVIOUR__NODES__COMMON__REGISTRAR_HPP_

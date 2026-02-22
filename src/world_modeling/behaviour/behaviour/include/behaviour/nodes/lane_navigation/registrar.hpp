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

#ifndef BEHAVIOUR__NODES__LANE_NAVIGATION__REGISTRAR_HPP_
#define BEHAVIOUR__NODES__LANE_NAVIGATION__REGISTRAR_HPP_

#include <behaviortree_cpp/bt_factory.h>

#include <behaviortree_ros2/ros_node_params.hpp>

#include "behaviour/nodes/node_registrar_base.hpp"
#include "behaviour/utils/types.hpp"

// actions
#include "behaviour/nodes/lane_navigation/actions/get_follow_lane_preferred_lanelets_action.hpp"
#include "behaviour/nodes/lane_navigation/actions/get_lane_change_preferred_lanelets_action.hpp"
#include "behaviour/nodes/lane_navigation/actions/get_route_context_action.hpp"

// conditions
#include "behaviour/nodes/lane_navigation/conditions/is_lane_transition_condition.hpp"
#include "behaviour/nodes/lane_navigation/conditions/safe_lane_change_condition.hpp"
#include "behaviour/nodes/lane_navigation/conditions/valid_lane_change_condition.hpp"

class LaneNavigationNodeRegistrar : public NodeRegistrarBase
{
public:
  void register_nodes(BT::BehaviorTreeFactory & factory, const BT::RosNodeParams & params) override
  {
    // enums
    factory.registerScriptingEnums<behaviour::types::LaneTransition>();

    // actions
    factory.registerNodeType<behaviour::GetRouteContextAction>("GetRouteContext");
    factory.registerNodeType<behaviour::GetFollowLanePreferredLaneletsAction>("GetFollowLanePreferredLanelets");
    factory.registerNodeType<behaviour::GetLaneChangePreferredLaneletsAction>("GetLaneChangePreferredLanelets");

    // conditions
    factory.registerNodeType<behaviour::IsLaneTransitionCondition>("IsLaneTransition");
    factory.registerNodeType<behaviour::ValidLaneChangeCondition>("ValidLaneChange");
    factory.registerNodeType<behaviour::SafeLaneChangeCondition>("SafeLaneChange");
  }
};

#endif  // BEHAVIOUR__NODES__LANE_NAVIGATION__REGISTRAR_HPP_

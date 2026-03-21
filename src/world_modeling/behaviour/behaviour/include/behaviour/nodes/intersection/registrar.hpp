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

#ifndef BEHAVIOUR__NODES__INTERSECTION__REGISTRAR_HPP_
#define BEHAVIOUR__NODES__INTERSECTION__REGISTRAR_HPP_

#include <behaviortree_cpp/bt_factory.h>

#include <stdexcept>

#include <behaviortree_ros2/ros_node_params.hpp>

#include "behaviour/nodes/node_registrar_base.hpp"
#include "behaviour/utils/types.hpp"

// actions
#include "behaviour/nodes/intersection/actions/clear_active_traffic_control_element_action.hpp"
#include "behaviour/nodes/intersection/actions/get_intersection_context_action.hpp"
#include "behaviour/nodes/intersection/actions/get_intersection_search_lanelets_action.hpp"
#include "behaviour/nodes/intersection/actions/get_lanelet_end_pose_action.hpp"
#include "behaviour/nodes/intersection/actions/get_stop_line_pose_action.hpp"
#include "behaviour/nodes/intersection/actions/get_stop_sign_cars_action.hpp"
#include "behaviour/nodes/intersection/actions/get_stop_sign_pedestrians_action.hpp"
#include "behaviour/nodes/intersection/actions/get_traffic_light_state_action.hpp"
#include "behaviour/nodes/intersection/actions/reset_intersection_context_action.hpp"
#include "behaviour/nodes/intersection/actions/set_reg_elem_ego_priority_action.hpp"
#include "behaviour/nodes/intersection/actions/set_stop_sign_priority_cars_action.hpp"

// conditions
#include "behaviour/nodes/intersection/conditions/active_traffic_control_element_exist_condition.hpp"
#include "behaviour/nodes/intersection/conditions/active_traffic_control_element_passed_condition.hpp"
#include "behaviour/nodes/intersection/conditions/is_regulatory_element_type_condition.hpp"
#include "behaviour/nodes/intersection/conditions/is_traffic_light_state_condition.hpp"
#include "behaviour/nodes/intersection/conditions/reg_elem_ego_priority_condition.hpp"
#include "behaviour/nodes/intersection/conditions/stop_sign_pedestrians_clear_condition.hpp"
#include "behaviour/nodes/intersection/conditions/stop_sign_priority_cars_clear_condition.hpp"
#include "behaviour/nodes/intersection/conditions/yield_sign_can_proceed_condition.hpp"

class IntersectionNodeRegistrar : public NodeRegistrarBase
{
public:
  void register_nodes(BT::BehaviorTreeFactory & factory, const BT::RosNodeParams & params) override
  {
    auto node = params.nh.lock();
    if (!node) {
      throw std::runtime_error("ROS node expired in IntersectionNodeRegistrar");
    }
    auto logger = node->get_logger();

    // enums
    factory.registerScriptingEnums<behaviour::types::TrafficControlElementType>();

    // actions
    factory.registerNodeType<behaviour::ClearActiveTrafficControlElementAction>(
      "ClearActiveTrafficControlElement", logger.get_child("ClearActiveTrafficControlElement"));
    factory.registerNodeType<behaviour::GetIntersectionContextAction>(
      "GetIntersectionContext", logger.get_child("GetIntersectionContext"));
    factory.registerNodeType<behaviour::GetIntersectionSearchLaneletsAction>(
      "GetIntersectionSearchLanelets", logger.get_child("GetIntersectionSearchLanelets"));
    factory.registerNodeType<behaviour::GetLaneletEndPoseAction>(
      "GetLaneletEndPose", logger.get_child("GetLaneletEndPose"));
    factory.registerNodeType<behaviour::GetStopSignCarsAction>("GetStopSignCars", logger.get_child("GetStopSignCars"));
    factory.registerNodeType<behaviour::GetStopSignPedestriansAction>(
      "GetStopSignPedestrians", logger.get_child("GetStopSignPedestrians"));
    factory.registerNodeType<behaviour::GetTrafficLightStateAction>(
      "GetTrafficLightState", logger.get_child("GetTrafficLightState"));
    factory.registerNodeType<behaviour::GetStopLinePoseAction>("GetStopLinePose", logger.get_child("GetStopLinePose"));
    factory.registerNodeType<behaviour::ResetIntersectionContextAction>(
      "ResetIntersectionContext", logger.get_child("ResetIntersectionContext"));
    factory.registerNodeType<behaviour::SetRegElemEgoPriorityAction>(
      "SetRegElemEgoPriority", logger.get_child("SetRegElemEgoPriority"));
    factory.registerNodeType<behaviour::SetStopSignPriorityCarsAction>(
      "SetStopSignPriorityCars", logger.get_child("SetStopSignPriorityCars"));

    // conditions
    factory.registerNodeType<behaviour::ActiveTrafficControlElementExistCondition>(
      "ActiveTrafficControlElementExist", logger.get_child("ActiveTrafficControlElementExist"));
    factory.registerNodeType<behaviour::ActiveTrafficControlElementPassedCondition>(
      "ActiveTrafficControlElementPassed", logger.get_child("ActiveTrafficControlElementPassed"));
    factory.registerNodeType<behaviour::IsRegulatoryElementTypeCondition>(
      "IsRegulatoryElementType", logger.get_child("IsRegulatoryElementType"));
    factory.registerNodeType<behaviour::IsTrafficLightStateCondition>(
      "IsTrafficLightState", logger.get_child("IsTrafficLightState"));
    factory.registerNodeType<behaviour::StopSignPriorityCarsClearCondition>(
      "StopSignPriorityCarsClear", logger.get_child("StopSignPriorityCarsClear"));
    factory.registerNodeType<behaviour::StopSignPedestriansClearCondition>(
      "StopSignPedestriansClear", logger.get_child("StopSignPedestriansClear"));
    factory.registerNodeType<behaviour::RegElemEgoPriorityCondition>(
      "RegElemEgoPriority", logger.get_child("RegElemEgoPriority"));
    factory.registerNodeType<behaviour::YieldSignCanProceedCondition>(
      "YieldCanProceed", logger.get_child("YieldCanProceed"));
  }
};

#endif  // BEHAVIOUR__NODES__INTERSECTION__REGISTRAR_HPP_

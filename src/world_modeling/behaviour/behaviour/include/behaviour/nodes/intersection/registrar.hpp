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

#include <behaviortree_ros2/ros_node_params.hpp>

#include <stdexcept>

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
#include "behaviour/nodes/intersection/actions/get_yielding_right_of_way_lanelet_id_action.hpp"
#include "behaviour/nodes/intersection/actions/reset_intersection_context_action.hpp"
#include "behaviour/nodes/intersection/actions/set_reg_elem_ego_priority_action.hpp"
#include "behaviour/nodes/intersection/actions/set_stop_sign_priority_cars_action.hpp"

// conditions
#include "behaviour/nodes/intersection/conditions/active_traffic_control_element_exist_condition.hpp"
#include "behaviour/nodes/intersection/conditions/active_traffic_control_element_passed_condition.hpp"
#include "behaviour/nodes/intersection/conditions/is_regulatory_element_type_condition.hpp"
#include "behaviour/nodes/intersection/conditions/is_traffic_light_state_condition.hpp"
#include "behaviour/nodes/intersection/conditions/passing_active_traffic_control_element_condition.hpp"
#include "behaviour/nodes/intersection/conditions/reg_elem_ego_priority_condition.hpp"
#include "behaviour/nodes/intersection/conditions/stop_sign_priority_cars_clear_condition.hpp"
#include "behaviour/nodes/intersection/conditions/stop_sign_pedestrians_clear_condition.hpp"
#include "behaviour/nodes/intersection/conditions/yield_sign_can_proceed_condition.hpp"

class IntersectionNodeRegistrar : public NodeRegistrarBase
{
public:
  explicit IntersectionNodeRegistrar(const rclcpp::Node::SharedPtr & node)
  : NodeRegistrarBase(node) {}

  void register_nodes(BT::BehaviorTreeFactory & factory, const BT::RosNodeParams & params) override
  {
    (void)params;

    // enums
    factory.registerScriptingEnums<behaviour::types::TrafficControlElementType>();

    // actions
    factory.registerNodeType<behaviour::ClearActiveTrafficControlElementAction>(
      "ClearActiveTrafficControlElement", logger_.get_child("ClearActiveTrafficControlElement"));
    factory.registerNodeType<behaviour::GetIntersectionContextAction>(
      "GetIntersectionContext", logger_.get_child("GetIntersectionContext"));
    factory.registerNodeType<behaviour::GetIntersectionSearchLaneletsAction>(
      "GetIntersectionSearchLanelets", logger_.get_child("GetIntersectionSearchLanelets"));
    factory.registerNodeType<behaviour::GetLaneletEndPoseAction>(
      "GetLaneletEndPose", logger_.get_child("GetLaneletEndPose"));
    factory.registerNodeType<behaviour::GetStopSignCarsAction>("GetStopSignCars", logger_.get_child("GetStopSignCars"));
    factory.registerNodeType<behaviour::GetStopSignPedestriansAction>(
      "GetStopSignPedestrians", logger_.get_child("GetStopSignPedestrians"));
    factory.registerNodeType<behaviour::GetTrafficLightStateAction>(
      "GetTrafficLightState", logger_.get_child("GetTrafficLightState"));
    factory.registerNodeType<behaviour::GetYieldingRightOfWayLaneletIdAction>(
      "GetYieldingRightOfWayLaneletId", logger_.get_child("GetYieldingRightOfWayLaneletId"));
    factory.registerNodeType<behaviour::GetStopLinePoseAction>("GetStopLinePose", logger_.get_child("GetStopLinePose"));
    factory.registerNodeType<behaviour::ResetIntersectionContextAction>(
      "ResetIntersectionContext", logger_.get_child("ResetIntersectionContext"));
    factory.registerNodeType<behaviour::SetRegElemEgoPriorityAction>(
      "SetRegElemEgoPriority", logger_.get_child("SetRegElemEgoPriority"));
    factory.registerNodeType<behaviour::SetStopSignPriorityCarsAction>(
      "SetStopSignPriorityCars", logger_.get_child("SetStopSignPriorityCars"));

    // conditions
    factory.registerNodeType<behaviour::ActiveTrafficControlElementExistCondition>(
      "ActiveTrafficControlElementExist", logger_.get_child("ActiveTrafficControlElementExist"));
    factory.registerNodeType<behaviour::ActiveTrafficControlElementPassedCondition>(
      "ActiveTrafficControlElementPassed", logger_.get_child("ActiveTrafficControlElementPassed"));
    factory.registerNodeType<behaviour::IsRegulatoryElementTypeCondition>(
      "IsRegulatoryElementType", logger_.get_child("IsRegulatoryElementType"));
    factory.registerNodeType<behaviour::IsTrafficLightStateCondition>(
      "IsTrafficLightState", logger_.get_child("IsTrafficLightState"));
    factory.registerNodeType<behaviour::PassingActiveTrafficControlElementCondition>(
      "PassingActiveTrafficControlElement", logger_.get_child("PassingActiveTrafficControlElement"));
    factory.registerNodeType<behaviour::StopSignPriorityCarsClearCondition>(
      "StopSignPriorityCarsClear", logger_.get_child("StopSignPriorityCarsClear"));
    factory.registerNodeType<behaviour::StopSignPedestriansClearCondition>(
      "StopSignPedestriansClear", logger_.get_child("StopSignPedestriansClear"));
    factory.registerNodeType<behaviour::RegElemEgoPriorityCondition>(
      "RegElemEgoPriority", logger_.get_child("RegElemEgoPriority"));
    factory.registerNodeType<behaviour::YieldSignCanProceedCondition>(
      "YieldCanProceed", logger_.get_child("YieldCanProceed"));
  }
};

#endif  // BEHAVIOUR__NODES__INTERSECTION__REGISTRAR_HPP_

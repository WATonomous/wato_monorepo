#ifndef BEHAVIOUR__NODES__LANE_NAVIGATION_NODE__REGISTRAR_HPP_
#define BEHAVIOUR__NODES__LANE_NAVIGATION_NODE__REGISTRAR_HPP_

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_ros2/ros_node_params.hpp>

#include "behaviour/nodes/node_registrar_base.hpp"
#include "behaviour/utils/types.hpp"

// actions
#include "behaviour/nodes/lane_navigation/actions/get_route_context_action.hpp"
#include "behaviour/nodes/lane_navigation/actions/get_follow_lane_preferred_lanelets_action.hpp"
#include "behaviour/nodes/lane_navigation/actions/get_lane_change_preferred_lanelets_action.hpp"

// conditions
#include "behaviour/nodes/lane_navigation/conditions/is_lane_transition_condition.hpp"
#include "behaviour/nodes/lane_navigation/conditions/valid_lane_change_condition.hpp"
#include "behaviour/nodes/lane_navigation/conditions/safe_lane_change_condition.hpp"

class LaneNavigationNodeRegistrar : public NodeRegistrarBase
{
public:
  void register_nodes(BT::BehaviorTreeFactory &factory, const BT::RosNodeParams &params) override
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

#endif // BEHAVIOUR__NODES__LANE_NAVIGATION_NODE__REGISTRAR_HPP_
#ifndef BEHAVIOUR__NODES__INTERSECTION_NODE__REGISTRAR_HPP_
#define BEHAVIOUR__NODES__INTERSECTION_NODE__REGISTRAR_HPP_

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_ros2/ros_node_params.hpp>

#include "behaviour/nodes/node_registrar_base.hpp"
#include "behaviour/utils/types.hpp"

// actions
#include "behaviour/nodes/intersection/actions/clear_active_traffic_control_element_action.hpp"
#include "behaviour/nodes/intersection/actions/clear_stop_sign_priority_cars_action.hpp"
#include "behaviour/nodes/intersection/actions/get_intersection_context_action.hpp"
#include "behaviour/nodes/intersection/actions/get_stop_sign_cars_action.hpp"
#include "behaviour/nodes/intersection/actions/set_stop_sign_priority_cars_action.hpp"
// #include "behaviour/nodes/intersection/actions/set_wall_service.hpp"

// conditions
#include "behaviour/nodes/intersection/conditions/active_traffic_control_element_exist_condition.hpp"
#include "behaviour/nodes/intersection/conditions/is_regulatory_element_type_condition.hpp"
#include "behaviour/nodes/intersection/conditions/stop_sign_can_proceed_condition.hpp"
#include "behaviour/nodes/intersection/conditions/yield_sign_can_proceed_condition.hpp"

class IntersectionNodeRegistrar : public NodeRegistrarBase
{
public:
    void register_nodes(BT::BehaviorTreeFactory &factory, const BT::RosNodeParams &params) override
    {
        // BT::RosNodeParams set_wall_params = params;
        // set_wall_params.server_timeout = std::chrono::milliseconds(5000);

        // enums
        factory.registerScriptingEnums<behaviour::types::TrafficControlElementType>();

        // actions
        factory.registerNodeType<behaviour::ClearActiveTrafficControlElementAction>("ClearActiveTrafficControlElement");
        factory.registerNodeType<behaviour::ClearStopSignPriorityCarsAction>("ClearStopSignPriorityCarsAction");
        factory.registerNodeType<behaviour::GetIntersectionContextAction>("GetIntersectionContext");
        factory.registerNodeType<behaviour::GetStopSignCarsAction>("GetStopSignCars");
        factory.registerNodeType<behaviour::SetStopSignPriorityCarsAction>("SetStopSignPriorityCars");
        // factory.registerNodeType<behaviour::SetWallService>("SetWallService", set_wall_params);

        // conditions
        factory.registerNodeType<behaviour::ActiveTrafficControlElementExistCondition>("ActiveTrafficControlElementExist");
        factory.registerNodeType<behaviour::IsRegulatoryElementTypeCondition>("IsRegulatoryElementType");
        factory.registerNodeType<behaviour::StopSignCanProceedCondition>("StopSignCanProceed");
        factory.registerNodeType<behaviour::YieldSignCanProceedCondition>("YieldCanProceed");
    }
};

#endif // BEHAVIOUR__NODES__INTERSECTION_NODE__REGISTRAR_HPP_

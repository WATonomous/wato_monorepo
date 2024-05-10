#ifndef WORLD_MODELING_STOP_SIGN_REG_ELEM_
#define WORLD_MODELING_STOP_SIGN_REG_ELEM_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Exceptions.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include "hd_map_router.hpp"

class StopSignRegElem : public lanelet::RegulatoryElement{
    public:
    StopSignRegElem();
    static constexpr char RuleName[] = "stop_sign";

    private:
    // The following lines are required so that the lanelet library can create the StopSignRegElem object
    // Refer to : https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_examples/src/02_regulatory_elements/main.cpp
    friend class lanelet::RegisterRegulatoryElement<StopSignRegElem>;
    explicit StopSignRegElem(const lanelet::RegulatoryElementDataPtr& data);
};

#endif
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
    void retrieve_stop_sign_msg();
    lanelet::ConstLanelet find_nearest_lanelet();
    
    private:

};

#endif
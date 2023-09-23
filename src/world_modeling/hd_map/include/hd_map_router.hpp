#ifndef WORLD_MODELING_HD_MAP_ROUTER_
#define WORLD_MODELING_HD_MAP_ROUTER_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Exceptions.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

lanelet::GPSPoint ros_gps_msg_to_lanelet_gps_point(sensor_msgs::msg::NavSatFix::SharedPtr gps_msg);

class HDMapRouter {
  public:
    HDMapRouter();

    bool set_lanelet(const lanelet::LaneletMapPtr &lanelet_ptr);

    lanelet::LaneletMapPtr get_lanelet();
    
    // TODO: implemtation of function to get nearest lanelet in lanelet map to gps point
    lanelet::ConstLanelet get_nearest_lanelet_to_gps(lanelet::GPSPoint gps_point);

    lanelet::Optional<lanelet::routing::LaneletPath> route(lanelet::GPSPoint from_point, lanelet::GPSPoint to_point);
    
    // TODO: implementation of route function to find shortest path from from_lanelet to to_lanelet
    // Reference: https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_examples/src/06_routing/main.cpp
    lanelet::Optional<lanelet::routing::LaneletPath> route(lanelet::ConstLanelet from_lanelet, lanelet::ConstLanelet to_lanelet);


  private:
    lanelet::LaneletMapPtr lanelet_ptr_;
    lanelet::routing::RoutingGraphPtr routing_graph_;
};

#endif

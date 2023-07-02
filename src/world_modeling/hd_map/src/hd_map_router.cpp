#include "hd_map_router.hpp"

HDMapRouter::HDMapRouter() { }


lanelet::GPSPoint ros_gps_msg_to_lanelet_gps_point(sensor_msgs::msg::NavSatFix::SharedPtr gps_msg) {
    lanelet::GPSPoint gpsPoint = lanelet::GPSPoint();
    gpsPoint.lat = gps_msg->latitude;
    gpsPoint.lon = gps_msg->longitude;
    gpsPoint.ele = gps_msg->altitude;

    return gpsPoint;
}

bool HDMapRouter::set_lanelet(const lanelet::LaneletMapPtr &lanelet_ptr){
    // TODO: Build traffic rules factory and assign routing graph
    this->lanelet_ptr_ = lanelet_ptr;

    /* 
    **OPTIONAL**
    //Routing Costs
    double laneChangeCost = <add-lane-change-cost>;
    RoutingCostPtrs costPtrs{std::make_shared<RoutingCostDistance>(laneChangeCost)};
    //Config
    RoutingGraph::Configuration routingGraphConf;
    routingGraphConf.emplace(std::make_pair(RoutingGraph::ParticipantHeight, Attribute("2.")));
    */
    lanelet::traffic_rules::TrafficRulesPtr trafficRules{lanelet::traffic_rules::TrafficRulesFactory::instance().create(lanelet::Locations::Germany, lanelet::Participants::Vehicle)};
    lanelet::routing::RoutingGraphPtr graph = lanelet::routing::RoutingGraph::build(*this->lanelet_ptr_, *trafficRules);

    if (graph == nullptr){
        RCLCPP_INFO(rclcpp::get_logger("hd_map_router"), "Routing Graph Build Failed...");
        return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("hd_map_router"), "Routing Graph Build Success...");
    return true;
    
}

lanelet::ConstLanelet HDMapRouter::get_nearest_lanelet_to_gps(lanelet::GPSPoint gps_point){
    // TODO: implemtation of function to get nearest lanelet in lanelet map to gps point
}

lanelet::Optional<lanelet::routing::Route> HDMapRouter::route(lanelet::GPSPoint from_point, lanelet::GPSPoint to_point){
    return route(get_nearest_lanelet_to_gps(from_point), get_nearest_lanelet_to_gps(to_point));
}

lanelet::Optional<lanelet::routing::Route> HDMapRouter::route(lanelet::ConstLanelet from_lanelet, lanelet::ConstLanelet to_lanelet){
    //TODO: Implementation of route function to get shortest path from from_lanelet to to_lanelet
    //https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_examples/src/06_routing/main.cpp

}

lanelet::LaneletMapPtr HDMapRouter::get_lanelet(){
    return this->lanelet_ptr_;
}

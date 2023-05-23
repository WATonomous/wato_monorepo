#include <string>
#include <vector>

#include "hd_map_routing.hpp"

namespace world_modeling::hd_map
{
    HDMapRouting::HDMapRouting() {
        std::string exampleMapPath = "/home/docker/ament_ws/src/maps/osm/Town05.osm";

        this->map_ = lanelet::load(exampleMapPath, lanelet::projection::UtmProjector(lanelet::Origin({0, 0})));
        // lanelet::traffic_rules::TrafficRulesPtr trafficRules =
            // lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Vehicle);

        // this->routing_graph_ = lanelet::routing::RoutingGraph::build(*(this->map_), *trafficRules);
    }
    
} 

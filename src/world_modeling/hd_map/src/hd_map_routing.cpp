#include <string>
#include <vector>

#include "hd_map_routing.hpp"

namespace world_modeling::hd_map
{
    HDMapRouting::HDMapRouting() {
        std::string exampleMapPath = "/home/docker/ament_ws/src/maps/osm/firetower.osm";

        this->map_ = lanelet::load(exampleMapPath, lanelet::projection::UtmProjector(lanelet::Origin({43.4327979, -80.5823243})));
        // lanelet::traffic_rules::TrafficRulesPtr trafficRules =
            // lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Vehicle);

        // this->routing_graph_ = lanelet::routing::RoutingGraph::build(*(this->map_), *trafficRules);
    }
    
} 

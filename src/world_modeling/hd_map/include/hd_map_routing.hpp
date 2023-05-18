#include <vector>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

namespace world_modeling::hd_map
{
  class HDMapRouting
  {

  public:
    HDMapRouting();

    lanelet::LaneletMapPtr map_;
    // lanelet::routing::RoutingGraphUPtr routing_graph_;
  };
}  
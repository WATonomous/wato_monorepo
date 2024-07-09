#ifndef WORLD_MODELING_TRAFFIC_LIGHT_REG_ELEM_
#define WORLD_MODELING_TRAFFIC_LIGHT_REG_ELEM_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/RegulartoryElement.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Exceptions.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include "hd_map_router.hpp"

class TrafficLightRegElem : public lanelet::RegulatoryElement{
    public:
        // lanelet2 looks for this string when matching the subtype of a regulatory element to the respective type
        static constexpr char RuleName[] = "traffic_light";

        static std::shared_ptr<TrafficLightRegElem> make(const lanelet::BoundingBox3d& bbox, const std::string& state);

    private:
    
    // The following lines are required so that the lanelet library can create the PedestrianRegElem object
    // Refer to : https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_examples/src/02_regulatory_elements/main.cpp

    friend class lanelet::RegulatoryElement<TrafficLightRegElem>;
    explicit TrafficLightRegElem(const lanelet::RegulatoryElementDataPtr& data) : RegulatoryElement(data) {}
};

namespace {
    lanelet::RegisterRegulatoryElement<TrafficLightRegElem> registerTrafficLightRegElem;
}  // namespace

#endif
#ifndef WORLD_MODELING_PEDESTRIAN_REG_ELEM_
#define WORLD_MODELING_PEDESTRIAN_REG_ELEM_

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
#include "hd_map_router.hpp"

class PedestrianRegElem : public lanelet::RegulatoryElement{
    public:
        PedestrianRegElem() = default;
        static constexpr char RuleName[] = "pedestrian";

        // Factory method to create a new instance
        static std::shared_ptr<PedestrianRegElem> make(const lanelet::BoundingBox3d& pedestrianBBox);

        // Get the pedestrian bounding box
        lanelet::BoundingBox3d pedestrianBBox() const;
    
    private:
        // The following lines are required so that the lanelet library can create the PedestrianRegElem object
        // Refer to : https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_examples/src/02_regulatory_elements/main.cpp

        // Constructor used by the factory method
    explicit PedestrianRegElem(const lanelet::RegulatoryElementDataPtr& data);

    // Required for Lanelet2 library to create the PedestrianRegElem object
    friend class lanelet::RegisterRegulatoryElement<PedestrianRegElem>;
};

#endif
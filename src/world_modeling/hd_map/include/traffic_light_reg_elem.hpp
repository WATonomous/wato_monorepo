#ifndef WORLD_MODELING_TRAFFIC_LIGHT_REG_ELEM_
#define WORLD_MODELING_TRAFFIC_LIGHT_REG_ELEM_

#include "rclcpp/rclcpp.hpp"
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/LaneletMap.h>

enum TrafficLightState {
    Red,
    Yellow,
    Green,
    Unknown,
};

class TrafficLightRegElem : public lanelet::RegulatoryElement{
    public:
        // lanelet2 looks for this string when matching the subtype of a regulatory element to the respective type
        static constexpr const char RuleName[] = "traffic_light";

        static std::shared_ptr<TrafficLightRegElem> make(const lanelet::BoundingBox3d& bbox, const TrafficLightState& state, uint64_t id);

        uint64_t getId() const;
        
        TrafficLightState getState() const;

        void updateTrafficLight(const lanelet::BoundingBox3d& bbox, const TrafficLightState& state);

    private:
        TrafficLightState state;
        uint64_t id;

        // The following lines are required so that the lanelet library can create the PedestrianRegElem object
        // Refer to : https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_examples/src/02_regulatory_elements/main.cpp
        friend class lanelet::RegisterRegulatoryElement<TrafficLightRegElem>;
        explicit TrafficLightRegElem(const lanelet::RegulatoryElementDataPtr& data, const TrafficLightState& state, uint64_t id = 0);
};

#endif

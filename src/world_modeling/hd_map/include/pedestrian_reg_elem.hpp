#ifndef WORLD_MODELING_PEDESTRIAN_REG_ELEM_
#define WORLD_MODELING_PEDESTRIAN_REG_ELEM_

#include "rclcpp/rclcpp.hpp"
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/LaneletMap.h>

// TODO: Define the enum for the different pedestrian states (see traffic_light_reg_elem.hpp)

class PedestrianRegElem : public lanelet::RegulatoryElement{
    public:
        // lanelet2 looks for this string when matching the subtype of a regulatory element to the respective type
        static constexpr char RuleName[] = "pedestrian";

        static std::shared_ptr<PedestrianRegElem> make(const lanelet::BoundingBox3d& pedestrianBBox, uint64_t id);

        void updatePedestrian(const lanelet::BoundingBox3d& pedestrianBBox);

        uint64_t getId() const;
    
    private:
        uint64_t id;

        // The following lines are required so that the lanelet library can create the PedestrianRegElem object
        // Refer to : https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_examples/src/02_regulatory_elements/main.cpp
        explicit PedestrianRegElem(const lanelet::RegulatoryElementDataPtr& data, uint64_t id = 0);
        friend class lanelet::RegisterRegulatoryElement<PedestrianRegElem>;
};

#endif

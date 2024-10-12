#ifndef WORLD_MODELING_PEDESTRIAN_REG_ELEM_
#define WORLD_MODELING_PEDESTRIAN_REG_ELEM_

#include "rclcpp/rclcpp.hpp"
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/LaneletMap.h>

class PedestrianRegElem : public lanelet::RegulatoryElement{
    public:
        static constexpr char RuleName[] = "pedestrian";

        // Factory method to create a new instance
        static std::shared_ptr<PedestrianRegElem> make(const lanelet::BoundingBox3d& pedestrianBBox, uint64_t id);

        // Update pedestrian bounding box
        void updatePedestrian(const lanelet::BoundingBox3d& pedestrianBBox);

        // Get Non-lanelet id
        uint64_t getId();
    
    private:
        uint64_t id;

        // The following lines are required so that the lanelet library can create the PedestrianRegElem object
        // Refer to : https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_examples/src/02_regulatory_elements/main.cpp

        // Constructor used by the factory method
        explicit PedestrianRegElem(const lanelet::RegulatoryElementDataPtr& data, uint64_t id = 0);

        // Required for Lanelet2 library to create the PedestrianRegElem object
        friend class lanelet::RegisterRegulatoryElement<PedestrianRegElem>;
};

#endif

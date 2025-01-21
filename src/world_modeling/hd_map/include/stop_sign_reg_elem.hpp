#ifndef WORLD_MODELING_STOP_SIGN_REG_ELEM_
#define WORLD_MODELING_STOP_SIGN_REG_ELEM_

#include "rclcpp/rclcpp.hpp"
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/LaneletMap.h>

// TODO: Define the enum for the different traffic sign states (see traffic_light_reg_elem.hpp)

class StopSignRegElem : public lanelet::RegulatoryElement{
    public:
        static constexpr char RuleName[] = "stop_sign";

        static std::shared_ptr<StopSignRegElem> make(const lanelet::BoundingBox3d& stopSignBBox, uint64_t id);

        void updateStopSign(const lanelet::BoundingBox3d& stopSignBBox);

        uint64_t getId() const;

    private:
        uint64_t id;

        // The following lines are required so that the lanelet library can create the PedestrianRegElem object
        // Refer to : https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_examples/src/02_regulatory_elements/main.cpp
        friend class lanelet::RegisterRegulatoryElement<StopSignRegElem>;
        explicit StopSignRegElem(const lanelet::RegulatoryElementDataPtr& data, uint64_t id);
};

#endif
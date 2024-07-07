#include "pedestrian_reg_elem.hpp"

// Static factory method to create a new instance
std::shared_ptr<PedestrianRegElem> PedestrianRegElem::make(const lanelet::BoundingBox3d& pedestrianBBox) {
    lanelet::RegulatoryElementDataPtr data = std::make_shared<lanelet::RegulatoryElementData>();
    data->attributes()[lanelet::AttributeName::Type] = RuleName;
    data->parameters()["pedestrian_bbox"].emplace_back(pedestrianBBox);

    return std::shared_ptr<PedestrianRegElem>(new PedestrianRegElem(data));
}

// Constructor used by the factory method
PedestrianRegElem::PedestrianRegElem(const lanelet::RegulatoryElementDataPtr& data) 
    : lanelet::RegulatoryElement(data) {}

// Get the pedestrian bounding box
lanelet::BoundingBox3d PedestrianRegElem::pedestrianBBox() const {
    return parameters()["pedestrian_bbox"].front().asBoundingBox();
}

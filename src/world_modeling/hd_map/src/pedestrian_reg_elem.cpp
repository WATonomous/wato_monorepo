#include "pedestrian_reg_elem.hpp"
#include "utils.hpp"

// Static factory method to create a new instance
std::shared_ptr<PedestrianRegElem> PedestrianRegElem::make(const lanelet::BoundingBox3d& pedestrianBBox, uint64_t id) {
    // Create a polygon from the bounding box
    auto pedestrian = utils::boundingBox3dToPolygon3d(pedestrianBBox);
    lanelet::RuleParameterMap rpm = {{lanelet::RoleNameString::Refers, {pedestrian}}};
    lanelet::AttributeMap am;
    auto data = std::make_shared<lanelet::RegulatoryElementData>(lanelet::utils::getId(), std::move(rpm), am);
    data->attributes[lanelet::AttributeName::Type] = lanelet::AttributeValueString::RegulatoryElement;
    data->attributes[lanelet::AttributeName::Subtype] = RuleName;

    return std::shared_ptr<PedestrianRegElem>(new PedestrianRegElem(data, id));
}

void PedestrianRegElem::updatePedestrian(const lanelet::BoundingBox3d& pedestrianBBox){
    // Create a polygon from the bounding box
    auto pedestrian = utils::boundingBox3dToPolygon3d(pedestrianBBox);
    parameters()[lanelet::RoleName::Refers].clear();
    parameters()[lanelet::RoleName::Refers].emplace_back(pedestrian);
}

uint64_t PedestrianRegElem::getId(){
  return id;
}

PedestrianRegElem::PedestrianRegElem(const lanelet::RegulatoryElementDataPtr& data, uint64_t id) 
    : lanelet::RegulatoryElement(data), id{id} {}

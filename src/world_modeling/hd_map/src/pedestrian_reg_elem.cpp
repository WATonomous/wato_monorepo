#include "pedestrian_reg_elem.hpp"

lanelet::Polygon3d boundingBox3dToPolygon3d(const lanelet::BoundingBox3d& bbox){
    auto min = bbox.min();
    auto max = bbox.max();

    lanelet::Polygon3d polygon{
        lanelet::utils::getId(),
        {
          lanelet::Point3d(lanelet::utils::getId(), min.x(), min.y(), min.z()),
          lanelet::Point3d(lanelet::utils::getId(), max.x(), min.y(), min.z()),
          lanelet::Point3d(lanelet::utils::getId(), max.x(), max.y(), min.z()),
          lanelet::Point3d(lanelet::utils::getId(), min.x(), max.y(), min.z()),
          lanelet::Point3d(lanelet::utils::getId(), min.x(), min.y(), max.z()),
          lanelet::Point3d(lanelet::utils::getId(), max.x(), min.y(), max.z()),
          lanelet::Point3d(lanelet::utils::getId(), max.x(), max.y(), max.z()),
          lanelet::Point3d(lanelet::utils::getId(), min.x(), max.y(), max.z())
        }
    };

    return polygon;
}

// Static factory method to create a new instance
std::shared_ptr<PedestrianRegElem> PedestrianRegElem::make(const lanelet::BoundingBox3d& pedestrianBBox, uint64_t id) {
    // Create a polygon from the bounding box
    auto pedestrian = boundingBox3dToPolygon3d(pedestrianBBox);
    lanelet::RuleParameterMap rpm = {{lanelet::RoleNameString::Refers, {pedestrian}}};
    lanelet::AttributeMap am;
    auto data = std::make_shared<lanelet::RegulatoryElementData>(lanelet::utils::getId(), std::move(rpm), am);
    data->attributes[lanelet::AttributeName::Type] = lanelet::AttributeValueString::RegulatoryElement;
    data->attributes[lanelet::AttributeName::Subtype] = RuleName;

    return std::shared_ptr<PedestrianRegElem>(new PedestrianRegElem(data, id));
}

void PedestrianRegElem::updatePedestrian(const lanelet::BoundingBox3d& pedestrianBBox){
    // Create a polygon from the bounding box
    auto pedestrian = boundingBox3dToPolygon3d(pedestrianBBox);
    parameters()[lanelet::RoleName::Refers].clear();
    parameters()[lanelet::RoleName::Refers].emplace_back(pedestrian);
}

uint64_t PedestrianRegElem::getId(){
  return id;
}

PedestrianRegElem::PedestrianRegElem(const lanelet::RegulatoryElementDataPtr& data, uint64_t id) 
    : lanelet::RegulatoryElement(data), id{id} {}

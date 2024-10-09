#include "traffic_light_reg_elem.hpp"

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

std::shared_ptr<TrafficLightRegElem> TrafficLightRegElem::make(const lanelet::BoundingBox3d& bbox, const std::string& color, uint64_t id) {
    auto traffic_light = boundingBox3dToPolygon3d(bbox);
    lanelet::RuleParameterMap rpm = {{lanelet::RoleNameString::Refers, {traffic_light}}};
    lanelet::AttributeMap am = {};

    std::shared_ptr data = std::make_shared<lanelet::RegulatoryElementData>(lanelet::utils::getId(), std::move(rpm), am);

    // add data to general RegElem
    data->attributes[lanelet::AttributeName::Type] = lanelet::AttributeValueString::RegulatoryElement;
    data->attributes[lanelet::AttributeName::Subtype] = RuleName;

    return std::make_shared<TrafficLightRegElem>(new TrafficLightRegElem(data, color, id));
}

// constructor
TrafficLightRegElem::TrafficLightRegElem(const lanelet::RegulatoryElementDataPtr& data, const std::string &color, uint64_t id)
    : lanelet::RegulatoryElement(data), color{color}, id{id} {}

// Getters

uint64_t TrafficLightRegElem::getId() const {
    return id;
}

std::string TrafficLightRegElem::getColor() const {
    return color;
}

// Setters
void TrafficLightRegElem::updateTrafficLight(const lanelet::BoundingBox3d& bbox, const std::string& color) {
    auto traffic_light = boundingBox3dToPolygon3d(bbox);
    parameters()[lanelet::RoleName::Refers].clear();
    parameters()[lanelet::RoleName::Refers].emplace_back(traffic_light);

    this->color = color;
}

#include "stop_sign_reg_elem.hpp"
#include "utils.hpp"

// Static factory method to create a new instance
std::shared_ptr<StopSignRegElem> StopSignRegElem::make(const lanelet::BoundingBox3d& stopSignBBox, uint64_t id) {
    auto stop_sign = utils::boundingBox3dToPolygon3d(stopSignBBox);
    lanelet::RuleParameterMap rpm = {{lanelet::RoleNameString::Refers, {stop_sign}}}; // Stop sign polygon in parameters
    lanelet::AttributeMap am;

    std::shared_ptr data = std::make_shared<lanelet::RegulatoryElementData>(lanelet::utils::getId(), std::move(rpm), am);

    data->attributes[lanelet::AttributeName::Type] = lanelet::AttributeValueString::RegulatoryElement;
    data->attributes[lanelet::AttributeName::Subtype] = RuleName;

}

// Constructor
StopSignRegElem::StopSignRegElem(const lanelet::RegulatoryElementDataPtr& data, uint64_t id) : lanelet::RegulatoryElement(data), id{id} {}

void StopSignRegElem::updateStopSign(const lanelet::BoundingBox3d& bbox) {
    auto stop_sign = utils::boundingBox3dToPolygon3d(bbox);
    parameters()[lanelet::RoleName::Refers].clear();
    parameters()[lanelet::RoleName::Refers].emplace_back(stop_sign); // Update the stop sign polygon
}

uint64_t StopSignRegElem::getId() const {
  return id;
}

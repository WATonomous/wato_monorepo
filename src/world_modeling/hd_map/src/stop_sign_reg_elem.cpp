#include "stop_sign_reg_elem.hpp"
#include "utils.hpp"

std::shared_ptr<StopSignRegElem> StopSignRegElem::make(const lanelet::BoundingBox3d& stopSignBBox, uint64_t id) {
    // Create a polygon from the bouding box
    auto stop_sign = utils::boundingBox3dToPolygon3d(stopSignBBox);
    lanelet::RuleParameterMap rpm = {{lanelet::RoleNameString::Refers, {stop_sign}}};
    lanelet::AttributeMap am;

    std::shared_ptr data = std::make_shared<lanelet::RegulatoryElementData>(lanelet::utils::getId(), std::move(rpm), am);

    // add data to general RegElem
    data->attributes[lanelet::AttributeName::Type] = lanelet::AttributeValueString::RegulatoryElement;
    data->attributes[lanelet::AttributeName::Subtype] = RuleName;

}

StopSignRegElem::StopSignRegElem(const lanelet::RegulatoryElementDataPtr& data, uint64_t id) : lanelet::RegulatoryElement(data), id{id} {}

uint64_t StopSignRegElem::getId(){
  return id;
}

void StopSignRegElem::updateStopSign(const lanelet::BoundingBox3d& bbox) {
    auto stop_sign = utils::boundingBox3dToPolygon3d(bbox);
    parameters()[lanelet::RoleName::Refers].clear();
    parameters()[lanelet::RoleName::Refers].emplace_back(stop_sign);

}

// NOTE : this is only a basic overview of the functions for the StopSignRegElem, more functions will be needed
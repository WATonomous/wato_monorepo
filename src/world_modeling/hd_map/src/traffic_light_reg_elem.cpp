#include "traffic_light_reg_elem.hpp"
#include "utils.hpp"


std::shared_ptr<TrafficLightRegElem> TrafficLightRegElem::make(const lanelet::BoundingBox3d& bbox, const TrafficLightState& state, uint64_t id) {
    auto traffic_light = utils::boundingBox3dToPolygon3d(bbox);
    lanelet::RuleParameterMap rpm = {{lanelet::RoleNameString::Refers, {traffic_light}}}; // Traffic light polygon in parameters
    lanelet::AttributeMap am = {};

    std::shared_ptr data = std::make_shared<lanelet::RegulatoryElementData>(lanelet::utils::getId(), std::move(rpm), am);

    data->attributes[lanelet::AttributeName::Type] = lanelet::AttributeValueString::RegulatoryElement;
    data->attributes[lanelet::AttributeName::Subtype] = RuleName;

    return std::shared_ptr<TrafficLightRegElem>(new TrafficLightRegElem(data, state, id));
}

// Constructor
TrafficLightRegElem::TrafficLightRegElem(const lanelet::RegulatoryElementDataPtr& data, const TrafficLightState& state, uint64_t id)
    : lanelet::RegulatoryElement(data), state{state}, id{id} {}

void TrafficLightRegElem::updateTrafficLight(const lanelet::BoundingBox3d& bbox, const TrafficLightState& state) {
    auto traffic_light = utils::boundingBox3dToPolygon3d(bbox);
    parameters()[lanelet::RoleName::Refers].clear();
    parameters()[lanelet::RoleName::Refers].emplace_back(traffic_light); // Update the traffic light polygon

    this->state = state;
}

uint64_t TrafficLightRegElem::getId() const {
    return id;
}

TrafficLightState TrafficLightRegElem::getState() const {
    return state;
}



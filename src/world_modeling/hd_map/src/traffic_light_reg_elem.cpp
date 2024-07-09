#include "traffic_light_reg_elem.hpp"

std::shared_ptr<TrafficLightRegElem> TrafficLightRegElem::make(const lanlet::BoundingBox3d& bbox, const std::string& string) {
    std::shared_ptr data = std::make_shared<lanelet::RegulatoryElement>();

    // add data to general RegElem
    data->attributes()[lanelet::AttributeName::Type] = RuleName;
    data->parameters()["traffic_light_bbox"].emplace_back(bbox);
    data->parameters()["traffic_light_state"].emplace_back(state);

    return std::make_shared<TrafficLightRegElem>(TrafficLightRegElem(data))
}

// constructor
TrafficLightRegElem::TrafficLightRegElem(const lanelet::RegulatoryElementDataPtr& data)
    : lanelet::RegulatoryElement(data) {}

// Getters
lanelet::BoundingBox3d TrafficLightRegElem::bbox() const {
    return parameters()["traffic_light_bbox"].front().asBoundingBox();
}

std::string TrafficLightRegElem::state() const {
    return parameters()["traffic_light_state"].front().asString();
}
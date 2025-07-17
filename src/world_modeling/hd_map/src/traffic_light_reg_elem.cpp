// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "traffic_light_reg_elem.hpp"

#include "utils.hpp"

// Static factory method to create a new instance
std::shared_ptr<TrafficLightRegElem> TrafficLightRegElem::make(
  const lanelet::BoundingBox3d & bbox, const TrafficLightState & state, uint64_t id)
{
  auto traffic_light = utils::boundingBox3dToPolygon3d(bbox);
  lanelet::RuleParameterMap rpm = {
    {lanelet::RoleNameString::Refers, {traffic_light}}};  // Traffic light polygon in parameters
  lanelet::AttributeMap am = {};

  std::shared_ptr data = std::make_shared<lanelet::RegulatoryElementData>(lanelet::utils::getId(), std::move(rpm), am);

  data->attributes[lanelet::AttributeName::Type] = lanelet::AttributeValueString::RegulatoryElement;
  data->attributes[lanelet::AttributeName::Subtype] = RuleName;

  return std::shared_ptr<TrafficLightRegElem>(new TrafficLightRegElem(data, state, id));
}

// Constructor
TrafficLightRegElem::TrafficLightRegElem(
  const lanelet::RegulatoryElementDataPtr & data, const TrafficLightState & state, uint64_t id)
: lanelet::RegulatoryElement(data)
, state{state}
, id{id}
{}

void TrafficLightRegElem::updateTrafficLight(const lanelet::BoundingBox3d & bbox, const TrafficLightState & state)
{
  auto traffic_light = utils::boundingBox3dToPolygon3d(bbox);
  parameters()[lanelet::RoleName::Refers].clear();
  parameters()[lanelet::RoleName::Refers].emplace_back(traffic_light);  // Update the traffic light polygon

  this->state = state;
}

uint64_t TrafficLightRegElem::getId() const
{
  return id;
}

TrafficLightState TrafficLightRegElem::getState() const
{
  return state;
}

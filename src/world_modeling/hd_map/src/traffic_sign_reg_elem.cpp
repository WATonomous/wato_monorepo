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

#include "hd_map/traffic_sign_reg_elem.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "hd_map/utils.hpp"

// Static factory method to create a new instance

std::shared_ptr<TrafficSignRegElem> TrafficSignRegElem::make(
  const lanelet::BoundingBox3d & bbox, const TrafficSignSubtype & subtype, uint64_t id)
{
  auto traffic_sign = utils::boundingBox3dToPolygon3d(bbox);

  lanelet::RuleParameterMap rpm = {{lanelet::RoleNameString::Refers, {traffic_sign}}};
  lanelet::AttributeMap am = {};

  std::shared_ptr data = std::make_shared<lanelet::RegulatoryElementData>(lanelet::utils::getId(), std::move(rpm), am);

  data->attributes[lanelet::AttributeName::Type] = lanelet::AttributeValueString::RegulatoryElement;
  data->attributes[lanelet::AttributeName::Subtype] = RuleName;

  return std::shared_ptr<TrafficSignRegElem>(new TrafficSignRegElem(data, subtype, id));
}

// Constructor
TrafficSignRegElem::TrafficSignRegElem(
  const lanelet::RegulatoryElementDataPtr & data, const TrafficSignSubtype & subtype, uint64_t id)
: lanelet::RegulatoryElement(data)
, subtype{subtype}
, id{id}
{}

void TrafficSignRegElem::updateTrafficSign(const lanelet::BoundingBox3d & bbox, const TrafficSignSubtype & subtype)
{
  auto traffic_sign = utils::boundingBox3dToPolygon3d(bbox);

  parameters()[lanelet::RoleName::Refers].clear();
  parameters()[lanelet::RoleName::Refers].emplace_back(traffic_sign);  // Update the traffic sign position

  this->subtype = subtype;
}

uint64_t TrafficSignRegElem::getId() const
{
  return id;
}

TrafficSignSubtype TrafficSignRegElem::getSubtype() const
{
  return subtype;
}

const std::unordered_map<std::string, TrafficSignSubtype> TrafficSignRegElem::classIdToSubtypeMap = {
  {"stop", TrafficSignSubtype::StopSign},
  {"yield", TrafficSignSubtype::YieldSign},
  {"speed_limit_15", TrafficSignSubtype::SpeedLimit15},
  {"speed_limit_30", TrafficSignSubtype::SpeedLimit30},
  {"speed_limit_40", TrafficSignSubtype::SpeedLimit40},
  {"speed_limit_60", TrafficSignSubtype::SpeedLimit60}};

TrafficSignSubtype TrafficSignRegElem::getSubtypeFromClassId(const std::string & class_id)
{
  auto it = classIdToSubtypeMap.find(class_id);
  return (it != classIdToSubtypeMap.end()) ? it->second : TrafficSignSubtype::UnknownSign;
}

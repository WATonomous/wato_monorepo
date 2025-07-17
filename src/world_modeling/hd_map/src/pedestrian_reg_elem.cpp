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

#include "pedestrian_reg_elem.hpp"

#include "utils.hpp"

// Static factory method to create a new instance
std::shared_ptr<PedestrianRegElem> PedestrianRegElem::make(const lanelet::BoundingBox3d & pedestrianBBox, uint64_t id)
{
  auto pedestrian = utils::boundingBox3dToPolygon3d(pedestrianBBox);
  lanelet::RuleParameterMap rpm = {
    {lanelet::RoleNameString::Refers, {pedestrian}}};  // Pedestrian polygon in parameters
  lanelet::AttributeMap am;

  std::shared_ptr data = std::make_shared<lanelet::RegulatoryElementData>(lanelet::utils::getId(), std::move(rpm), am);

  data->attributes[lanelet::AttributeName::Type] = lanelet::AttributeValueString::RegulatoryElement;
  data->attributes[lanelet::AttributeName::Subtype] = RuleName;

  return std::shared_ptr<PedestrianRegElem>(new PedestrianRegElem(data, id));
}

// Constructor
PedestrianRegElem::PedestrianRegElem(const lanelet::RegulatoryElementDataPtr & data, uint64_t id)
: lanelet::RegulatoryElement(data)
, id{id}
{}

void PedestrianRegElem::updatePedestrian(const lanelet::BoundingBox3d & pedestrianBBox)
{
  auto pedestrian = utils::boundingBox3dToPolygon3d(pedestrianBBox);
  parameters()[lanelet::RoleName::Refers].clear();
  parameters()[lanelet::RoleName::Refers].emplace_back(pedestrian);  // Update pedestrian polygon
}

uint64_t PedestrianRegElem::getId() const
{
  return id;
}

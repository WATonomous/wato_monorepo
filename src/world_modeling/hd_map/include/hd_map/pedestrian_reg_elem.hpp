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

#ifndef WORLD_MODELING_PEDESTRIAN_REG_ELEM_
#define WORLD_MODELING_PEDESTRIAN_REG_ELEM_

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <memory>  // Required for std::shared_ptr

#include "rclcpp/rclcpp.hpp"

// TODO(wato): Define the enum for the different pedestrian states (see traffic_light_reg_elem.hpp)

class PedestrianRegElem : public lanelet::RegulatoryElement
{
public:
  static constexpr char RuleName[] = "pedestrian";

  static std::shared_ptr<PedestrianRegElem> make(const lanelet::BoundingBox3d & pedestrianBBox, uint64_t id);

  void updatePedestrian(const lanelet::BoundingBox3d & pedestrianBBox);

  uint64_t getId() const;

private:
  uint64_t id;

  explicit PedestrianRegElem(const lanelet::RegulatoryElementDataPtr & data, uint64_t id = 0);
  friend class lanelet::RegisterRegulatoryElement<PedestrianRegElem>;
};

#endif

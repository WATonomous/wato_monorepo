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

#ifndef WORLD_MODELING_TRAFFIC_LIGHT_REG_ELEM_
#define WORLD_MODELING_TRAFFIC_LIGHT_REG_ELEM_

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

enum TrafficLightState
{
  Red,
  Yellow,
  Green,
  UnknownLight
};

class TrafficLightRegElem : public lanelet::RegulatoryElement
{
public:
  // lanelet2 looks for this string when matching the subtype of a regulatory element to the
  // respective type
  static constexpr const char RuleName[] = "traffic_light";

  static std::shared_ptr<TrafficLightRegElem> make(
    const lanelet::BoundingBox3d & bbox, const TrafficLightState & state, uint64_t id);

  uint64_t getId() const;

  TrafficLightState getState() const;

  void updateTrafficLight(const lanelet::BoundingBox3d & bbox, const TrafficLightState & state);

private:
  TrafficLightState state;
  uint64_t id;

  // The following lines are required so that the lanelet library can create the PedestrianRegElem
  // object Refer to :
  // https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_examples/src/02_regulatory_elements/main.cpp
  friend class lanelet::RegisterRegulatoryElement<TrafficLightRegElem>;
  explicit TrafficLightRegElem(
    const lanelet::RegulatoryElementDataPtr & data, const TrafficLightState & state, uint64_t id = 0);
};

#endif

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

#ifndef WORLD_MODELING_TRAFFIC_SIGN_REG_ELEM_
#define WORLD_MODELING_TRAFFIC_SIGN_REG_ELEM_

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"

enum TrafficSignSubtype
{
  StopSign,
  YieldSign,
  SpeedLimit15,
  SpeedLimit30,
  SpeedLimit40,
  SpeedLimit60,
  UnknownSign,
};

class TrafficSignRegElem : public lanelet::RegulatoryElement
{
public:
  // lanelet2 looks for this string when matching the subtype of a regulatory element to the
  // respective type
  static constexpr const char RuleName[] = "traffic_sign";

  static std::shared_ptr<TrafficSignRegElem> make(
    const lanelet::BoundingBox3d & bbox, const TrafficSignSubtype & subtype, uint64_t id);

  uint64_t getId() const;

  TrafficSignSubtype getSubtype() const;
  static TrafficSignSubtype getSubtypeFromClassId(const std::string & class_id);

  void updateTrafficSign(const lanelet::BoundingBox3d & bbox, const TrafficSignSubtype & subtype);

private:
  TrafficSignSubtype subtype;
  uint64_t id;

  static const std::unordered_map<std::string, TrafficSignSubtype> classIdToSubtypeMap;

  // The following lines are required so that the lanelet library can create the TrafficSignRegElem
  // object Refer to :
  // https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_examples/src/02_regulatory_elements/main.cpp
  friend class lanelet::RegisterRegulatoryElement<TrafficSignRegElem>;
  explicit TrafficSignRegElem(
    const lanelet::RegulatoryElementDataPtr & data, const TrafficSignSubtype & subtype, uint64_t id = 0);
};

#endif

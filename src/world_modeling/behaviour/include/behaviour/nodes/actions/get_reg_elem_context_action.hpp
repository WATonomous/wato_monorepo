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

#ifndef BEHAVIOUR__NODES__ACTIONS__GET_REG_ELEM_CONTEXT_ACTION_HPP_
#define BEHAVIOUR__NODES__ACTIONS__GET_REG_ELEM_CONTEXT_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "behaviour/dynamic_object_store.hpp"
#include "behaviour/utils/utils.hpp"

namespace behaviour
{
/**
   * @class GetRegElemContextAction
   * @brief BT sync action node to extract regulatory elements (traffic lights, stop signs, yields) from the planned path.
   *
   * Inputs:
   *   - lanelet_context: Current lane context containing ego position and current lanelet
   *   - path: The planned path containing lanelets
   *
   * Outputs:
   *   - reg_elems: Array of RegulatoryElement structs found in the path
   */
class GetRegElemContextAction : public BT::SyncActionNode
{
public:
  GetRegElemContextAction(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<types::CurrentLaneContextPtr>("lanelet_context", "Current lane context"),
      BT::InputPort<types::PathPtr>("path", "Planned path"),
      BT::OutputPort<types::RegulatoryElementArrayPtr>("reg_elems", "Array of regulatory elements found in path"),
    };
  }

  BT::NodeStatus tick() override
  {
    types::CurrentLaneContextPtr lanelet_context;
    types::PathPtr path;

    try {
      lanelet_context = ports::getPtr<types::CurrentLaneContext>(*this, "lanelet_context");
      path = ports::getPtr<types::Path>(*this, "path");
    } catch (const BT::RuntimeError & e) {
      return BT::NodeStatus::FAILURE;
    }

    if (path->lanelets.empty()) {
      return BT::NodeStatus::FAILURE;
    }

    const int64_t current_lanelet_id = lanelet_context->current_lanelet.id;
    const auto & lanelets = path->lanelets;

    // Find current lanelet index
    auto it = std::find_if(lanelets.begin(), lanelets.end(), [current_lanelet_id](const types::Lanelet & lanelet) {
      return lanelet.id == current_lanelet_id;
    });

    if (it == lanelets.end()) {
      return BT::NodeStatus::FAILURE;
    }

    auto reg_elems = std::make_shared<types::RegulatoryElementArray>();

    // Get the distance to the very first regulatory element from the current context
    // We take the minimum of all relevant distance fields in CurrentLaneContext
    double first_elem_distance = std::numeric_limits<double>::max();
    bool found_distance = false;

    if (lanelet_context->distance_to_traffic_light_m >= 0.0) {
      first_elem_distance = std::min(first_elem_distance, lanelet_context->distance_to_traffic_light_m);
      found_distance = true;
    }
    if (lanelet_context->distance_to_stop_line_m >= 0.0) {
      first_elem_distance = std::min(first_elem_distance, lanelet_context->distance_to_stop_line_m);
      found_distance = true;
    }
    if (lanelet_context->distance_to_yield_m >= 0.0) {
      first_elem_distance = std::min(first_elem_distance, lanelet_context->distance_to_yield_m);
      found_distance = true;
    }
    // Fallback to intersection distance if specific ones aren't set
    if (!found_distance && lanelet_context->distance_to_intersection_m >= 0.0) {
      first_elem_distance = lanelet_context->distance_to_intersection_m;
      found_distance = true;
    }

    if (!found_distance) {
      first_elem_distance = 0.0;
    }

    double accumulated_distance = 0.0;
    bool first_found = false;

    // Scan through lanelets starting from the current one
    for (auto current_it = it; current_it != lanelets.end(); ++current_it) {
      const auto & lanelet = *current_it;

      // Skip if not an intersection
      if (!lanelet.is_intersection) {
        continue;
      }

      types::RegulatoryElement elem;
      elem.id = lanelet.id;  // Default ID
      elem.stop_line_id = -1;

      if (lanelet.has_traffic_light) {
        elem.type = types::RegElemType::TRAFFIC_LIGHT;
        if (!lanelet.traffic_lights.empty()) {
          elem.id = lanelet.traffic_lights[0].id;
          elem.stop_line_id = lanelet.traffic_lights[0].stop_line_id;
        }
      } else if (lanelet.has_yield_sign) {
        elem.type = types::RegElemType::YIELD;
        if (!lanelet.stop_lines.empty()) {
          elem.stop_line_id = lanelet.stop_lines[0].id;
        }
      } else {
        // If it's an intersection but not TL or Yield, it's a stop sign
        elem.type = types::RegElemType::STOP_SIGN;
        if (!lanelet.stop_lines.empty()) {
          elem.stop_line_id = lanelet.stop_lines[0].id;
        }
      }

      // Distance logic:
      // The first reg elem found gets the distance from the CurrentLaneContext
      // Subsequent ones would add the distance between them (accumulated_distance logic)
      if (!first_found) {
        elem.distance = first_elem_distance;
        first_found = true;
      } else {
        // For now, if we find multiple, we'd need path geometry to calculate gaps.
        // We'll set it relative to the first one found.
        elem.distance = first_elem_distance + accumulated_distance;
      }

      reg_elems->push_back(elem);
    }

    setOutput("reg_elems", reg_elems);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__ACTIONS__GET_REG_ELEM_CONTEXT_ACTION_HPP_

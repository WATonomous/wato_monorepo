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

#ifndef BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_STOP_SIGN_PEDESTRIANS_ACTION_HPP_
#define BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_STOP_SIGN_PEDESTRIANS_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include "behaviour/nodes/bt_logger_base.hpp"

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>
#include <unordered_set>
#include <vector>

#include "behaviour/utils/utils.hpp"
#include "lanelet_msgs/msg/regulatory_element.hpp"
#include "world_model_msgs/msg/world_object.hpp"

namespace behaviour
{
/**
 * @class GetStopSignPedestriansAction
 * @brief Collects pedestrian IDs on the yield lanelets of a stop sign.
 *
 * Any pedestrian present on a yield lanelet is considered to be crossing
 * the intersection and must be yielded to before proceeding.
 */
class GetStopSignPedestriansAction : public BT::SyncActionNode, protected BTLoggerBase
{
public:
  GetStopSignPedestriansAction(const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::SyncActionNode(name, config)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<lanelet_msgs::msg::RegulatoryElement::SharedPtr>("stop_sign"),
      BT::InputPort<std::vector<world_model_msgs::msg::WorldObject>>("objects"),
      BT::InputPort<std::size_t>("hypothesis_index"),
      BT::OutputPort<std::vector<std::string>>("out_stop_sign_pedestrian_ids"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto missing = [this](const char * port) {
      RCLCPP_DEBUG_STREAM(logger(), "missing_input port=" << port);
    };

    auto stop_sign = ports::tryGetPtr<lanelet_msgs::msg::RegulatoryElement>(*this, "stop_sign");
    if (!ports::require(stop_sign, "stop_sign", missing)) return BT::NodeStatus::FAILURE;

    auto objects = ports::tryGet<std::vector<world_model_msgs::msg::WorldObject>>(*this, "objects");
    if (!ports::require(objects, "objects", missing)) return BT::NodeStatus::FAILURE;

    auto hypothesis_index = ports::tryGet<std::size_t>(*this, "hypothesis_index");
    if (!ports::require(hypothesis_index, "hypothesis_index", missing)) return BT::NodeStatus::FAILURE;

    std::vector<std::string> out_ids;
    std::unordered_set<std::string> seen_ids;

    for (const auto lanelet_id : stop_sign->yield_lanelet_ids) {
      const auto pedestrians =
        utils::world_objects::getPedestriansByLanelet(*objects, *hypothesis_index, lanelet_id);

      for (const auto * obj : pedestrians) {
        if (obj == nullptr) continue;
        if (!seen_ids.insert(obj->detection.id).second) continue;
        out_ids.push_back(obj->detection.id);
      }
    }

    setOutput("out_stop_sign_pedestrian_ids", out_ids);

    if (!out_ids.empty()) {
      RCLCPP_DEBUG_STREAM(
        logger(), "stop_sign_pedestrians_detected count=" << out_ids.size());
    }

    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_STOP_SIGN_PEDESTRIANS_ACTION_HPP_

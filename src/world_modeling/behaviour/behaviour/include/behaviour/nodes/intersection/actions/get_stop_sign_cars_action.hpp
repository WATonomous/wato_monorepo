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

#ifndef BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_STOP_SIGN_CARS_ACTION_HPP_
#define BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_STOP_SIGN_CARS_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>
#include <unordered_set>
#include <vector>

#include "behaviour/nodes/bt_logger_base.hpp"
#include "behaviour/utils/utils.hpp"
#include "world_model_msgs/msg/world_object.hpp"

namespace behaviour
{
/**
   * @class GetStopSignCarsAction
   * @brief SyncActionNode to collect car IDs occupying relevant right-of-way lanelets.
   */
class GetStopSignCarsAction : public BT::SyncActionNode, protected BTLoggerBase
{
public:
  GetStopSignCarsAction(const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::SyncActionNode(name, config)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<int64_t>>("lanelet_ids"),
      BT::InputPort<std::vector<world_model_msgs::msg::WorldObject>>("objects"),
      BT::InputPort<std::size_t>("hypothesis_index"),
      BT::OutputPort<std::vector<std::string>>("out_stop_sign_car_ids"),
      BT::OutputPort<std::vector<world_model_msgs::msg::WorldObject>>("out_stop_sign_cars"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      RCLCPP_DEBUG_STREAM(logger(), "missing_input port=" << port_name);
    };

    auto lanelet_ids = ports::tryGet<std::vector<int64_t>>(*this, "lanelet_ids");
    if (!ports::require(lanelet_ids, "lanelet_ids", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto objects = ports::tryGet<std::vector<world_model_msgs::msg::WorldObject>>(*this, "objects");
    if (!ports::require(objects, "objects", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto hypothesis_index = ports::tryGet<std::size_t>(*this, "hypothesis_index");
    if (!ports::require(hypothesis_index, "hypothesis_index", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    std::vector<std::string> out_ids;
    std::vector<world_model_msgs::msg::WorldObject> out_cars;
    std::unordered_set<std::string> seen_ids;
    for (const auto lanelet_id : *lanelet_ids) {
      const auto cars_in_lanelet = utils::world_objects::getCarsByLanelet(*objects, *hypothesis_index, lanelet_id);

      for (const auto * obj : cars_in_lanelet) {
        if (obj == nullptr) {
          continue;
        }

        if (!seen_ids.insert(obj->detection.id).second) {
          continue;
        }

        out_ids.push_back(obj->detection.id);
        out_cars.push_back(*obj);
      }
    }
    setOutput("out_stop_sign_car_ids", out_ids);
    setOutput("out_stop_sign_cars", out_cars);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_STOP_SIGN_CARS_ACTION_HPP_

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

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>
#include <unordered_set>
#include <vector>

#include "behaviour/utils/utils.hpp"

#include "lanelet_msgs/msg/regulatory_element.hpp"
#include "world_model_msgs/msg/world_object.hpp"

namespace behaviour
{
  /**
   * @class GetStopSignCarsAction
   * @brief SyncActionNode to collect stop-sign queued car IDs.
   */
  class GetStopSignCarsAction : public BT::SyncActionNode
  {
  public:
    GetStopSignCarsAction(const std::string &name, const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<lanelet_msgs::msg::RegulatoryElement::SharedPtr>("stop_sign"),
          BT::InputPort<std::vector<world_model_msgs::msg::WorldObject>>("objects"),
          BT::InputPort<int>("hypothesis_index"),
          BT::InputPort<double>("stop_sign_line_threshold_m"),
          BT::OutputPort<std::vector<std::string>>("out_stop_sign_car_ids"),
          BT::OutputPort<std::vector<world_model_msgs::msg::WorldObject>>("out_stop_sign_cars"),
      };
    }

    BT::NodeStatus tick() override
    {
      const auto missing_input_callback = [&](const char *port_name)
      {
        std::cout << "[GetStopSignCars] Missing " << port_name << " input" << std::endl;
      };

      auto stop_sign = ports::tryGetPtr<lanelet_msgs::msg::RegulatoryElement>(*this, "stop_sign");
      if (!ports::require(stop_sign, "stop_sign", missing_input_callback))
      {
        return BT::NodeStatus::FAILURE;
      }

      auto objects = ports::tryGet<std::vector<world_model_msgs::msg::WorldObject>>(*this, "objects");
      if (!ports::require(objects, "objects", missing_input_callback))
      {
        return BT::NodeStatus::FAILURE;
      }

      auto hypothesis_index = ports::tryGet<int>(*this, "hypothesis_index");
      if (!ports::require(hypothesis_index, "hypothesis_index", missing_input_callback))
      {
        return BT::NodeStatus::FAILURE;
      }

      auto stop_sign_line_threshold_m = ports::tryGet<double>(*this, "stop_sign_line_threshold_m");
      if (!ports::require(stop_sign_line_threshold_m, "stop_sign_line_threshold_m", missing_input_callback))
      {
        return BT::NodeStatus::FAILURE;
      }

      if (*hypothesis_index < 0)
      {
        std::cout << "[GetStopSignCars] invalid hypothesis_index" << std::endl;
        return BT::NodeStatus::FAILURE;
      }

      std::vector<std::string> out_ids;
      std::vector<world_model_msgs::msg::WorldObject> out_cars;
      std::unordered_set<std::string> seen_ids;

      for (const auto lanelet_id : stop_sign->yield_lanelet_ids)
      {
        const auto cars_in_lanelet = object_utils::getCarsByLanelet(*objects, *hypothesis_index, lanelet_id);

        for (const auto *obj : cars_in_lanelet)
        {
          if (obj == nullptr)
          {
            continue;
          }

          // todo clarify on the which ref line is the right one
          double min_ref_line_distance = std::numeric_limits<double>::infinity();
          for (const auto &ref_line : stop_sign->ref_lines)
          {
            if (ref_line.points.empty())
            {
              continue;
            }
            const double d = geometry::objectToRefLineDistanceXY(*obj, ref_line);
            if (d < min_ref_line_distance)
            {
              min_ref_line_distance = d;
            }
          }
          if (!std::isfinite(min_ref_line_distance) || min_ref_line_distance > *stop_sign_line_threshold_m)
          {
            continue;
          }

          if (!seen_ids.insert(obj->detection.id).second)
          {
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

} // namespace behaviour

#endif // BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_STOP_SIGN_CARS_ACTION_HPP_

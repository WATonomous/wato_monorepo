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

#ifndef BEHAVIOUR__PREDICTION_CONDITION_HPP_
#define BEHAVIOUR__PREDICTION_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <memory>
#include <string>
#include <vector>

namespace behaviour
{
  /**
   * @class PredictionCondition
   * @brief BT node to check if any dynamic objects' predicted trajectories will hit the car given a safe radius.
   * TODO(wato): fill the logic
   */
  class PredictionCondition : public BT::ConditionNode
  {
  public:
    PredictionCondition(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<std::shared_ptr<std::vector<std::string>>>("objects", "List of nearby objects to check"),
          BT::InputPort<double>("radius", 15.0, "Safe radius for predicted paths in meters")};
    }

    /**
     * @brief Evaluates predicted path safety.
     * @return SUCCESS if safe, FAILURE otherwise.
     */
    BT::NodeStatus tick() override
    {
      // TODO(wato): determine the object type
      auto objects = getInput<std::shared_ptr<std::vector<std::string>>>("objects");
      auto radius = getInput<double>("radius");

      if (!objects || !radius)
      {
        return BT::NodeStatus::FAILURE;
      }

      // Placeholder logic: For now, we assume it's always safe.

      return BT::NodeStatus::SUCCESS;
    }
  };
} // namespace behaviour

#endif // BEHAVIOUR__CHECK_PREDICTED_CONDITION_HPP_

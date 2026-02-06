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

#ifndef BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_TRAFFIC_LIGHT_STATE_ACTION_HPP_
#define BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_TRAFFIC_LIGHT_STATE_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

#include "behaviour/dynamic_object_store.hpp"
#include "behaviour/utils/ports.hpp"

#include "lanelet_msgs/msg/regulatory_element.hpp"

namespace behaviour
{
    /**
     * @class GetTrafficLightStateAction
     * @brief Gets the traffic light state from DynamicObjectStore using the active reg elem id.
     *
     * Assumptions (current implementation):
     * - The world object's detection.id for this traffic light equals std::to_string(reg_elem->id).
     * - The traffic light state is stored in detection.results[hypothesis_index].hypothesis.class_id.
     *
     * Output:
     * - out_traffic_light_state: string (e.g., "red", "yellow", "green", or raw class_id)
     */
    class GetTrafficLightStateAction : public BT::SyncActionNode
    {
    public:
        GetTrafficLightStateAction(const std::string &name, const BT::NodeConfig &config)
            : BT::SyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<lanelet_msgs::msg::RegulatoryElement::SharedPtr>("traffic_light"),
                BT::InputPort<std::shared_ptr<const DynamicObjectStore::Snapshot>>("dynamic_objects_snapshot"),
                BT::InputPort<int>("state_hypothesis_index"),
                BT::OutputPort<std::string>("out_traffic_light_state"),
                BT::OutputPort<std::string>("error_message"),
            };
        }

        BT::NodeStatus tick() override
        {
            auto reg_elem = ports::tryGetPtr<lanelet_msgs::msg::RegulatoryElement>(*this, "traffic_light");
            auto snap = ports::tryGetPtr<const DynamicObjectStore::Snapshot>(*this, "dynamic_objects_snapshot");
            auto hypothesis_index = ports::tryGet<int>(*this, "state_hypothesis_index");

            if (!reg_elem)
            {
                setOutput("error_message", "missing_port:traffic_light");
                std::cout << "[GetTrafficLightState] Missing traffic_light" << std::endl;
                return BT::NodeStatus::FAILURE;
            }

            if (!snap || !snap->objects_snapshot_)
            {
                setOutput("error_message", "missing_port:dynamic_objects_snapshot");
                std::cout << "[GetTrafficLightState] Missing dynamic_objects_snapshot" << std::endl;
                return BT::NodeStatus::FAILURE;
            }

            if (!hypothesis_index)
            {
                setOutput("error_message", "missing_port:state_hypothesis_index");
                std::cout << "[GetTrafficLightState] Missing state_hypothesis_index" << std::endl;
                return BT::NodeStatus::FAILURE;
            }

            const std::string tl_object_id = std::to_string(reg_elem->id);

            const auto *tl = snap->getTrafficLightById(tl_object_id);
            if (!tl)
            {
                setOutput("error_message", "traffic_light_not_found_in_snapshot");
                std::cout << "[GetTrafficLightState] traffic light object not found for id=" << tl_object_id << std::endl;
                return BT::NodeStatus::FAILURE;
            }

            const int idx = *hypothesis_index;
            if (idx < 0 || static_cast<size_t>(idx) >= tl->detection.results.size())
            {
                setOutput("error_message", "hypothesis_index_out_of_range");
                std::cout << "[GetTrafficLightState] hypothesis_index out of range: " << idx << std::endl;
                return BT::NodeStatus::FAILURE;
            }

            const auto &class_id = tl->detection.results[idx].hypothesis.class_id;

            // Return the raw class_id (e.g., "red"/"yellow"/"green"/"traffic_light"/custom)
            setOutput("out_traffic_light_state", class_id);
            std::cout << "[GetTrafficLightState] state=" << class_id << " id=" << tl_object_id << std::endl;

            return BT::NodeStatus::SUCCESS;
        }
    };

} // namespace behaviour

#endif // BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_TRAFFIC_LIGHT_STATE_ACTION_HPP_

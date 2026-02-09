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

#ifndef BEHAVIOUR__NODES__COMMON__ACTIONS__EXECUTE_BEHAVIOUR_PUBLISHER_HPP_
#define BEHAVIOUR__NODES__COMMON__ACTIONS__EXECUTE_BEHAVIOUR_PUBLISHER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <behaviortree_ros2/bt_topic_pub_node.hpp>

#include "behaviour/utils/ports.hpp"
#include "behaviour_msgs/msg/execute_behaviour.hpp"  // Assuming msg equivalent exists

namespace behaviour
{
/**
 * @class ExecuteBehaviourPublisher
 * @brief RosTopicPubNode to publish ExecuteBehaviour messages.
 */
class ExecuteBehaviourPublisher : public BT::RosTopicPubNode<behaviour_msgs::msg::ExecuteBehaviour>
{
public:
  ExecuteBehaviourPublisher(const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
  : BT::RosTopicPubNode<behaviour_msgs::msg::ExecuteBehaviour>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {BT::InputPort<std::string>("behaviour", "The behaviour to execute"),
       BT::InputPort<std::vector<int64_t>>("preferred_lanelet_ids")});
  }

  /**
     * @brief Callback to fill the message.
     * Returning true sends the message; returning false skips publishing.
     */
  bool setMessage(behaviour_msgs::msg::ExecuteBehaviour & msg) override
  {
    auto behaviour = ports::tryGet<std::string>(*this, "behaviour");
    auto preferred_lanelet_ids = ports::tryGet<std::vector<int64_t>>(*this, "preferred_lanelet_ids");
    if (!behaviour) {
      RCLCPP_WARN(node_->get_logger(), "[%s] No behaviour provided on port", name().c_str());
      return false;
    }

    if (!preferred_lanelet_ids) {
      RCLCPP_WARN(node_->get_logger(), "[%s] No preferred lanelet IDs provided on port", name().c_str());
      ;
    }

    msg.behaviour = behaviour.value();
    msg.preferred_lanelet_ids =
      preferred_lanelet_ids.has_value() ? preferred_lanelet_ids.value() : std::vector<int64_t>{};

    RCLCPP_INFO(node_->get_logger(), "[%s] Publishing behaviour: %s", name().c_str(), msg.behaviour.c_str());
    RCLCPP_INFO(
      node_->get_logger(), "[%s] Preferred lanelet IDs count: %zu", name().c_str(), msg.preferred_lanelet_ids.size());
    return true;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__COMMON__ACTIONS__EXECUTE_BEHAVIOUR_PUBLISHER_HPP_

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

#ifndef BEHAVIOUR__NODES__COMMON__ACTIONS__SPEED_BEHAVIOUR_PUBLISHER_HPP_
#define BEHAVIOUR__NODES__COMMON__ACTIONS__SPEED_BEHAVIOUR_PUBLISHER_HPP_

#include <string>

#include <behaviortree_ros2/bt_topic_pub_node.hpp>

#include "behaviour/utils/ports.hpp"
#include "behaviour_msgs/msg/speed_behaviour.hpp"

namespace behaviour
{
/**
 * @class SpeedBehaviourPublisher
 * @brief RosTopicPubNode to publish intersection speed-behaviour context.
 */
class SpeedBehaviourPublisher : public BT::RosTopicPubNode<behaviour_msgs::msg::SpeedBehaviour>
{
public:
  SpeedBehaviourPublisher(const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
  : BT::RosTopicPubNode<behaviour_msgs::msg::SpeedBehaviour>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("speed_behaviour", "Speed behaviour advisory"),
      BT::InputPort<double>("dist_to_stop", "Distance from ego to the upcoming stop/intersection"),
    });
  }

  bool setMessage(behaviour_msgs::msg::SpeedBehaviour & msg) override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      RCLCPP_WARN(node_->get_logger(), "[%s] Missing input port: %s", name().c_str(), port_name);
    };

    auto speed_behaviour = ports::tryGet<std::string>(*this, "speed_behaviour");
    auto dist_to_stop = ports::tryGet<double>(*this, "dist_to_stop");
    if (!ports::require(speed_behaviour, "speed_behaviour", missing_input_callback)) {
      return false;
    }
    if (!ports::require(dist_to_stop, "dist_to_stop", missing_input_callback)) {
      return false;
    }

    msg.speed_behaviour = *speed_behaviour;
    msg.dist_to_stop = *dist_to_stop;
    return true;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__COMMON__ACTIONS__SPEED_BEHAVIOUR_PUBLISHER_HPP_

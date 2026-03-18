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

#ifndef BEHAVIOUR__NODES__COMMON__ACTIONS__GET_WORLD_OBJECTS_SUBSCRIBER_HPP_
#define BEHAVIOUR__NODES__COMMON__ACTIONS__GET_WORLD_OBJECTS_SUBSCRIBER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <behaviortree_ros2/bt_topic_sub_node.hpp>

#include "world_model_msgs/msg/world_object.hpp"
#include "world_model_msgs/msg/world_object_array.hpp"

namespace behaviour
{
/**
 * @class GetWorldObjectsSubscriber
 * @brief RosTopicSubNode that provides the latest world objects snapshot on each tick.
 */
class GetWorldObjectsSubscriber : public BT::RosTopicSubNode<world_model_msgs::msg::WorldObjectArray>
{
public:
  GetWorldObjectsSubscriber(const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
  : BT::RosTopicSubNode<world_model_msgs::msg::WorldObjectArray>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::OutputPort<std::vector<world_model_msgs::msg::WorldObject>>("objects"),
      BT::OutputPort<std::string>("error_message"),
    });
  }

  BT::NodeStatus onTick(const std::shared_ptr<world_model_msgs::msg::WorldObjectArray> & last_msg) override
  {
    if (!last_msg) {
      setOutput("error_message", "no_message_received");
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(logger(), "[%s] Received %zu objects", name().c_str(), last_msg->objects.size());
    setOutput("objects", last_msg->objects);
    return BT::NodeStatus::SUCCESS;
  }

  bool latchLastMessage() const override
  {
    return true;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__COMMON__ACTIONS__GET_WORLD_OBJECTS_SUBSCRIBER_HPP_

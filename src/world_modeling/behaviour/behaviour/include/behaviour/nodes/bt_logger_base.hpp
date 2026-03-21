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

#ifndef BEHAVIOUR__NODES__BT_LOGGER_BASE_HPP_
#define BEHAVIOUR__NODES__BT_LOGGER_BASE_HPP_

#include <stdexcept>
#include <string>
#include <utility>

#include <behaviortree_ros2/ros_node_params.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace behaviour
{

class BTLoggerBase
{
protected:
  explicit BTLoggerBase(rclcpp::Logger logger)
  : logger_(std::move(logger))
  {}

  const rclcpp::Logger & logger() const
  {
    return logger_;
  }

private:
  rclcpp::Logger logger_;
};

inline rclcpp::Logger makeBTChildLogger(const BT::RosNodeParams & params, const std::string & child_name)
{
  auto node = params.nh.lock();
  if (!node) {
    throw std::runtime_error("ROS node expired while creating BT logger for " + child_name);
  }
  return node->get_logger().get_child(child_name);
}

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__BT_LOGGER_BASE_HPP_

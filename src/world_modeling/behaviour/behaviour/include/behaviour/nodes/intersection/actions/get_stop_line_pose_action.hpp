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

#ifndef BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_STOP_LINE_POSE_ACTION_HPP_
#define BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_STOP_LINE_POSE_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <cstddef>
#include <iostream>
#include <memory>
#include <string>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "behaviour/utils/ports.hpp"
#include "behaviour/utils/utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lanelet_msgs/msg/ref_line.hpp"
#include "lanelet_msgs/msg/regulatory_element.hpp"

namespace behaviour
{
  /**
   * @class GetStopLinePoseAction
   * @brief SyncActionNode to build a stop-line pose from a regulatory element.
   */
  class GetStopLinePoseAction : public BT::SyncActionNode
  {
  public:
    GetStopLinePoseAction(const std::string &name, const BT::NodeConfig &conf)
        : BT::SyncActionNode(name, conf)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<lanelet_msgs::msg::RegulatoryElement::SharedPtr>("reg_elem"),
          BT::InputPort<std::string>("map_frame"),
          BT::OutputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("pose"),
          BT::OutputPort<std::string>("error_message"),
      };
    }

    BT::NodeStatus tick() override
    {
      const auto missing_input_callback = [&](const char *port_name)
      {
        std::cout << "[GetStopLinePose] Missing " << port_name << " input" << std::endl;
      };

      auto reg_elem = ports::tryGetPtr<lanelet_msgs::msg::RegulatoryElement>(*this, "reg_elem");
      if (!ports::require(reg_elem, "reg_elem", missing_input_callback))
      {
        return BT::NodeStatus::FAILURE;
      }

      if (reg_elem->ref_lines.empty())
      {
        std::cout << "[GetStopLinePose] RegulatoryElement has no ref_lines" << std::endl;
        setOutput("error_message", "invalid_port");
        return BT::NodeStatus::FAILURE;
      }

      auto map_frame = ports::tryGet<std::string>(*this, "map_frame");
      if (!ports::require(map_frame, "map_frame", missing_input_callback))
      {
        return BT::NodeStatus::FAILURE;
      }

      // todo clarify on the which ref line is the right one
      const auto &ref = reg_elem->ref_lines.front();
      const auto &pts = ref.points;

      if (pts.size() < 2)
      {
        std::cout << "[GetStopLinePose] RefLine has fewer than 2 points" << std::endl;
        setOutput("error_message", "invalid_port");
        return BT::NodeStatus::FAILURE;
      }

      const std::size_t mid = pts.size() / 2;
      const auto &pm = pts[mid];

      const std::size_t i0 = (mid == 0) ? 0 : (mid - 1);
      const std::size_t i1 = (mid + 1 >= pts.size()) ? (pts.size() - 1) : (mid + 1);

      const double dx = static_cast<double>(pts[i1].x) - static_cast<double>(pts[i0].x);
      const double dy = static_cast<double>(pts[i1].y) - static_cast<double>(pts[i0].y);
      const double yaw = std::atan2(dy, dx);

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);

      auto pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
      pose->header.stamp = rclcpp::Time(0);

      pose->header.frame_id = *map_frame;

      pose->pose.position.x = pm.x;
      pose->pose.position.y = pm.y;
      pose->pose.position.z = pm.z;
      pose->pose.orientation = tf2::toMsg(q);

      std::cout << "[GetStopLinePose] Extracted stop line pose at (" << pose->pose.position.x << ", "
                << pose->pose.position.y << ", " << pose->pose.position.z << ") with yaw " << yaw << std::endl;

      setOutput("pose", pose);
      return BT::NodeStatus::SUCCESS;
    }
  };

} // namespace behaviour

#endif // BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_STOP_LINE_POSE_ACTION_HPP_

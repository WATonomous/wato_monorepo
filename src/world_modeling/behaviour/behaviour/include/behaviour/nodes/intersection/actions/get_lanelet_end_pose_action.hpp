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

#ifndef BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_LANELET_END_POSE_ACTION_HPP_
#define BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_LANELET_END_POSE_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include "behaviour/nodes/bt_logger_base.hpp"
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <cstddef>
#include <memory>
#include <string>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "behaviour/utils/utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lanelet_msgs/msg/current_lane_context.hpp"
#include "rclcpp/time.hpp"

namespace behaviour
{
class GetLaneletEndPoseAction : public BT::SyncActionNode, protected BTLoggerBase
{
public:
  GetLaneletEndPoseAction(const std::string & name, const BT::NodeConfig & conf, const rclcpp::Logger & logger)
  : BT::SyncActionNode(name, conf)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<lanelet_msgs::msg::CurrentLaneContext::SharedPtr>("lane_ctx"),
      BT::InputPort<std::string>("map_frame"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("pose"),
      BT::OutputPort<std::string>("error_message"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      RCLCPP_DEBUG_STREAM(logger(), "Missing " << port_name << " input" );
    };

    auto lane_ctx = ports::tryGetPtr<lanelet_msgs::msg::CurrentLaneContext>(*this, "lane_ctx");
    if (!ports::require(lane_ctx, "lane_ctx", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto map_frame = ports::tryGet<std::string>(*this, "map_frame");
    if (!ports::require(map_frame, "map_frame", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    const auto & lanelet = lane_ctx->current_lanelet;
    if (lanelet.centerline.size() < 2) {
      RCLCPP_DEBUG_STREAM(logger(), "Current lanelet centerline has fewer than 2 points" );
      setOutput("error_message", "invalid_lanelet_centerline");
      return BT::NodeStatus::FAILURE;
    }

    const std::size_t mid = lanelet.centerline.size() / 2;
    const std::size_t i0 = (mid == 0) ? 0 : (mid - 1);
    const std::size_t i1 = (mid + 1 >= lanelet.centerline.size()) ? (lanelet.centerline.size() - 1) : (mid + 1);

    const auto & mid_point = lanelet.centerline[mid];
    const auto & p0 = lanelet.centerline[i0];
    const auto & p1 = lanelet.centerline[i1];
    const double dx = static_cast<double>(p1.x) - static_cast<double>(p0.x);
    const double dy = static_cast<double>(p1.y) - static_cast<double>(p0.y);
    const double yaw = std::atan2(dy, dx);

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);

    auto pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
    pose->header.stamp = rclcpp::Time(0);
    pose->header.frame_id = *map_frame;
    pose->pose.position = mid_point;
    pose->pose.orientation = tf2::toMsg(q);

    RCLCPP_DEBUG_STREAM(
      logger(), "Extracted lanelet midpoint pose at (" << pose->pose.position.x << ", "
                << pose->pose.position.y << ", " << pose->pose.position.z << ") with yaw " << yaw );

    setOutput("pose", pose);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_LANELET_END_POSE_ACTION_HPP_

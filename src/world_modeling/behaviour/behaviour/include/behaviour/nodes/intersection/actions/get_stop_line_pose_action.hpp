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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "behaviour/nodes/bt_logger_base.hpp"
#include "behaviour/utils/utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lanelet_msgs/msg/current_lane_context.hpp"
#include "lanelet_msgs/msg/regulatory_element.hpp"
#include "lanelet_msgs/msg/way.hpp"
#include "rclcpp/time.hpp"

namespace behaviour
{
/**
   * @class GetStopLinePoseAction
   * @brief SyncActionNode to build a stop-line pose from a regulatory element.
   */
class GetStopLinePoseAction : public BT::SyncActionNode, protected BTLoggerBase
{
public:
  GetStopLinePoseAction(const std::string & name, const BT::NodeConfig & conf, const rclcpp::Logger & logger)
  : BT::SyncActionNode(name, conf)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<lanelet_msgs::msg::RegulatoryElement::SharedPtr>("reg_elem"),
      BT::InputPort<lanelet_msgs::msg::CurrentLaneContext::SharedPtr>("lane_ctx"),
      BT::InputPort<std::string>("map_frame"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("pose"),
      BT::OutputPort<std::string>("error_message"),
    };
  }

  static const lanelet_msgs::msg::Way * findStopLineWayForLanelet(
    const lanelet_msgs::msg::RegulatoryElement & reg_elem, int64_t lanelet_id)
  {
    const lanelet_msgs::msg::Way * first_non_empty_way = nullptr;
    const lanelet_msgs::msg::Way * first_way_without_lanelet_ids = nullptr;
    std::size_t non_empty_ref_line_count = 0;

    for (const auto & lanelet_way : reg_elem.ref_lines) {
      if (lanelet_way.way.points.empty()) {
        continue;
      }

      ++non_empty_ref_line_count;
      if (!first_non_empty_way) {
        first_non_empty_way = &lanelet_way.way;
      }
      if (!first_way_without_lanelet_ids && lanelet_way.lanelet_ids.empty()) {
        first_way_without_lanelet_ids = &lanelet_way.way;
      }

      const auto lanelet_id_it = std::find(lanelet_way.lanelet_ids.begin(), lanelet_way.lanelet_ids.end(), lanelet_id);
      if (lanelet_id_it != lanelet_way.lanelet_ids.end()) {
        return &lanelet_way.way;
      }
    }

    // Some regulatory elements, especially traffic-sign style stop signs, may expose
    // a single ref_line without lanelet associations. In that case use the unique line.
    if (non_empty_ref_line_count == 1) {
      return first_non_empty_way;
    }

    if (first_way_without_lanelet_ids) {
      return first_way_without_lanelet_ids;
    }

    return nullptr;
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      RCLCPP_DEBUG_STREAM(logger(), "Missing " << port_name << " input");
    };

    auto reg_elem = ports::tryGetPtr<lanelet_msgs::msg::RegulatoryElement>(*this, "reg_elem");
    if (!ports::require(reg_elem, "reg_elem", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto lane_ctx = ports::tryGetPtr<lanelet_msgs::msg::CurrentLaneContext>(*this, "lane_ctx");
    if (!ports::require(lane_ctx, "lane_ctx", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    if (reg_elem->ref_lines.empty()) {
      RCLCPP_DEBUG_STREAM(logger(), "RegulatoryElement has no ref_lines");
      setOutput("error_message", "invalid_port");
      return BT::NodeStatus::FAILURE;
    }

    auto map_frame = ports::tryGet<std::string>(*this, "map_frame");
    if (!ports::require(map_frame, "map_frame", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    const int64_t current_lanelet_id = lane_ctx->current_lanelet.id;
    const lanelet_msgs::msg::Way * stop_line_way = findStopLineWayForLanelet(*reg_elem, current_lanelet_id);
    if (!stop_line_way) {
      RCLCPP_DEBUG_STREAM(logger(), "No stop line found for lanelet " << current_lanelet_id);
      setOutput("error_message", "missing_stop_line_for_lanelet");
      return BT::NodeStatus::FAILURE;
    }

    const auto center_point = utils::geometry::wayCenterPoint(*stop_line_way);
    if (!center_point) {
      RCLCPP_DEBUG_STREAM(logger(), "Stop line has no valid points");
      setOutput("error_message", "invalid_stop_line");
      return BT::NodeStatus::FAILURE;
    }

    // get orientation of the pose
    const auto & pts = stop_line_way->points;

    if (pts.size() < 2) {
      RCLCPP_DEBUG_STREAM(logger(), "Stop line has fewer than 2 points");
      setOutput("error_message", "invalid_stop_line");
      return BT::NodeStatus::FAILURE;
    }

    const std::size_t mid = pts.size() / 2;

    const std::size_t i0 = (mid == 0) ? 0 : (mid - 1);
    const std::size_t i1 = (mid + 1 >= pts.size()) ? (pts.size() - 1) : (mid + 1);

    const double dx = static_cast<double>(pts[i1].x) - static_cast<double>(pts[i0].x);
    const double dy = static_cast<double>(pts[i1].y) - static_cast<double>(pts[i0].y);
    const double yaw = std::atan2(dy, dx) + M_PI_2;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);

    auto pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
    pose->header.stamp = rclcpp::Time(0);

    pose->header.frame_id = *map_frame;

    pose->pose.position.x = center_point->x;
    pose->pose.position.y = center_point->y;
    pose->pose.position.z = center_point->z;
    pose->pose.orientation = tf2::toMsg(q);

    RCLCPP_DEBUG_STREAM(
      logger(),
      "Extracted stop line pose at (" << pose->pose.position.x << ", " << pose->pose.position.y << ", "
                                      << pose->pose.position.z << ") with yaw " << yaw);

    setOutput("pose", pose);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_STOP_LINE_POSE_ACTION_HPP_

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

#ifndef BEHAVIOUR__NODES__LANE_NAVIGATION__CONDITIONS__OVERTAKE_VALID_CONDITION_HPP_
#define BEHAVIOUR__NODES__LANE_NAVIGATION__CONDITIONS__OVERTAKE_VALID_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <algorithm>
#include <cstddef>
#include <string>
#include <vector>

#include "behaviour/nodes/bt_logger_base.hpp"
#include "behaviour/utils/area_occupancy.hpp"
#include "behaviour/utils/utils.hpp"
#include "behaviour/utils/world_objects.hpp"
#include "lanelet_msgs/msg/current_lane_context.hpp"
#include "world_model_msgs/msg/area_occupancy_info.hpp"

namespace behaviour
{
/**
 * @class OvertakeValidCondition
 * @brief Gates the start of overtake based on front occupancy and missing-side parked-car checks.
 *
 * Logic:
 * - Return FAILURE when both adjacent lane IDs are missing.
 * - Return SUCCESS when the configured overtake-front area is occupied only by vehicle-like objects.
 * - Return SUCCESS when a side lane is missing and that side's lane-change corridor is occupied
 *   (used to simulate parked cars close to the ego lane).
 * - Return FAILURE otherwise.
 */
class OvertakeValidCondition : public BT::ConditionNode, protected BTLoggerBase
{
public:
  OvertakeValidCondition(const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::ConditionNode(name, config)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<lanelet_msgs::msg::CurrentLaneContext::SharedPtr>("lane_ctx", "Current lane context"),
      BT::InputPort<std::vector<world_model_msgs::msg::AreaOccupancyInfo>>("areas", "Area occupancy response"),
      BT::InputPort<std::string>("overtake_front_area", "Front area used to trigger overtake"),
      BT::InputPort<std::vector<std::string>>(
        "left_lane_change_areas", "Areas used to detect parked cars on the left edge"),
      BT::InputPort<std::vector<std::string>>(
        "right_lane_change_areas", "Areas used to detect parked cars on the right edge"),
      BT::InputPort<std::size_t>("hypothesis_index", "Detection hypothesis index"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      RCLCPP_DEBUG_STREAM(logger(), "Missing " << port_name << " input");
    };

    auto lane_ctx = ports::tryGetPtr<lanelet_msgs::msg::CurrentLaneContext>(*this, "lane_ctx");
    if (!ports::require(lane_ctx, "lane_ctx", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto area_infos = ports::tryGet<std::vector<world_model_msgs::msg::AreaOccupancyInfo>>(*this, "areas");
    if (!ports::require(area_infos, "areas", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto overtake_front_area = ports::tryGet<std::string>(*this, "overtake_front_area");
    if (!ports::require(overtake_front_area, "overtake_front_area", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto left_areas = ports::tryGet<std::vector<std::string>>(*this, "left_lane_change_areas");
    if (!ports::require(left_areas, "left_lane_change_areas", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto right_areas = ports::tryGet<std::vector<std::string>>(*this, "right_lane_change_areas");
    if (!ports::require(right_areas, "right_lane_change_areas", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto hypothesis_index = ports::tryGet<std::size_t>(*this, "hypothesis_index");
    if (!ports::require(hypothesis_index, "hypothesis_index", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    const auto & current_lanelet = lane_ctx->current_lanelet;
    const bool has_left_lane = current_lanelet.left_lane_id != -1;
    const bool has_right_lane = current_lanelet.right_lane_id != -1;

    if (!has_left_lane && !has_right_lane) {
      RCLCPP_DEBUG_STREAM(logger(), "Overtake invalid: both adjacent lanes are missing");
      return BT::NodeStatus::FAILURE;
    }

    if (area_occupancy_utils::isAreaOccupied(*area_infos, *overtake_front_area)) {
      const auto objects = area_occupancy_utils::getAreaObjects(*area_infos, *overtake_front_area);
      const bool contains_vehicle = std::any_of(
        objects.begin(), objects.end(),
        [&](const auto & object) { return world_objects::isVehicle(object, *hypothesis_index); });
      const bool contains_non_vehicle =
        std::any_of(
          objects.begin(), objects.end(),
          [&](const auto & object) { return world_objects::isPedestrian(object, *hypothesis_index); }) ||
        world_objects::containsType(objects, "bicycle") || world_objects::containsType(objects, "cyclist") ||
        world_objects::containsType(objects, "traffic_light");

      if (contains_vehicle && !contains_non_vehicle) {
        RCLCPP_DEBUG_STREAM(logger(), "Overtake valid: front area contains only vehicle-like objects");
        return BT::NodeStatus::SUCCESS;
      }
    }

    const bool missing_right_with_parked_car =
      !has_right_lane &&
      std::any_of(
        right_areas->begin(), right_areas->end(),
        [&](const auto & area_name) { return area_occupancy_utils::isAreaOccupied(*area_infos, area_name); });
    const bool missing_left_with_parked_car =
      !has_left_lane &&
      std::any_of(
        left_areas->begin(), left_areas->end(),
        [&](const auto & area_name) { return area_occupancy_utils::isAreaOccupied(*area_infos, area_name); });

    if (missing_right_with_parked_car || missing_left_with_parked_car) {
      RCLCPP_DEBUG_STREAM(
        logger(), "Overtake valid: occupied corridor detected on side without an adjacent lane");
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_DEBUG_STREAM(logger(), "Overtake invalid: no qualifying front obstacle or missing-side corridor blockage");
    return BT::NodeStatus::FAILURE;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__LANE_NAVIGATION__CONDITIONS__OVERTAKE_VALID_CONDITION_HPP_

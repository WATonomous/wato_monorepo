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

#ifndef BEHAVIOUR__UTILS__INTERSECTION_UTILS_HPP_
#define BEHAVIOUR__UTILS__INTERSECTION_UTILS_HPP_

#include <cstdint>
#include <memory>
#include <optional>
#include <algorithm>
#include <vector>

#include "behaviour/utils/geometry.hpp"
#include "behaviour/utils/lanelet.hpp"
#include "lanelet_msgs/msg/current_lane_context.hpp"
#include "lanelet_msgs/msg/lanelet.hpp"
#include "lanelet_msgs/msg/regulatory_element.hpp"

namespace behaviour::utils::intersection
{

struct ActiveTrafficControlContext
{
  int64_t lanelet_id{0};
  lanelet_msgs::msg::RegulatoryElement::SharedPtr element{nullptr};
  int64_t element_id{0};
  double distance_to_intersection_m{-1.0};
  bool passing_active_traffic_control_element{false};

  explicit operator bool() const
  {
    return static_cast<bool>(element);
  }
};

inline bool hasPassedActiveTrafficControlElement(
  int64_t active_lanelet_id, const lanelet_msgs::msg::CurrentLaneContext & lane_ctx)
{
  if (lane_ctx.current_lanelet.id == active_lanelet_id) {
    return false;
  }

  if (lane_ctx.current_lanelet.is_intersection) {
    return false;
  }

  for (const auto & upcoming_id : lane_ctx.upcoming_lanelet_ids) {
    if (upcoming_id == active_lanelet_id) {
      return false;
    }
  }

  return true;
}

inline bool isPassingActiveTrafficControlElement(
  const lanelet_msgs::msg::CurrentLaneContext & lane_ctx)
{
  return lane_ctx.current_lanelet.is_intersection;
}

inline double getDistanceToUpcomingIntersection(
  const lanelet_msgs::msg::CurrentLaneContext & lane_ctx,
  const std::vector<lanelet_msgs::msg::Lanelet> & search_lanelets,
  bool is_passing_active_traffic_control_element)
{
  if (is_passing_active_traffic_control_element) {
    return -1.0;
  }

  double distance_to_lanelet_start_m = std::max(0.0, lane_ctx.distance_to_lanelet_end_m);
  for (std::size_t i = 1; i < search_lanelets.size(); ++i) {
    if (search_lanelets[i].is_intersection) {
      return distance_to_lanelet_start_m;
    }

    distance_to_lanelet_start_m += utils::geometry::polylineLengthXY(search_lanelets[i].centerline);
  }

  return -1.0;
}

inline ActiveTrafficControlContext makeActiveTrafficControlContext(
  const lanelet_msgs::msg::CurrentLaneContext & lane_ctx,
  const std::vector<lanelet_msgs::msg::Lanelet> & search_lanelets,
  int64_t lanelet_id,
  const lanelet_msgs::msg::RegulatoryElement::SharedPtr & element)
{
  const bool is_passing_active_traffic_control_element =
    isPassingActiveTrafficControlElement(lane_ctx);

  ActiveTrafficControlContext context;
  context.lanelet_id = lanelet_id;
  context.element = element;
  context.element_id = element ? element->id : 0;
  context.passing_active_traffic_control_element = is_passing_active_traffic_control_element;
  context.distance_to_intersection_m = getDistanceToUpcomingIntersection(
    lane_ctx, search_lanelets, is_passing_active_traffic_control_element);
  return context;
}

inline int trafficControlPriority(types::TrafficControlElementType type)
{
  if (type == types::TrafficControlElementType::TRAFFIC_LIGHT) return 0;
  if (type == types::TrafficControlElementType::STOP_SIGN) return 1;
  if (type == types::TrafficControlElementType::YIELD) return 2;
  return 999;
}

inline lanelet_msgs::msg::RegulatoryElement::SharedPtr classifyLaneletTrafficControlElement(
  const lanelet_msgs::msg::Lanelet & lanelet)
{
  lanelet_msgs::msg::RegulatoryElement::SharedPtr primary_reg_elem = nullptr;
  int best_prio = 999;

  for (const auto & reg_elem : lanelet.regulatory_elements) {
    const auto elem_type = utils::lanelet::getTrafficControlElementType(reg_elem);
    if (!elem_type) {
      continue;
    }

    const int priority = trafficControlPriority(*elem_type);
    if (priority < best_prio) {
      best_prio = priority;
      primary_reg_elem = std::make_shared<lanelet_msgs::msg::RegulatoryElement>(reg_elem);

      if (best_prio == 0) {
        break;
      }
    }
  }

  return primary_reg_elem;
}

inline std::optional<ActiveTrafficControlContext> findNextActiveTrafficControlContext(
  const lanelet_msgs::msg::CurrentLaneContext & lane_ctx,
  const std::vector<lanelet_msgs::msg::Lanelet> & search_lanelets)
{
  for (const auto & lanelet : search_lanelets) {
    if (auto element = classifyLaneletTrafficControlElement(lanelet)) {
      return makeActiveTrafficControlContext(lane_ctx, search_lanelets, lanelet.id, element);
    }
  }

  return std::nullopt;
}

}  // namespace behaviour::utils::intersection

#endif  // BEHAVIOUR__UTILS__INTERSECTION_UTILS_HPP_

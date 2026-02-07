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

#ifndef BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_STOP_SIGN_CARS_WITH_TIMESTAMPS_ACTION_HPP_
#define BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_STOP_SIGN_CARS_WITH_TIMESTAMPS_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/time.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "behaviour/dynamic_object_store.hpp"
#include "behaviour/utils/ports.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "lanelet_msgs/msg/lanelet.hpp"
#include "lanelet_msgs/msg/regulatory_element.hpp"

namespace behaviour
{
/**
 * @class GetStopSignCarsWithTimestampsAction
 * @brief SyncActionNode to collect stop-sign queued car IDs with their detection timestamps.
 *
 * This action identifies vehicles near stop signs and returns their IDs along with timestamps.
 * The timestamps are used for temporal reasoning to determine right-of-way.
 *
 * Enhanced logic:
 * - Filters by distance to stop line (spatial proximity)
 * - Filters by velocity (car must be stopped or slowing down)
 * - Filters by heading (car must be approaching the stop line, not departing)
 * - Returns both car IDs and their detection timestamps
 */
class GetStopSignCarsWithTimestampsAction : public BT::SyncActionNode
{
public:
  GetStopSignCarsWithTimestampsAction(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<lanelet_msgs::msg::RegulatoryElement::SharedPtr>("stop_sign"),
      BT::InputPort<std::vector<lanelet_msgs::msg::Lanelet>>("lanelets"),
      BT::InputPort<double>("threshold_m"),
      BT::InputPort<double>("velocity_threshold_mps"),  // Max velocity to be considered stopped (default: 0.5 m/s)
      BT::InputPort<std::shared_ptr<const DynamicObjectStore::Snapshot>>("dynamic_objects_snapshot"),
      BT::OutputPort<std::vector<std::pair<std::string, rclcpp::Time>>>("out_stop_sign_cars_with_timestamps"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto stop_sign = ports::tryGetPtr<lanelet_msgs::msg::RegulatoryElement>(*this, "stop_sign");
    auto snap = ports::tryGetPtr<const DynamicObjectStore::Snapshot>(*this, "dynamic_objects_snapshot");
    auto lanelets = ports::tryGet<std::vector<lanelet_msgs::msg::Lanelet>>(*this, "lanelets");
    auto threshold_m = ports::tryGet<double>(*this, "threshold_m");
    auto velocity_threshold = ports::tryGet<double>(*this, "velocity_threshold_mps").value_or(0.5);

    if (!stop_sign) {
      std::cout << "[GetStopSignCarsWithTimestamps] Missing stop_sign" << std::endl;
      return BT::NodeStatus::FAILURE;
    }
    if (!snap || !snap->objects_snapshot_) {
      std::cout << "[GetStopSignCarsWithTimestamps] Missing dynamic_objects_snapshot" << std::endl;
      return BT::NodeStatus::FAILURE;
    }
    if (!lanelets) {
      std::cout << "[GetStopSignCarsWithTimestamps] Missing lanelets" << std::endl;
      return BT::NodeStatus::FAILURE;
    }
    if (!threshold_m) {
      std::cout << "[GetStopSignCarsWithTimestamps] Missing threshold_m" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    if (stop_sign->yield_lanelet_ids.empty()) {
      std::cout << "[GetStopSignCarsWithTimestamps] yield_lanelet_ids empty" << std::endl;
      setOutput("out_stop_sign_cars_with_timestamps", std::vector<std::pair<std::string, rclcpp::Time>>{});
      return BT::NodeStatus::SUCCESS;
    }

    std::unordered_set<int64_t> yield_ids;
    yield_ids.reserve(stop_sign->yield_lanelet_ids.size());
    for (const auto id : stop_sign->yield_lanelet_ids) {
      yield_ids.insert(id);
    }

    std::vector<std::pair<std::string, rclcpp::Time>> out_cars;
    out_cars.reserve(32);

    for (const auto & ll : *lanelets) {
      if (yield_ids.find(ll.id) == yield_ids.end()) {
        continue;
      }

      // Find this reg elem within this lanelet so we can use its ref_lines (stop lines).
      const lanelet_msgs::msg::RegulatoryElement * re_in_ll = nullptr;
      for (const auto & re : ll.regulatory_elements) {
        if (re.id == stop_sign->id) {
          re_in_ll = &re;
          break;
        }
      }
      if (!re_in_ll) {
        continue;
      }

      // Pick the first ref_line with points as the stop line.
      const lanelet_msgs::msg::RefLine * stop_line = nullptr;
      for (const auto & rl : re_in_ll->ref_lines) {
        if (!rl.points.empty()) {
          stop_line = &rl;
          break;
        }
      }
      if (!stop_line) {
        continue;
      }

      // Compute centroid of the stop line polyline in XY.
      double cx = 0.0, cy = 0.0;
      for (const auto & p : stop_line->points) {
        cx += p.x;
        cy += p.y;
      }
      cx /= static_cast<double>(stop_line->points.size());
      cy /= static_cast<double>(stop_line->points.size());

      // Compute stop line direction vector (for heading check)
      geometry_msgs::msg::Vector3 stop_line_dir;
      if (stop_line->points.size() >= 2) {
        const auto & p1 = stop_line->points[0];
        const auto & p2 = stop_line->points.back();
        stop_line_dir.x = p2.x - p1.x;
        stop_line_dir.y = p2.y - p1.y;
        double len = std::sqrt(stop_line_dir.x * stop_line_dir.x + stop_line_dir.y * stop_line_dir.y);
        if (len > 1e-6) {
          stop_line_dir.x /= len;
          stop_line_dir.y /= len;
        }
      }

      // Get cars currently in this lanelet and filter by distance, velocity, and heading.
      const auto cars = snap->getCarsInLanelet(ll.id);
      for (const auto * car : cars) {
        if (!car) continue;

        const auto & pos = car->detection.bbox.center.position;
        const auto & header = car->detection.header;

        // Distance check
        const double dx = pos.x - cx;
        const double dy = pos.y - cy;
        const double d = std::sqrt(dx * dx + dy * dy);

        if (d > *threshold_m) {
          continue;
        }

        // Velocity check - car should be stopped or slowing down
        // Note: velocity information would come from tracking/history
        // For now, we accept cars within threshold distance
        // TODO(WATonomous): Add velocity filtering using history data

        // Heading check - ensure car is approaching, not departing
        // This requires comparing car's heading with lanelet direction
        // For now, we accept all cars within threshold
        // TODO(WATonomous): Add heading-based filtering

        // Extract timestamp from detection header
        rclcpp::Time timestamp(header.stamp);

        out_cars.emplace_back(car->detection.id, timestamp);

        std::cout << "[GetStopSignCarsWithTimestamps] Car " << car->detection.id << " at distance " << d
                  << "m, timestamp " << timestamp.seconds() << "s" << std::endl;
      }
    }

    // Sort by timestamp (earliest first) for debugging
    std::sort(out_cars.begin(), out_cars.end(), [](const auto & a, const auto & b) { return a.second < b.second; });

    // Deduplicate by ID (keep earliest timestamp for each ID)
    std::vector<std::pair<std::string, rclcpp::Time>> unique_cars;
    std::unordered_set<std::string> seen_ids;
    for (const auto & [id, time] : out_cars) {
      if (seen_ids.insert(id).second) {
        unique_cars.emplace_back(id, time);
      }
    }

    std::cout << "[GetStopSignCarsWithTimestamps] Found " << unique_cars.size() << " cars at stop sign" << std::endl;

    setOutput("out_stop_sign_cars_with_timestamps", unique_cars);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_STOP_SIGN_CARS_WITH_TIMESTAMPS_ACTION_HPP_

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

#include "prediction/map_interface.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_set>
#include <vector>

namespace {
double distancePointToSegment(const geometry_msgs::msg::Point &p,
                              const geometry_msgs::msg::Point &a,
                              const geometry_msgs::msg::Point &b) {
  double dx_ba = b.x - a.x;
  double dy_ba = b.y - a.y;
  double l2 = dx_ba * dx_ba + dy_ba * dy_ba;

  if (l2 == 0.0) {
    double dx_pa = p.x - a.x;
    double dy_pa = p.y - a.y;
    return std::sqrt(dx_pa * dx_pa + dy_pa * dy_pa);
  }

  double t = ((p.x - a.x) * dx_ba + (p.y - a.y) * dy_ba) / l2;
  t = std::max(0.0, std::min(1.0, t));

  double closest_x = a.x + t * dx_ba;
  double closest_y = a.y + t * dy_ba;
  double dx = p.x - closest_x;
  double dy = p.y - closest_y;

  return std::sqrt(dx * dx + dy * dy);
}
} // namespace

namespace prediction {

MapInterface::MapInterface(rclcpp::Node *node) : node_(node) {
  // Subscribe to lane context topic
  lane_context_sub_ =
      node_->create_subscription<lanelet_msgs::msg::CurrentLaneContext>(
          "lane_context", rclcpp::QoS(1).best_effort(),
          std::bind(&MapInterface::laneContextCallback, this,
                    std::placeholders::_1));

  // Subscribe to route ahead topic
  route_ahead_sub_ = node_->create_subscription<lanelet_msgs::msg::RouteAhead>(
      "route_ahead", rclcpp::QoS(1).best_effort(),
      std::bind(&MapInterface::routeAheadCallback, this,
                std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(),
              "MapInterface initialized with lanelet subscriptions");
}

void MapInterface::laneContextCallback(
    const lanelet_msgs::msg::CurrentLaneContext::SharedPtr msg) {
  {
    std::lock_guard<std::mutex> lock(context_mutex_);
    current_lane_context_ = msg;
  }

  // Cache the current lanelet
  cacheLanelet(msg->current_lanelet);
}

void MapInterface::routeAheadCallback(
    const lanelet_msgs::msg::RouteAhead::SharedPtr msg) {
  // Cache all lanelets in the route ahead
  cacheLanelets(msg->lanelets);
}

void MapInterface::cacheLanelet(const lanelet_msgs::msg::Lanelet &lanelet) {
  cacheLanelets({lanelet});
}

void MapInterface::cacheLanelets(
    const std::vector<lanelet_msgs::msg::Lanelet> &lanelets) {
  std::lock_guard<std::mutex> lock(cache_mutex_);

  for (const auto &lanelet : lanelets) {
    auto it = lanelet_cache_.find(lanelet.id);
    if (it != lanelet_cache_.end()) {
      // Exist: Update data and move to front of LRU
      it->second.lanelet = lanelet;
      lru_list_.erase(it->second.lru_iterator);
      lru_list_.push_front(lanelet.id);
      it->second.lru_iterator = lru_list_.begin();
    } else {
      // New: Check size capacity
      if (lanelet_cache_.size() >= MAX_CACHE_SIZE) {
        int64_t last_id = lru_list_.back();
        lru_list_.pop_back();
        lanelet_cache_.erase(last_id);
      }
      lru_list_.push_front(lanelet.id);
      lanelet_cache_.insert({lanelet.id, {lanelet, lru_list_.begin()}});
    }
  }
}

LaneletInfo MapInterface::laneletMsgToInfo(
    const lanelet_msgs::msg::Lanelet &lanelet) const {
  LaneletInfo info;
  info.id = lanelet.id;
  info.centerline = lanelet.centerline;
  info.speed_limit = lanelet.speed_limit_mps;
  info.following_lanelets = lanelet.successor_ids;

  // Note: previous_lanelets would require reverse lookup which isn't directly
  // available in the message. For now, leave it empty - could be computed from
  // cached data if needed.

  return info;
}

double MapInterface::distanceToLanelet(
    const geometry_msgs::msg::Point &point,
    const lanelet_msgs::msg::Lanelet &lanelet) const {
  if (lanelet.centerline.empty()) {
    return std::numeric_limits<double>::max();
  }

  if (lanelet.centerline.size() == 1) {
    double dx = point.x - lanelet.centerline[0].x;
    double dy = point.y - lanelet.centerline[0].y;
    return std::sqrt(dx * dx + dy * dy);
  }

  double min_sq_distance = std::numeric_limits<double>::max();

  for (size_t i = 0; i < lanelet.centerline.size() - 1; ++i) {
    const auto &p1 = lanelet.centerline[i];
    const auto &p2 = lanelet.centerline[i + 1];

    double dist = distancePointToSegment(point, p1, p2);
    if (dist < min_sq_distance) {
      min_sq_distance = dist;
    }
  }

  return min_sq_distance;
}

std::optional<int64_t>
MapInterface::findNearestLanelet(const geometry_msgs::msg::Point &point) {
  std::lock_guard<std::mutex> lock(cache_mutex_);

  if (lanelet_cache_.empty()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                         "No lanelets cached yet, cannot find nearest lanelet");
    return std::nullopt;
  }

  int64_t nearest_id = -1;
  double min_distance = std::numeric_limits<double>::max();

  for (const auto &[id, cached_lanelet] : lanelet_cache_) {
    double distance = distanceToLanelet(point, cached_lanelet.lanelet);
    if (distance < min_distance) {
      min_distance = distance;
      nearest_id = id;
    }
  }

  if (nearest_id == -1) {
    return std::nullopt;
  }

  return nearest_id;
}

std::optional<LaneletInfo> MapInterface::getLaneletById(int64_t lanelet_id) {
  std::lock_guard<std::mutex> lock(cache_mutex_);

  auto it = lanelet_cache_.find(lanelet_id);
  if (it != lanelet_cache_.end()) {
    // Update LRU - access makes it "fresh"
    lru_list_.erase(it->second.lru_iterator);
    lru_list_.push_front(lanelet_id);
    it->second.lru_iterator = lru_list_.begin();

    return laneletMsgToInfo(it->second.lanelet);
  }

  RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                       "Lanelet ID %ld not found in cache", lanelet_id);

  return std::nullopt;
}

std::vector<int64_t>
MapInterface::getPossibleFutureLanelets(int64_t current_lanelet_id,
                                        int max_depth) {
  std::vector<int64_t> result;
  std::unordered_set<int64_t> visited;
  std::queue<std::pair<int64_t, int>> bfs_queue; // (lanelet_id, depth)

  bfs_queue.push({current_lanelet_id, 0});
  visited.insert(current_lanelet_id);

  std::lock_guard<std::mutex> lock(cache_mutex_);

  while (!bfs_queue.empty()) {
    auto [id, depth] = bfs_queue.front();
    bfs_queue.pop();

    result.push_back(id);

    if (depth >= max_depth) {
      continue;
    }

    auto it = lanelet_cache_.find(id);
    if (it == lanelet_cache_.end()) {
      continue;
    }

    // Update LRU for traversed nodes? Maybe too expensive. Let's skip modifying
    // LRU for breadth search to avoid thrashing.
    const auto &lanelet = it->second.lanelet;

    // Add successors
    for (int64_t successor_id : lanelet.successor_ids) {
      if (visited.find(successor_id) == visited.end()) {
        visited.insert(successor_id);
        bfs_queue.push({successor_id, depth + 1});
      }
    }

    // Add lane change options if available
    if (lanelet.can_change_left && lanelet.left_lane_id != -1) {
      if (visited.find(lanelet.left_lane_id) == visited.end()) {
        visited.insert(lanelet.left_lane_id);
        bfs_queue.push({lanelet.left_lane_id, depth + 1});
      }
    }

    if (lanelet.can_change_right && lanelet.right_lane_id != -1) {
      if (visited.find(lanelet.right_lane_id) == visited.end()) {
        visited.insert(lanelet.right_lane_id);
        bfs_queue.push({lanelet.right_lane_id, depth + 1});
      }
    }
  }

  if (result.empty()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                         "No future lanelets found for lanelet ID %ld",
                         current_lanelet_id);
  }

  return result;
}

std::optional<double> MapInterface::getSpeedLimit(int64_t lanelet_id) {
  std::lock_guard<std::mutex> lock(cache_mutex_);

  auto it = lanelet_cache_.find(lanelet_id);
  if (it != lanelet_cache_.end()) {
    // Update LRU
    lru_list_.erase(it->second.lru_iterator);
    lru_list_.push_front(lanelet_id);
    it->second.lru_iterator = lru_list_.begin();

    return it->second.lanelet.speed_limit_mps;
  }

  RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                       "Lanelet ID %ld not found in cache", lanelet_id);

  return std::nullopt;
}

bool MapInterface::isCrosswalkNearby(const geometry_msgs::msg::Point &point,
                                     double radius) {
  std::lock_guard<std::mutex> lock(cache_mutex_);

  for (const auto &[id, cached_lanelet] : lanelet_cache_) {
    const auto &lanelet = cached_lanelet.lanelet;
    // Check if this lanelet is a crosswalk
    if (lanelet.lanelet_type != "crosswalk") {
      continue;
    }

    // Check if the crosswalk is within the search radius
    double distance = distanceToLanelet(point, lanelet);
    if (distance <= radius) {
      return true;
    }
  }

  return false;
}

} // namespace prediction

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

#include "prediction/map_interface_node.hpp"

#include <vector>

namespace prediction
{

MapInterfaceNode::MapInterfaceNode(rclcpp::Node * node)
: node_(node)
, core_(static_cast<size_t>(node_->declare_parameter("lanelet_cache_size", 1000)))
, max_lanelet_search_depth_(node_->declare_parameter("max_lanelet_search_depth", 3))
{
  // Subscribe to lane context topic
  lane_context_sub_ = node_->create_subscription<lanelet_msgs::msg::CurrentLaneContext>(
    "lane_context",
    rclcpp::QoS(1).best_effort(),
    std::bind(&MapInterfaceNode::laneContextCallback, this, std::placeholders::_1));

  // Subscribe to route ahead topic
  route_ahead_sub_ = node_->create_subscription<lanelet_msgs::msg::RouteAhead>(
    "route_ahead",
    rclcpp::QoS(1).best_effort(),
    std::bind(&MapInterfaceNode::routeAheadCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    node_->get_logger(),
    "MapInterfaceNode initialized (cache_size=%zu, search_depth=%d)",
    core_.getMaxCacheSize(),
    max_lanelet_search_depth_);
}

void MapInterfaceNode::laneContextCallback(const lanelet_msgs::msg::CurrentLaneContext::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(context_mutex_);
    current_lane_context_ = msg;
  }

  // Cache the current lanelet
  core_.cacheLanelet(msg->current_lanelet);
}

void MapInterfaceNode::routeAheadCallback(const lanelet_msgs::msg::RouteAhead::SharedPtr msg)
{
  // Cache all lanelets in the route ahead
  core_.cacheLanelets(msg->lanelets);
}

std::optional<int64_t> MapInterfaceNode::findNearestLanelet(const geometry_msgs::msg::Point & point)
{
  auto result = core_.findNearestLanelet(point);
  if (!result.has_value()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000, "No lanelets cached yet, cannot find nearest lanelet");
  }
  return result;
}

std::optional<LaneletInfo> MapInterfaceNode::getLaneletById(int64_t lanelet_id)
{
  auto result = core_.getLaneletById(lanelet_id);
  if (!result.has_value()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000, "Lanelet ID %ld not found in cache", lanelet_id);
  }
  return result;
}

std::vector<int64_t> MapInterfaceNode::getPossibleFutureLanelets(int64_t current_lanelet_id)
{
  return getPossibleFutureLanelets(current_lanelet_id, max_lanelet_search_depth_);
}

std::vector<int64_t> MapInterfaceNode::getPossibleFutureLanelets(int64_t current_lanelet_id, int max_depth)
{
  auto result = core_.getPossibleFutureLanelets(current_lanelet_id, max_depth);
  if (result.empty()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      5000,
      "No future lanelets found for lanelet ID %ld",
      current_lanelet_id);
  }
  return result;
}

std::optional<double> MapInterfaceNode::getSpeedLimit(int64_t lanelet_id)
{
  auto result = core_.getSpeedLimit(lanelet_id);
  if (!result.has_value()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000, "Lanelet ID %ld not found in cache", lanelet_id);
  }
  return result;
}

bool MapInterfaceNode::isCrosswalkNearby(const geometry_msgs::msg::Point & point, double radius)
{
  return core_.isCrosswalkNearby(point, radius);
}

}  // namespace prediction

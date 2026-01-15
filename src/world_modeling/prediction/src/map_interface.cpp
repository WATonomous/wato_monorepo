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

namespace prediction
{

MapInterface::MapInterface(rclcpp::Node* node)
: node_(node)
{
  RCLCPP_INFO(node_->get_logger(), "MapInterface initialized (using placeholders until map services are available)");
}

int64_t MapInterface::findNearestLanelet(const geometry_msgs::msg::Point & point)
{
  // PLACEHOLDER: Return mock lanelet ID until map services are available
  RCLCPP_DEBUG_ONCE(node_->get_logger(), 
                    "Using placeholder lanelet ID (map services not available)");
  
  // Return a deterministic lanelet ID based on grid position
  int64_t grid_x = static_cast<int64_t>(std::floor(point.x / 10.0));
  int64_t grid_y = static_cast<int64_t>(std::floor(point.y / 10.0));
  return 1000 + grid_x * 100 + grid_y;
}

LaneletInfo MapInterface::getLaneletById(int64_t lanelet_id)
{
  // PLACEHOLDER: Return mock lanelet info with straight centerline
  LaneletInfo info;
  info.id = lanelet_id;

  // Generate a simple straight centerline based on lanelet ID
  for (int i = 0; i < 10; ++i) {
    geometry_msgs::msg::Point point;
    point.x = (lanelet_id % 100) * 10.0 + i * 5.0;
    point.y = ((lanelet_id / 100) % 100) * 10.0;
    point.z = 0.0;
    info.centerline.push_back(point);
  }

  info.speed_limit = 13.4;  // ~30 mph default
  
  // Mock following lanelets (straight continuation)
  info.following_lanelets.push_back(lanelet_id + 1);
  
  // Mock previous lanelets
  if (lanelet_id > 1000) {
    info.previous_lanelets.push_back(lanelet_id - 1);
  }

  RCLCPP_DEBUG_ONCE(node_->get_logger(), 
                    "Using placeholder lanelet info (map services not available)");
  return info;
}

std::vector<int64_t> MapInterface::getPossibleFutureLanelets(
  int64_t current_lanelet_id,
  int max_depth)
{
  // PLACEHOLDER: Return simple linear sequence until map graph is available
  std::vector<int64_t> possible_lanelets;

  // Current lanelet
  possible_lanelets.push_back(current_lanelet_id);

  // Generate forward lanelets (straight continuation)
  for (int i = 1; i <= max_depth; ++i) {
    possible_lanelets.push_back(current_lanelet_id + i);
  }

  // Add lane change options at depth 1
  if (max_depth >= 1) {
    possible_lanelets.push_back(current_lanelet_id + 100);  // Left lane
    possible_lanelets.push_back(current_lanelet_id - 100);  // Right lane
  }

  RCLCPP_DEBUG_ONCE(node_->get_logger(), 
                    "Using placeholder future lanelets (map services not available)");
  
  return possible_lanelets;
}

double MapInterface::getSpeedLimit(int64_t lanelet_id)
{
  // PLACEHOLDER: Return default speed limit
  RCLCPP_DEBUG_ONCE(node_->get_logger(), 
                    "Using placeholder speed limit (map services not available)");
  
  // Vary speed limit slightly based on lanelet ID for testing
  double base_speed = 13.4;  // ~30 mph
  double variation = (lanelet_id % 3) * 2.2;  // 0, 2.2, or 4.4 m/s variation
  return base_speed + variation;
}

bool MapInterface::isCrosswalkNearby(
  const geometry_msgs::msg::Point & point,
  double radius)
{
  // PLACEHOLDER: Simple grid-based crosswalk detection
  RCLCPP_DEBUG_ONCE(node_->get_logger(), 
                    "Using placeholder crosswalk detection (map services not available)");
  
  // Simulate crosswalks at intersection-like grid positions
  int grid_x = static_cast<int>(std::floor(point.x / 20.0));
  int grid_y = static_cast<int>(std::floor(point.y / 20.0));
  
  // Crosswalk every 100m at grid intersections
  return (grid_x % 5 == 0 && grid_y % 5 == 0);
}

}  // namespace prediction

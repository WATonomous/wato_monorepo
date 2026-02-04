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

#include "costmap/layers/virtual_wall_layer.hpp"

#include <cmath>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace costmap
{

void VirtualWallLayer::configure(
  rclcpp_lifecycle::LifecycleNode * node,
  const std::string & layer_name,
  tf2_ros::Buffer * tf_buffer)
{
  node_ = node;
  layer_name_ = layer_name;
  tf_buffer_ = tf_buffer;
}

void VirtualWallLayer::activate()
{
  spawn_srv_ = node_->create_service<costmap_msgs::srv::SpawnWall>(
    "spawn_wall",
    std::bind(
      &VirtualWallLayer::spawnWallCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  despawn_srv_ = node_->create_service<costmap_msgs::srv::DespawnWall>(
    "despawn_wall",
    std::bind(
      &VirtualWallLayer::despawnWallCallback, this,
      std::placeholders::_1, std::placeholders::_2));
}

void VirtualWallLayer::deactivate()
{
  spawn_srv_.reset();
  despawn_srv_.reset();
}

void VirtualWallLayer::cleanup()
{
  spawn_srv_.reset();
  despawn_srv_.reset();
  std::lock_guard<std::mutex> lock(walls_mutex_);
  walls_.clear();
}

void VirtualWallLayer::spawnWallCallback(
  const costmap_msgs::srv::SpawnWall::Request::SharedPtr request,
  costmap_msgs::srv::SpawnWall::Response::SharedPtr response)
{
  std::lock_guard<std::mutex> lock(walls_mutex_);

  Wall wall;
  wall.pose = request->pose;
  wall.length = request->length;
  wall.width = request->width;

  int64_t id = next_wall_id_++;
  walls_[id] = wall;

  response->success = true;
  response->wall_id = id;
}

void VirtualWallLayer::despawnWallCallback(
  const costmap_msgs::srv::DespawnWall::Request::SharedPtr request,
  costmap_msgs::srv::DespawnWall::Response::SharedPtr response)
{
  std::lock_guard<std::mutex> lock(walls_mutex_);

  auto it = walls_.find(request->wall_id);
  if (it != walls_.end()) {
    walls_.erase(it);
    response->success = true;
  } else {
    response->success = false;
    response->error_message = "Wall ID " + std::to_string(request->wall_id) + " not found";
  }
}

void VirtualWallLayer::markBox(
  nav_msgs::msg::OccupancyGrid & grid,
  double cx, double cy, double yaw,
  double half_x, double half_y,
  int8_t cost) const
{
  const auto & info = grid.info;
  const double ox = info.origin.position.x;
  const double oy = info.origin.position.y;
  const double res = info.resolution;
  const int w = static_cast<int>(info.width);
  const int h = static_cast<int>(info.height);

  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  double corners_local[4][2] = {
    {-half_x, -half_y},
    { half_x, -half_y},
    { half_x,  half_y},
    {-half_x,  half_y}
  };

  double min_gx = 1e9, max_gx = -1e9, min_gy = 1e9, max_gy = -1e9;
  for (auto & c : corners_local) {
    double gx = cx + cos_yaw * c[0] - sin_yaw * c[1];
    double gy = cy + sin_yaw * c[0] + cos_yaw * c[1];
    min_gx = std::min(min_gx, gx);
    max_gx = std::max(max_gx, gx);
    min_gy = std::min(min_gy, gy);
    max_gy = std::max(max_gy, gy);
  }

  int min_col = std::max(0, static_cast<int>((min_gx - ox) / res));
  int max_col = std::min(w - 1, static_cast<int>((max_gx - ox) / res));
  int min_row = std::max(0, static_cast<int>((min_gy - oy) / res));
  int max_row = std::min(h - 1, static_cast<int>((max_gy - oy) / res));

  for (int row = min_row; row <= max_row; ++row) {
    for (int col = min_col; col <= max_col; ++col) {
      double wx = ox + (col + 0.5) * res;
      double wy = oy + (row + 0.5) * res;

      double dx = wx - cx;
      double dy = wy - cy;
      double lx = cos_yaw * dx + sin_yaw * dy;
      double ly = -sin_yaw * dx + cos_yaw * dy;

      if (std::abs(lx) <= half_x && std::abs(ly) <= half_y) {
        int idx = row * w + col;
        grid.data[idx] = std::max(grid.data[idx], cost);
      }
    }
  }
}

void VirtualWallLayer::update(
  nav_msgs::msg::OccupancyGrid & grid,
  const geometry_msgs::msg::TransformStamped & map_to_costmap)
{
  std::lock_guard<std::mutex> lock(walls_mutex_);

  const double tx = map_to_costmap.transform.translation.x;
  const double ty = map_to_costmap.transform.translation.y;
  const auto & q = map_to_costmap.transform.rotation;
  const double tf_yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double cos_tf = std::cos(tf_yaw);
  const double sin_tf = std::sin(tf_yaw);

  for (const auto & [id, wall] : walls_) {
    double map_x = wall.pose.position.x;
    double map_y = wall.pose.position.y;
    double map_yaw = std::atan2(
      2.0 * (wall.pose.orientation.w * wall.pose.orientation.z +
             wall.pose.orientation.x * wall.pose.orientation.y),
      1.0 - 2.0 * (wall.pose.orientation.y * wall.pose.orientation.y +
                    wall.pose.orientation.z * wall.pose.orientation.z));

    double cx = cos_tf * map_x + sin_tf * map_y + tx;
    double cy = -sin_tf * map_x + cos_tf * map_y + ty;
    double wall_yaw = map_yaw + tf_yaw;

    markBox(grid, cx, cy, wall_yaw, wall.length / 2.0, wall.width / 2.0, 100);
  }
}

}  // namespace costmap

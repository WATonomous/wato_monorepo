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

#include <algorithm>
#include <cmath>
#include <string>

#include "costmap/costmap_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace costmap
{

void VirtualWallLayer::configure(
  rclcpp_lifecycle::LifecycleNode * node, const std::string & layer_name, tf2_ros::Buffer * tf_buffer)
{
  node_ = node;
  layer_name_ = layer_name;
  tf_buffer_ = tf_buffer;
}

void VirtualWallLayer::activate()
{
  spawn_srv_ = node_->create_service<costmap_msgs::srv::SpawnWall>(
    "spawn_wall", std::bind(&VirtualWallLayer::spawnWallCallback, this, std::placeholders::_1, std::placeholders::_2));

  despawn_srv_ = node_->create_service<costmap_msgs::srv::DespawnWall>(
    "despawn_wall",
    std::bind(&VirtualWallLayer::despawnWallCallback, this, std::placeholders::_1, std::placeholders::_2));
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
  wall.inflation_radius = request->inflation_radius;

  // If position_frame is specified, it means the input pose is defined in that frame,
  // but we want to store it relative to wall.pose.header.frame_id (the target frame).
  if (!request->position_frame.empty()) {
    // Check if we need to transform
    if (request->position_frame != wall.pose.header.frame_id) {
      if (!tf_buffer_) {
        response->success = false;
        response->error_message = "TF buffer not available";
        return;
      }

      try {
        // We want to transform from position_frame to wall.pose.header.frame_id
        // The input pose coordinates are assumed to be in position_frame
        geometry_msgs::msg::PoseStamped input_pose;
        input_pose.header.frame_id = request->position_frame;
        input_pose.header.stamp = rclcpp::Time(0);  // Use latest available transform
        input_pose.pose = request->pose.pose;

        geometry_msgs::msg::PoseStamped transformed_pose;
        // Transform into the target frame (request->pose.header.frame_id)
        auto tf = tf_buffer_->lookupTransform(wall.pose.header.frame_id, request->position_frame, tf2::TimePointZero);

        tf2::doTransform(input_pose, transformed_pose, tf);

        // Update the wall pose with the transformed result
        // The header.frame_id remains the target frame
        wall.pose = transformed_pose;
      } catch (const tf2::TransformException & ex) {
        response->success = false;
        response->error_message = std::string("Failed to transform from position_frame: ") + ex.what();
        return;
      }
    }
  }

  // If position_frame is specified, it means the input pose is defined in that frame,
  // but we want to store it relative to wall.pose.header.frame_id (the target frame).
  if (!request->position_frame.empty()) {
    // Check if we need to transform
    if (request->position_frame != wall.pose.header.frame_id) {
      if (!tf_buffer_) {
        response->success = false;
        response->error_message = "TF buffer not available";
        return;
      }

      try {
        // We want to transform from position_frame to wall.pose.header.frame_id
        // The input pose coordinates are assumed to be in position_frame
        geometry_msgs::msg::PoseStamped input_pose;
        input_pose.header.frame_id = request->position_frame;
        input_pose.header.stamp = rclcpp::Time(0);  // Use latest available transform
        input_pose.pose = request->pose.pose;

        geometry_msgs::msg::PoseStamped transformed_pose;
        // Transform into the target frame (request->pose.header.frame_id)
        auto tf = tf_buffer_->lookupTransform(wall.pose.header.frame_id, request->position_frame, tf2::TimePointZero);

        tf2::doTransform(input_pose, transformed_pose, tf);

        // Update the wall pose with the transformed result
        // The header.frame_id remains the target frame
        wall.pose = transformed_pose;
      } catch (const tf2::TransformException & ex) {
        response->success = false;
        response->error_message = std::string("Failed to transform from position_frame: ") + ex.what();
        return;
      }
    }
  }

  int32_t id = next_wall_id_++;
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

void VirtualWallLayer::update(
  nav_msgs::msg::OccupancyGrid & grid, const geometry_msgs::msg::TransformStamped & /*map_to_costmap*/)
{
  std::lock_guard<std::mutex> lock(walls_mutex_);

  for (const auto & [id, wall] : walls_) {
    // Transform the wall pose from its source frame into the costmap frame
    geometry_msgs::msg::PoseStamped pose_out;
    try {
      auto tf = tf_buffer_->lookupTransform(grid.header.frame_id, wall.pose.header.frame_id, tf2::TimePointZero);
      tf2::doTransform(wall.pose, pose_out, tf);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        2000,
        "VirtualWallLayer TF lookup failed for wall %d: %s",
        id,
        ex.what());
      continue;
    }

    double cx = pose_out.pose.position.x;
    double cy = pose_out.pose.position.y;
    double wall_yaw = yawFromQuat(pose_out.pose.orientation);

    markBox(grid, cx, cy, wall_yaw, wall.length / 2.0, wall.width / 2.0, 100);

    if (wall.inflation_radius > 0.0) {
      const auto & info = grid.info;
      const double ox = info.origin.position.x;
      const double oy = info.origin.position.y;
      const double res = info.resolution;
      const int w = static_cast<int>(info.width);
      const int h = static_cast<int>(info.height);
      const double half_x = wall.length / 2.0;
      const double half_y = wall.width / 2.0;
      const double cos_yaw = std::cos(wall_yaw);
      const double sin_yaw = std::sin(wall_yaw);

      // Compute bounding box of the wall in grid coordinates (mirrors markBox logic)
      double corners_local[4][2] = {{-half_x, -half_y}, {half_x, -half_y}, {half_x, half_y}, {-half_x, half_y}};
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

      // Inflate only the wall's own cells, not all cost-100 cells in the grid
      for (int row = min_row; row <= max_row; ++row) {
        for (int col = min_col; col <= max_col; ++col) {
          double wx = ox + (col + 0.5) * res;
          double wy = oy + (row + 0.5) * res;
          double dx = wx - cx;
          double dy = wy - cy;
          double lx = cos_yaw * dx + sin_yaw * dy;
          double ly = -sin_yaw * dx + cos_yaw * dy;
          if (std::abs(lx) <= half_x && std::abs(ly) <= half_y) {
            inflateCell(grid, row, col, wall.inflation_radius, 0.5);
          }
        }
      }
    }
  }
}

}  // namespace costmap

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(costmap::VirtualWallLayer, costmap::CostmapLayer)

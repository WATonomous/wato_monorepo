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

#include "costmap/layers/objects_layer.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace costmap
{

void ObjectsLayer::configure(
  rclcpp_lifecycle::LifecycleNode * node, const std::string & layer_name, tf2_ros::Buffer * tf_buffer)
{
  node_ = node;
  layer_name_ = layer_name;
  tf_buffer_ = tf_buffer;

  node_->declare_parameter("layers." + layer_name_ + ".bbox_inflation_m", 0.5);
  node_->declare_parameter("layers." + layer_name_ + ".bbox_cost_decay", 1.0);
  node_->declare_parameter("layers." + layer_name_ + ".prediction_inflation_m", 0.3);
  node_->declare_parameter("layers." + layer_name_ + ".prediction_cost_decay", 0.8);

  bbox_inflation_m_ = node_->get_parameter("layers." + layer_name_ + ".bbox_inflation_m").as_double();
  bbox_cost_decay_ = node_->get_parameter("layers." + layer_name_ + ".bbox_cost_decay").as_double();
  prediction_inflation_m_ = node_->get_parameter("layers." + layer_name_ + ".prediction_inflation_m").as_double();
  prediction_cost_decay_ = node_->get_parameter("layers." + layer_name_ + ".prediction_cost_decay").as_double();
}

void ObjectsLayer::activate()
{
  objects_sub_ = node_->create_subscription<world_model_msgs::msg::WorldObjectArray>(
    "world_objects", rclcpp::QoS(10), std::bind(&ObjectsLayer::objectsCallback, this, std::placeholders::_1));
}

void ObjectsLayer::deactivate()
{
  objects_sub_.reset();
}

void ObjectsLayer::cleanup()
{
  objects_sub_.reset();
  latest_objects_.reset();
}

void ObjectsLayer::objectsCallback(const world_model_msgs::msg::WorldObjectArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_objects_ = msg;
}

void ObjectsLayer::markBox(
  nav_msgs::msg::OccupancyGrid & grid, double cx, double cy, double yaw, double half_x, double half_y, int8_t cost)
  const
{
  const auto & info = grid.info;
  const double ox = info.origin.position.x;
  const double oy = info.origin.position.y;
  const double res = info.resolution;
  const int w = static_cast<int>(info.width);
  const int h = static_cast<int>(info.height);

  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  // Corners in local frame
  double corners_local[4][2] = {{-half_x, -half_y}, {half_x, -half_y}, {half_x, half_y}, {-half_x, half_y}};

  // Transform corners to grid frame and find bounding box
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

      // Transform to box-local frame
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

static double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

void ObjectsLayer::update(
  nav_msgs::msg::OccupancyGrid & grid, const geometry_msgs::msg::TransformStamped & /*map_to_costmap*/)
{
  world_model_msgs::msg::WorldObjectArray::SharedPtr objects;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    objects = latest_objects_;
  }

  if (!objects) {
    return;
  }

  // Look up transform from the objects' frame to costmap frame
  geometry_msgs::msg::TransformStamped obj_to_costmap;
  try {
    obj_to_costmap = tf_buffer_->lookupTransform(grid.header.frame_id, objects->header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000, "ObjectsLayer TF lookup failed: %s", ex.what());
    return;
  }

  for (const auto & obj : objects->objects) {
    const auto & det = obj.detection;
    const auto & bbox = det.bbox;

    // Transform bbox center to costmap frame
    geometry_msgs::msg::PoseStamped pose_in, pose_out;
    pose_in.pose = bbox.center;
    tf2::doTransform(pose_in, pose_out, obj_to_costmap);

    double obj_yaw = yawFromQuat(pose_out.pose.orientation);
    double base_half_x = bbox.size.x / 2.0;
    double base_half_y = bbox.size.y / 2.0;

    // Mark inflated region at decayed cost, then original bbox at full cost
    if (bbox_inflation_m_ > 0.0) {
      int8_t inflated_cost = static_cast<int8_t>(std::max(1.0, 100.0 * bbox_cost_decay_));
      markBox(
        grid,
        pose_out.pose.position.x,
        pose_out.pose.position.y,
        obj_yaw,
        base_half_x + bbox_inflation_m_,
        base_half_y + bbox_inflation_m_,
        inflated_cost);
    }

    markBox(grid, pose_out.pose.position.x, pose_out.pose.position.y, obj_yaw, base_half_x, base_half_y, 100);

    // Project prediction poses with decaying cost
    for (const auto & prediction : obj.predictions) {
      double cost_factor = 1.0;
      for (const auto & ps : prediction.poses) {
        cost_factor *= prediction_cost_decay_;
        int8_t pred_cost = static_cast<int8_t>(std::max(1.0, 100.0 * cost_factor));

        geometry_msgs::msg::PoseStamped pred_out;
        tf2::doTransform(ps, pred_out, obj_to_costmap);

        double pred_half = prediction_inflation_m_;
        markBox(
          grid,
          pred_out.pose.position.x,
          pred_out.pose.position.y,
          yawFromQuat(pred_out.pose.orientation),
          pred_half,
          pred_half,
          pred_cost);
      }
    }
  }
}

}  // namespace costmap

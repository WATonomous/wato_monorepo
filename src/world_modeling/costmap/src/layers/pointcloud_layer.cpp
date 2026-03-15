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

#include "costmap/layers/pointcloud_layer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

#include "costmap/costmap_utils.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace costmap
{

void PointCloudLayer::configure(
  rclcpp_lifecycle::LifecycleNode * node, const std::string & layer_name, tf2_ros::Buffer * tf_buffer)
{
  node_ = node;
  layer_name_ = layer_name;
  tf_buffer_ = tf_buffer;

  node_->declare_parameter("layers." + layer_name_ + ".max_height_m", 0.0);
  node_->declare_parameter("layers." + layer_name_ + ".min_height_m", 0.0);
  node_->declare_parameter("layers." + layer_name_ + ".inflation_m", 0.0);
  node_->declare_parameter("layers." + layer_name_ + ".cost_decay", 0.0);
  max_height_m_ = node_->get_parameter("layers." + layer_name_ + ".max_height_m").as_double();
  min_height_m_ = node_->get_parameter("layers." + layer_name_ + ".min_height_m").as_double();
  inflation_m_ = node_->get_parameter("layers." + layer_name_ + ".inflation_m").as_double();
  cost_decay_ = node_->get_parameter("layers." + layer_name_ + ".cost_decay").as_double();
  footprint_front_left_ = node_->get_parameter("footprint_front_left").as_double_array();
  footprint_rear_right_ = node_->get_parameter("footprint_rear_right").as_double_array();
}

void PointCloudLayer::activate()
{
  cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "pointcloud", rclcpp::SensorDataQoS(), std::bind(&PointCloudLayer::cloudCallback, this, std::placeholders::_1));
}

void PointCloudLayer::deactivate()
{
  cloud_sub_.reset();
}

void PointCloudLayer::cleanup()
{
  cloud_sub_.reset();
  latest_cloud_.reset();
}

void PointCloudLayer::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_cloud_ = msg;
}

void PointCloudLayer::update(
  nav_msgs::msg::OccupancyGrid & grid, const geometry_msgs::msg::TransformStamped & /*map_to_costmap*/)
{
  sensor_msgs::msg::PointCloud2::SharedPtr cloud;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    cloud = latest_cloud_;
  }

  if (!cloud || cloud->data.empty()) {
    return;
  }

  // Look up transform from the cloud's frame to costmap frame
  geometry_msgs::msg::TransformStamped cloud_to_costmap;
  try {
    cloud_to_costmap = tf_buffer_->lookupTransform(grid.header.frame_id, cloud->header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000, "PointCloudLayer TF lookup failed: %s", ex.what());
    return;
  }

  // Find x, y, z field offsets in the PointCloud2 message
  int x_offset = -1, y_offset = -1, z_offset = -1;
  for (const auto & field : cloud->fields) {
    if (field.name == "x") {
      x_offset = static_cast<int>(field.offset);
    }
    if (field.name == "y") {
      y_offset = static_cast<int>(field.offset);
    }
    if (field.name == "z") {
      z_offset = static_cast<int>(field.offset);
    }
  }
  if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "PointCloudLayer: cloud missing x/y/z fields");
    return;
  }

  const auto & info = grid.info;
  const double ox = info.origin.position.x;
  const double oy = info.origin.position.y;
  const double res = info.resolution;
  const int w = static_cast<int>(info.width);
  const int h = static_cast<int>(info.height);

  const uint32_t point_step = cloud->point_step;
  const size_t num_points = cloud->width * cloud->height;

  for (size_t i = 0; i < num_points; ++i) {
    const uint8_t * ptr = &cloud->data[i * point_step];

    float px, py, pz;
    std::memcpy(&px, ptr + x_offset, sizeof(float));
    std::memcpy(&py, ptr + y_offset, sizeof(float));
    std::memcpy(&pz, ptr + z_offset, sizeof(float));

    if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
      continue;
    }

    // Transform point to costmap frame
    geometry_msgs::msg::PointStamped pt_in, pt_out;
    pt_in.point.x = static_cast<double>(px);
    pt_in.point.y = static_cast<double>(py);
    pt_in.point.z = static_cast<double>(pz);
    tf2::doTransform(pt_in, pt_out, cloud_to_costmap);

    // Filter by height in costmap frame
    if (max_height_m_ != 0.0 && pt_out.point.z > max_height_m_) {
      continue;
    }
    if (min_height_m_ != 0.0 && pt_out.point.z < min_height_m_) {
      continue;
    }

    // Exclude points inside the robot footprint (front_left / rear_right corners)
    if (
      footprint_front_left_.size() == 2 && footprint_rear_right_.size() == 2 &&
      (footprint_front_left_[0] != footprint_rear_right_[0] || footprint_front_left_[1] != footprint_rear_right_[1]))
    {
      double fl_x = footprint_front_left_[0];
      double fl_y = footprint_front_left_[1];
      double rr_x = footprint_rear_right_[0];
      double rr_y = footprint_rear_right_[1];
      double min_x = std::min(fl_x, rr_x);
      double max_x = std::max(fl_x, rr_x);
      double min_y = std::min(fl_y, rr_y);
      double max_y = std::max(fl_y, rr_y);
      if (pt_out.point.x >= min_x && pt_out.point.x <= max_x && pt_out.point.y >= min_y && pt_out.point.y <= max_y) {
        continue;
      }
    }

    // Compute grid cell
    int col = static_cast<int>((pt_out.point.x - ox) / res);
    int row = static_cast<int>((pt_out.point.y - oy) / res);

    if (col < 0 || col >= w || row < 0 || row >= h) {
      continue;
    }

    // Mark center cell at full cost
    grid.data[row * w + col] = std::max(grid.data[row * w + col], static_cast<int8_t>(100));

    // Inflate surrounding cells with decaying cost
    if (inflation_m_ > 0.0 && cost_decay_ > 0.0) {
      inflateCell(grid, row, col, inflation_m_, cost_decay_);
    }
  }
}

}  // namespace costmap

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(costmap::PointCloudLayer, costmap::CostmapLayer)

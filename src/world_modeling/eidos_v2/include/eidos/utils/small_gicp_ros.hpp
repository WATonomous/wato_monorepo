#pragma once

#include <cmath>
#include <cstring>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <small_gicp/points/point_cloud.hpp>

namespace eidos {

/// @brief Convert a ROS PointCloud2 message to a small_gicp::PointCloud.
/// Extracts XYZ from the message fields, skips NaN/Inf points.
/// No PCL dependency.
inline small_gicp::PointCloud::Ptr fromRosMsg(
    const sensor_msgs::msg::PointCloud2& msg) {
  auto cloud = std::make_shared<small_gicp::PointCloud>();

  // Find x, y, z field offsets
  int x_offset = -1, y_offset = -1, z_offset = -1;
  for (const auto& field : msg.fields) {
    if (field.name == "x") x_offset = static_cast<int>(field.offset);
    else if (field.name == "y") y_offset = static_cast<int>(field.offset);
    else if (field.name == "z") z_offset = static_cast<int>(field.offset);
  }

  if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
    return cloud;  // No XYZ fields found
  }

  const size_t point_step = msg.point_step;
  const size_t num_points = msg.width * msg.height;
  cloud->points.reserve(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    const uint8_t* ptr = msg.data.data() + i * point_step;
    float x, y, z;
    std::memcpy(&x, ptr + x_offset, sizeof(float));
    std::memcpy(&y, ptr + y_offset, sizeof(float));
    std::memcpy(&z, ptr + z_offset, sizeof(float));

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }

    cloud->points.emplace_back(
        static_cast<double>(x), static_cast<double>(y),
        static_cast<double>(z), 1.0);
  }

  return cloud;
}

}  // namespace eidos

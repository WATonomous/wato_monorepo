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

#pragma once

#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include "patchwork/patchworkpp.h"

namespace wato::perception::patchworkpp
{

/**
 * @brief Patchwork++ ground removal core
 *
 * Wraps the Patchwork++ library, converting clouds between ROS 2 and Eigen and
 * tracking the latest ground/non-ground output.
 */
class GroundRemovalCore
{
public:
  /**
   * @brief Build a Patchwork++ core with the given parameters.
   * @param params Patchwork++ algorithm parameters
   */
  explicit GroundRemovalCore(const patchwork::Params & params);

  /**
   * @brief Segment ground/non-ground from an N×3 point cloud.
   * @param cloud Input point cloud as N×3 Eigen matrix
   */
  void process(const Eigen::MatrixX3f & cloud);

  /**
   * @brief Ground points from the last call to process().
   * @return M×3 matrix of ground points
   */
  Eigen::MatrixX3f getGround() const;

  /**
   * @brief Non-ground points from the last call to process().
   * @return K×3 matrix of non-ground points
   */
  Eigen::MatrixX3f getNonground() const;

  /**
   * @brief Processing time (ms) for the most recent call.
   * @return Processing time in milliseconds
   */
  double getTimeTaken() const;

  /**
   * @brief Convert a PointCloud2 (xyz float32) to an Eigen N×3 matrix.
   * @param cloud_msg Input PointCloud2 message
   * @return N×3 Eigen matrix
   * @throws std::runtime_error if x/y/z are missing or not FLOAT32
   */
  static Eigen::MatrixX3f pointCloud2ToEigen(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg);

  /**
   * @brief Convert an Eigen N×3 matrix to a PointCloud2 with the given header.
   * @param points Input points as N×3 Eigen matrix
   * @param header Header to copy to output message
   * @return PointCloud2 message with XYZ float32 fields
   */
  static sensor_msgs::msg::PointCloud2 eigenToPointCloud2(
    const Eigen::MatrixX3f & points, const std_msgs::msg::Header & header);

private:
  /**
   * @brief Find the index of a named PointField.
   * @param fields Vector of PointField structures
   * @param field_name Name to find (e.g., "x", "y", "z")
   * @return Index of the field if found, -1 otherwise
   */
  static int findFieldIndex(const std::vector<sensor_msgs::msg::PointField> & fields, const std::string & field_name);

  std::unique_ptr<patchwork::PatchWorkpp> patchwork_;
};

}  // namespace wato::perception::patchworkpp

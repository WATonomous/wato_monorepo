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

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include "patchwork/patchworkpp.h"

namespace wato::perception::patchworkpp
{

/**
 * @brief Core ground removal processing using Patchwork++ algorithm
 *
 * Wraps the Patchwork++ library to provide ground segmentation functionality.
 * Handles point cloud conversion between ROS 2 and Eigen formats, and manages
 * the underlying Patchwork++ instance.
 */
class GroundRemovalCore
{
public:
  /**
   * @brief Constructs the Patchwork++ ground removal core
   *
   * Allocates and configures an internal PatchWork++ instance using the
   * provided parameters. This sets up the algorithm that segments ground
   * from LiDAR point clouds in subsequent calls to process().
   *
   * @param params Patchwork++ algorithm parameters
   */
  explicit GroundRemovalCore(const patchwork::Params & params);

  /**
   * @brief Runs ground estimation on the input point cloud
   *
   * Expects an N×3 matrix where each row is [x, y, z] in meters.
   * Internally forwards to PatchWork++ estimateGround(), updating the
   * cached ground and non-ground partitions and timing information.
   *
   * @param cloud Input point cloud as N×3 Eigen matrix
   */
  void process(const Eigen::MatrixX3f & cloud);

  /**
   * @brief Returns the ground points from the last processed cloud
   *
   * @return M×3 matrix containing points classified as ground by PatchWork++
   *         during the most recent call to process()
   */
  Eigen::MatrixX3f getGround() const;

  /**
   * @brief Returns the non-ground points from the last processed cloud
   *
   * @return K×3 matrix of points rejected as ground (i.e., obstacles/structures)
   *         by PatchWork++ during the last process()
   */
  Eigen::MatrixX3f getNonground() const;

  /**
   * @brief Returns the processing time for the most recent call to process()
   *
   * @return Processing time in milliseconds
   */
  double getTimeTaken() const;

  /**
   * @brief Converts a ROS 2 PointCloud2 to an Eigen N×3 matrix
   *
   * Locates the x/y/z fields by name, verifies they are FLOAT32, and
   * reads each point into a contiguous Eigen matrix (one row per point).
   * Endianness is handled according to the message flag.
   *
   * @param cloud_msg Input PointCloud2 message
   * @return N×3 Eigen matrix with one row per point
   * @throws std::runtime_error if x/y/z are missing, not FLOAT32, or dimensions are invalid
   */
  static Eigen::MatrixX3f pointCloud2ToEigen(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg);

  /**
   * @brief Converts an Eigen N×3 matrix to a ROS 2 PointCloud2
   *
   * Produces a minimal XYZ-only cloud with FLOAT32 fields at offsets 0, 4, and 8,
   * height=1, width=N, is_bigendian=false, and is_dense=false. The provided header
   * is copied to the output message.
   *
   * @param points Input points as N×3 Eigen matrix
   * @param header Header to copy to output message
   * @return PointCloud2 message with XYZ fields
   */
  static sensor_msgs::msg::PointCloud2 eigenToPointCloud2(
    const Eigen::MatrixX3f & points, const std_msgs::msg::Header & header);

private:
  /**
   * @brief Finds the index of a field in a PointCloud2 message
   *
   * Searches through the fields array to find a field with the specified name.
   * Used to locate x, y, z fields for point cloud conversion.
   *
   * @param fields Vector of PointField structures to search
   * @param field_name Name of the field to find (e.g., "x", "y", "z")
   * @return Index of the field if found, -1 otherwise
   */
  static int findFieldIndex(
    const std::vector<sensor_msgs::msg::PointField> & fields, const std::string & field_name);

  std::unique_ptr<patchwork::PatchWorkpp> patchwork_;
};

}  // namespace wato::perception::patchworkpp

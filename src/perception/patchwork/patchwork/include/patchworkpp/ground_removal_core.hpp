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

class GroundRemovalCore
{
public:
  explicit GroundRemovalCore(const patchwork::Params & params);
  /*
   * Constructs the Patchwork++ ground removal core.
   * Allocates and configures an internal PatchWork++ instance using the
   * provided parameters. This sets up the algorithm that segments ground
   * from LiDAR point clouds in subsequent calls to process().
   */

  void process(const Eigen::MatrixX3f & cloud);
  /*
   * Runs ground estimation on the input point cloud.
   * Expects an N×3 matrix where each row is [x, y, z] in meters.
   * Internally forwards to PatchWork++ estimateGround(), updating the
   * cached ground and non-ground partitions and timing information.
   */
  Eigen::MatrixX3f getGround() const;
  /*
   * Returns the ground points from the last processed cloud.
   * The result is an M×3 matrix containing points classified as ground
   * by PatchWork++ during the most recent call to process().
   */
  Eigen::MatrixX3f getNonground() const;
  /*
   * Returns the non-ground points from the last processed cloud.
   * The result is a K×3 matrix of points rejected as ground (i.e.,
   * obstacles/structures) by PatchWork++ during the last process().
   */
  double getTimeTaken() const;
  /*
   * Returns the processing time, in milliseconds, for the most recent
   * call to process(). This is provided by the underlying PatchWork++
   * implementation for quick performance monitoring.
   */

  static Eigen::MatrixX3f pointCloud2ToEigen(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg);
  /*
   * Converts a ROS 2 PointCloud2 to an Eigen N×3 matrix.
   * Locates the x/y/z fields by name, verifies they are FLOAT32, and
   * reads each point into a contiguous Eigen matrix (one row per point).
   * Endianness is handled according to the message flag. Throws
   * std::runtime_error if x/y/z are missing or not FLOAT32.
   */
  static sensor_msgs::msg::PointCloud2 eigenToPointCloud2(
    const Eigen::MatrixX3f & points, const std_msgs::msg::Header & header);
  /*
   * Converts an Eigen N×3 matrix to a ROS 2 PointCloud2.
   * Produces a minimal XYZ-only cloud with FLOAT32 fields at offsets
   * 0, 4, and 8, height=1, width=N, is_bigendian=false, and is_dense=false.
   * The provided header is copied to the output message.
   */

private:
  std::unique_ptr<patchwork::PatchWorkpp> patchwork_;
};

}  // namespace wato::perception::patchworkpp

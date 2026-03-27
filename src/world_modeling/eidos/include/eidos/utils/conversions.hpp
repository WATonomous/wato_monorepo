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

#include <gtsam/geometry/Pose3.h>
#include <pcl/common/transforms.h>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>

#include "eidos/utils/types.hpp"

namespace eidos
{

/**
 * @brief Convert a gtsam::Pose3 to PoseType for PCL storage.
 * @param pose The gtsam::Pose3 to convert.
 * @param time Timestamp to embed in the PoseType (default 0.0).
 * @return PoseType with position, RPY angles, and timestamp fields populated.
 */
inline PoseType gtsamPose3ToPoseType(const gtsam::Pose3 & pose, double time = 0.0)
{
  PoseType p;
  p.x = static_cast<float>(pose.translation().x());
  p.y = static_cast<float>(pose.translation().y());
  p.z = static_cast<float>(pose.translation().z());
  p.roll = static_cast<float>(pose.rotation().roll());
  p.pitch = static_cast<float>(pose.rotation().pitch());
  p.yaw = static_cast<float>(pose.rotation().yaw());
  p.time = time;
  p.intensity = 0;
  return p;
}

/**
 * @brief Convert a PoseType to gtsam::Pose3.
 * @param p The PoseType (x, y, z, roll, pitch, yaw) to convert.
 * @return Equivalent gtsam::Pose3 with Rot3::RzRyRx rotation.
 */
inline gtsam::Pose3 poseTypeToGtsamPose3(const PoseType & p)
{
  return gtsam::Pose3(
    gtsam::Rot3::RzRyRx(static_cast<double>(p.roll), static_cast<double>(p.pitch), static_cast<double>(p.yaw)),
    gtsam::Point3(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z)));
}

/**
 * @brief Convert a PoseType to an Eigen Affine3f transform.
 * @param p The PoseType (x, y, z, roll, pitch, yaw) to convert.
 * @return Eigen::Affine3f representing the same 6-DOF transform.
 */
inline Eigen::Affine3f poseTypeToAffine3f(const PoseType & p)
{
  return pcl::getTransformation(p.x, p.y, p.z, p.roll, p.pitch, p.yaw);
}

/**
 * @brief Convert a ROS timestamp to seconds as a double.
 * @tparam T Any type convertible to rclcpp::Time (e.g. builtin_interfaces::msg::Time).
 * @param stamp The timestamp to convert.
 * @return Time in seconds (including fractional nanoseconds).
 */
template <typename T>
inline double stamp2Sec(const T & stamp)
{
  return rclcpp::Time(stamp).seconds();
}

}  // namespace eidos

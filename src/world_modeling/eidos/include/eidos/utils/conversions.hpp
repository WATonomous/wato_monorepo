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

/// Convert gtsam::Pose3 to PoseType for PCL storage.
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

/// Convert PoseType to gtsam::Pose3.
inline gtsam::Pose3 poseTypeToGtsamPose3(const PoseType & p)
{
  return gtsam::Pose3(
    gtsam::Rot3::RzRyRx(static_cast<double>(p.roll), static_cast<double>(p.pitch), static_cast<double>(p.yaw)),
    gtsam::Point3(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z)));
}

/// Convert PoseType to Eigen Affine3f transform.
inline Eigen::Affine3f poseTypeToAffine3f(const PoseType & p)
{
  return pcl::getTransformation(p.x, p.y, p.z, p.roll, p.pitch, p.yaw);
}

/// Convert ROS timestamp to seconds.
template <typename T>
inline double stamp2Sec(const T & stamp)
{
  return rclcpp::Time(stamp).seconds();
}

}  // namespace eidos

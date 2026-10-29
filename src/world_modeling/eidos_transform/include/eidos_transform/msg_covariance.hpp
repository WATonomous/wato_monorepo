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

#include <array>
#include <cmath>
#include <Eigen/Core>

namespace eidos_transform
{

/**
 * @brief Convert a ROS 6x6 covariance diagonal to per-DOF EKF noise (std-dev).
 *
 * ROS pose/twist covariances are ordered [x, y, z, rot_x, rot_y, rot_z] (row-major 6x6),
 * while the EKF uses GTSAM order [rot_x, rot_y, rot_z, x, y, z]. Noise values are
 * standard deviations, so the variance diagonal is square-rooted. Entries that are
 * not positive (unset) fall back to the configured noise.
 *
 * @param cov Row-major 6x6 covariance from a nav_msgs/Odometry pose or twist.
 * @param fallback Configured noise in EKF order, used for unset entries.
 * @return Noise std-devs in EKF order.
 */
inline Eigen::Matrix<double, 6, 1> noiseFromMsgCovariance(
  const std::array<double, 36> & cov, const Eigen::Matrix<double, 6, 1> & fallback)
{
  Eigen::Matrix<double, 6, 1> noise;
  for (int i = 0; i < 6; ++i) {
    const int msg_i = (i < 3) ? i + 3 : i - 3;
    const double var = cov[static_cast<size_t>(msg_i * 6 + msg_i)];
    noise(i) = (var > 0.0) ? std::sqrt(var) : fallback(i);
  }
  return noise;
}

}  // namespace eidos_transform

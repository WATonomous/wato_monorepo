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

#include <cmath>
#include <cstddef>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace eidos
{

/**
 * @brief Time a row of an organized spinning-lidar cloud was fired (ROS-free).
 *
 * Rows are firings in time order over one revolution of `period` seconds, and the cloud is stamped
 * at the end of the revolution (as the velodyne driver publishes it). With `period` <= 0 every row
 * is taken at the stamp.
 */
inline double firingTime(double stamp, double period, std::size_t row, std::size_t rows)
{
  if (period <= 0.0 || rows == 0) return stamp;
  return stamp - period + period * (static_cast<double>(row) + 0.5) / static_cast<double>(rows);
}

/**
 * @brief A point seen at time t, moved into the vehicle frame at time tau (ROS-free).
 *
 * The vehicle moves with a constant body twist (linear v, angular w, both in its frame at tau) over
 * dt = t - tau, which lasts well under a revolution here: its pose at t relative to tau is then
 * (Exp(w dt), v dt), and the point (in the vehicle frame at t) maps to Exp(w dt) * p + v dt.
 */
inline Eigen::Vector3d moveToTime(
  const Eigen::Vector3d & p, double dt, const Eigen::Vector3d & v, const Eigen::Vector3d & w)
{
  const Eigen::Vector3d angle = w * dt;
  const double theta = angle.norm();
  const Eigen::Matrix3d r =
    theta < 1e-12 ? Eigen::Matrix3d::Identity() : Eigen::AngleAxisd(theta, angle / theta).toRotationMatrix();
  return r * p + v * dt;
}

}  // namespace eidos

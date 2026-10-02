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

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace eidos::wheel_kinematics
{

/// One wheel odometry sample, expressed at the base frame.
struct Sample
{
  double t = 0.0;  ///< Timestamp (s)
  double vx = 0.0;  ///< Base-frame forward velocity (m/s)
  double vy = 0.0;  ///< Base-frame lateral velocity (m/s), from the axle lever arm
  double wz = 0.0;  ///< Yaw rate (rad/s), from the IMU gyro
  bool zero_speed = false;  ///< True when the wheels reported zero (stopped or below the sensor cutoff)
};

/// Planar pose (x, y, yaw).
struct Pose2
{
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

/// Result of integrating samples over a time window.
struct Segment
{
  Pose2 delta;  ///< Relative planar motion, expressed in the start pose's frame
  double distance = 0.0;  ///< Path length travelled (m)
  double zero_speed_time = 0.0;  ///< Time spent with zero-speed readings (s)
};

/// Speed noise parameters for a single reading.
struct SpeedNoise
{
  double speed_std_moving = 0.05;  ///< Forward speed std-dev when wheels report motion (m/s)
  double zero_speed_std = 0.4;  ///< Forward speed std-dev when wheels report zero (m/s), covers the ABS cutoff
  double lateral_std = 0.05;  ///< Lateral (nonholonomic) std-dev (m/s)
  double vertical_std = 0.05;  ///< Vertical std-dev (m/s)
  double yaw_rate_std = 0.01;  ///< Yaw rate std-dev (rad/s)
  double unobserved_std = 1.0e3;  ///< Std-dev for DOFs this source says nothing about
};

/// Noise parameters for graph BetweenFactors.
struct FactorNoise
{
  double trans_std_per_meter = 0.02;  ///< Translation std-dev growth per metre travelled
  double yaw_std_per_rad = 0.05;  ///< Yaw std-dev growth per radian turned
  double zero_speed_std = 0.4;  ///< Possible creep speed while wheels read zero (m/s)
  double min_std = 0.01;  ///< Floor on constrained std-devs (m or rad)
  double loose_std = 10.0;  ///< Std-dev for z, roll, pitch (m or rad)
};

/**
 * @brief Rear-axle forward speed from left/right wheel joint velocities.
 *
 * CAN wheel speeds are unsigned, so the result is clamped to >= 0 (forward only).
 *
 * @param w_left Left wheel joint velocity (rad/s).
 * @param w_right Right wheel joint velocity (rad/s).
 * @param wheel_radius Effective rolling radius (m).
 * @return Forward speed at the axle centre (m/s).
 */
inline double axleSpeed(double w_left, double w_right, double wheel_radius)
{
  return std::max(0.0, wheel_radius * (w_left + w_right) / 2.0);
}

/**
 * @brief Transfer a velocity from the axle centre to the base frame origin (rigid body).
 *
 * v_base = v_axle - omega x p_axle, where p_axle is the axle position in the base frame.
 * For an axle behind the base origin (p_axle.x < 0), a positive yaw rate gives positive
 * lateral velocity at the base origin.
 *
 * @param v_axle Velocity at the axle centre, in base-frame axes (m/s).
 * @param omega Angular velocity, in base-frame axes (rad/s).
 * @param p_axle Axle position in the base frame (m).
 * @return Velocity at the base frame origin (m/s).
 */
inline Eigen::Vector3d axleToBaseVelocity(
  const Eigen::Vector3d & v_axle, const Eigen::Vector3d & omega, const Eigen::Vector3d & p_axle)
{
  return v_axle - omega.cross(p_axle);
}

/**
 * @brief Twist covariance for one reading, in nav_msgs order [lin x,y,z, ang x,y,z], row-major 6x6.
 *
 * Zero-speed readings get a wide forward-speed variance since the wheels cannot tell
 * "stopped" from "creeping below the sensor cutoff".
 */
inline std::array<double, 36> twistCovariance(bool zero_speed, const SpeedNoise & n)
{
  std::array<double, 36> cov{};
  const double speed_std = zero_speed ? n.zero_speed_std : n.speed_std_moving;
  const std::array<double, 6> stds = {
    speed_std, n.lateral_std, n.vertical_std, n.unobserved_std, n.unobserved_std, n.yaw_rate_std};
  for (size_t i = 0; i < 6; ++i) {
    cov[i * 6 + i] = stds[i] * stds[i];
  }
  return cov;
}

/**
 * @brief Exact planar motion for a constant body-frame twist held for dt.
 * @return Relative pose expressed in the start frame.
 */
inline Pose2 integrateConstantTwist(double vx, double vy, double wz, double dt)
{
  Pose2 d;
  const double a = wz * dt;
  if (std::abs(wz) < 1e-9) {
    d.x = vx * dt;
    d.y = vy * dt;
  } else {
    const double s = std::sin(a);
    const double c = std::cos(a);
    d.x = (s * vx + (c - 1.0) * vy) / wz;
    d.y = ((1.0 - c) * vx + s * vy) / wz;
  }
  d.theta = a;
  return d;
}

/// Compose a relative pose onto a base pose: result = base * delta.
inline Pose2 compose(const Pose2 & base, const Pose2 & delta)
{
  const double c = std::cos(base.theta);
  const double s = std::sin(base.theta);
  return {base.x + c * delta.x - s * delta.y, base.y + s * delta.x + c * delta.y, base.theta + delta.theta};
}

/**
 * @brief Integrate samples over (t0, t1] with a zero-order hold.
 *
 * Each sample's twist is held from the previous sample time (or t0) up to its own
 * timestamp. The last sample in the window is then held until t1.
 *
 * @param samples Time-ordered samples. Samples outside (t0, t1] are ignored.
 * @param t0 Window start (s).
 * @param t1 Window end (s).
 * @param max_dt Gaps longer than this are skipped (stale data).
 */
inline Segment integrateSegment(const std::deque<Sample> & samples, double t0, double t1, double max_dt)
{
  Segment seg;
  double t_prev = t0;
  const Sample * last = nullptr;

  auto step = [&](const Sample & s, double dt) {
    if (dt <= 0.0 || dt > max_dt) return;
    seg.delta = compose(seg.delta, integrateConstantTwist(s.vx, s.vy, s.wz, dt));
    seg.distance += std::hypot(s.vx, s.vy) * dt;
    if (s.zero_speed) seg.zero_speed_time += dt;
  };

  for (const auto & s : samples) {
    if (s.t <= t0) continue;
    if (s.t > t1) break;
    step(s, s.t - t_prev);
    t_prev = s.t;
    last = &s;
  }
  if (last != nullptr) {
    step(*last, t1 - t_prev);
  }
  return seg;
}

/**
 * @brief Diagonal std-devs for a Pose3 BetweenFactor, GTSAM order [roll, pitch, yaw, x, y, z].
 *
 * Translation noise grows with distance and with time spent at zero-speed readings
 * (possible undetected creep). Yaw noise grows with the angle turned.
 */
inline std::array<double, 6> betweenStd(const Segment & seg, const FactorNoise & n)
{
  const double trans =
    std::max(n.min_std, n.trans_std_per_meter * seg.distance + n.zero_speed_std * seg.zero_speed_time);
  const double yaw = std::max(n.min_std, n.yaw_std_per_rad * std::abs(seg.delta.theta));
  return {n.loose_std, n.loose_std, yaw, trans, trans, n.loose_std};
}

}  // namespace eidos::wheel_kinematics

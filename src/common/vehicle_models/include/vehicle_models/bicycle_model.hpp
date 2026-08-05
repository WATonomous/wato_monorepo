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

#include <Eigen/Dense>
#include <cmath>

namespace vehicle_models
{

// State vector dimension: [x, y, theta, v]
constexpr int STATE_DIM = 4;

// Control vector dimension: [delta (steering angle), a (acceleration)]
constexpr int CONTROL_DIM = 2;

using StateVec = Eigen::Matrix<double, STATE_DIM, 1>;
using ControlVec = Eigen::Matrix<double, CONTROL_DIM, 1>;
using StateMat = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;
using InputMat = Eigen::Matrix<double, STATE_DIM, CONTROL_DIM>;

/// @brief Extract the yaw (heading, the state's theta) from a quaternion's
/// components. Message-type agnostic so both ROS and ROS-free callers can use it.
inline double yaw_from_quaternion(double qx, double qy, double qz, double qw)
{
  return std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
}

/**
 * @brief Result of linearizing the bicycle model at an operating point.
 *
 * Represents the discrete-time affine system:
 *   x_{k+1} = A * x_k + B * u_k + g
 */
struct LinearizedModel
{
  // Discrete state transition matrix
  StateMat A;

  // Discrete input matrix
  InputMat B;

  // Affine offset from linearization (captures nonlinearity residual)
  StateVec g;
};

/**
 * @brief Slip-corrected kinematic bicycle model for vehicle motion prediction.
 *
 * Models the vehicle as a rigid body with front-axle steering, referenced at
 * the centre of gravity (CoG). State is [x, y, theta, v] where theta is heading
 * and v is longitudinal speed. Control inputs are steering angle and
 * longitudinal acceleration.
 *
 * The body slip angle beta = atan((lr/L) * tan(delta)) is applied so the
 * velocity vector points along (theta + beta), capturing the steady-state
 * cornering geometry that the rear-axle kinematic model omits. Tire forces are
 * not modelled (no dynamic/slip-ratio effects) — this is the kinematic regime.
 *
 * This lives in the shared `vehicle_models` package so controllers and planners
 * can reuse it. Consumers depend only on this narrow surface (dynamics + step +
 * linearize), so a higher-fidelity model (e.g. an acados dynamic bicycle) can be
 * swapped in without changing callers.
 */
class BicycleModel
{
public:
  /// @brief Construct with the vehicle's geometry.
  /// @param wheelbase Distance between front and rear axles in meters.
  /// @param lr Distance from the rear axle to the centre of gravity in meters.
  ///           Values <= 0 default to wheelbase / 2 (CoG at the midpoint).
  explicit BicycleModel(double wheelbase, double lr = 0.0);

  /**
   * @brief Evaluate continuous-time nonlinear dynamics: x_dot = f(x, u).
   * @param x Current state [x, y, theta, v].
   * @param u Control input [steering_angle, acceleration].
   * @return Time derivative of the state vector.
   */
  StateVec dynamics(const StateVec & x, const ControlVec & u) const;

  /**
   * @brief Integrate the nonlinear dynamics one step with classic RK4.
   * @param x State at the start of the step.
   * @param u Control input held constant over the step.
   * @param dt Step length in seconds.
   * @return State after dt (the nominal discrete transition).
   */
  StateVec step(const StateVec & x, const ControlVec & u, double dt) const;

  /**
   * @brief Linearize and discretize dynamics around an operating point.
   *
   * Builds the discrete affine model of the RK4 integrator at (x_ref, u_ref).
   * The Jacobians A_d = d(step)/dx and B_d = d(step)/du are obtained by central
   * finite differences of step(); the affine term g_d makes the linearization
   * exact at the operating point.
   *
   * @param x_ref State operating point for linearization.
   * @param u_ref Control operating point for linearization (curvature feedforward).
   * @param dt Time step for discretization in seconds.
   * @return LinearizedModel with A_d, B_d, g_d such that:
   *         x_{k+1} = A_d * x_k + B_d * u_k + g_d
   */
  LinearizedModel linearize(const StateVec & x_ref, const ControlVec & u_ref, double dt) const;

  /// @brief Get the wheelbase used by this model.
  double wheelbase() const
  {
    return wheelbase_;
  }

  /// @brief Get the rear-axle-to-CoG distance used by this model.
  double lr() const
  {
    return lr_;
  }

private:
  // Front-to-rear axle distance in meters
  double wheelbase_;

  // Rear-axle-to-CoG distance in meters (for the slip-angle correction)
  double lr_;
};

}  // namespace vehicle_models

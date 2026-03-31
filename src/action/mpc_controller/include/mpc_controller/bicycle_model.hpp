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

namespace mpc_controller
{

// State vector dimension: [x, y, theta, v]
constexpr int STATE_DIM = 4;

// Control vector dimension: [delta (steering angle), a (acceleration)]
constexpr int CONTROL_DIM = 2;

using StateVec = Eigen::Matrix<double, STATE_DIM, 1>;
using ControlVec = Eigen::Matrix<double, CONTROL_DIM, 1>;
using StateMat = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;
using InputMat = Eigen::Matrix<double, STATE_DIM, CONTROL_DIM>;

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
 * @brief Kinematic bicycle model for vehicle motion prediction.
 *
 * Models the vehicle as a rigid body with front-axle steering. The rear axle
 * is the reference point. State is [x, y, theta, v] where theta is heading
 * and v is longitudinal speed. Control inputs are steering angle and
 * longitudinal acceleration.
 *
 * Used by MpcCore to linearize dynamics at each horizon step for QP
 * formulation. The model assumes small slip angles (kinematic regime).
 */
class BicycleModel
{
public:
  /// @brief Construct with the vehicle's wheelbase (front-to-rear axle distance).
  /// @param wheelbase Distance between front and rear axles in meters.
  explicit BicycleModel(double wheelbase);

  /**
   * @brief Evaluate continuous-time nonlinear dynamics: x_dot = f(x, u).
   * @param x Current state [x, y, theta, v].
   * @param u Control input [steering_angle, acceleration].
   * @return Time derivative of the state vector.
   */
  StateVec dynamics(const StateVec & x, const ControlVec & u) const;

  /**
   * @brief Linearize and discretize dynamics around an operating point.
   *
   * Computes the Jacobians df/dx and df/du at (x_ref, u_ref), then applies
   * forward Euler discretization with time step dt.
   *
   * @param x_ref State operating point for linearization.
   * @param u_ref Control operating point for linearization.
   * @param dt Time step for Euler discretization in seconds.
   * @return LinearizedModel with A_d, B_d, g_d such that:
   *         x_{k+1} = A_d * x_k + B_d * u_k + g_d
   */
  LinearizedModel linearize(const StateVec & x_ref, const ControlVec & u_ref, double dt) const;

  /// @brief Get the wheelbase used by this model.
  double wheelbase() const { return wheelbase_; }

private:
  // Front-to-rear axle distance in meters
  double wheelbase_;
};

}  // namespace mpc_controller

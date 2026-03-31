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

// State: [x, y, theta, v]
// Control: [delta (steering angle), a (acceleration)]
constexpr int STATE_DIM = 4;
constexpr int CONTROL_DIM = 2;

using StateVec = Eigen::Matrix<double, STATE_DIM, 1>;
using ControlVec = Eigen::Matrix<double, CONTROL_DIM, 1>;
using StateMat = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;
using InputMat = Eigen::Matrix<double, STATE_DIM, CONTROL_DIM>;

struct LinearizedModel
{
  StateMat A;    // Discrete state transition matrix
  InputMat B;    // Discrete input matrix
  StateVec g;    // Affine term from linearization
};

class BicycleModel
{
public:
  explicit BicycleModel(double wheelbase);

  /// Evaluate nonlinear dynamics: x_dot = f(x, u)
  StateVec dynamics(const StateVec & x, const ControlVec & u) const;

  /// Linearize around (x_ref, u_ref) and discretize with forward Euler.
  /// Returns A_d, B_d, g_d such that: x_{k+1} ≈ A_d * x_k + B_d * u_k + g_d
  LinearizedModel linearize(const StateVec & x_ref, const ControlVec & u_ref, double dt) const;

  double wheelbase() const { return wheelbase_; }

private:
  double wheelbase_;
};

}  // namespace mpc_controller

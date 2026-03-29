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

#include "mpc_controller/bicycle_model.hpp"

#include <cmath>

namespace mpc_controller
{

BicycleModel::BicycleModel(double wheelbase)
: wheelbase_(wheelbase)
{}

StateVec BicycleModel::dynamics(const StateVec & x, const ControlVec & u) const
{
  const double theta = x(2);
  const double v = x(3);
  const double delta = u(0);
  const double a = u(1);

  StateVec x_dot;
  x_dot(0) = v * std::cos(theta);
  x_dot(1) = v * std::sin(theta);
  x_dot(2) = v / wheelbase_ * std::tan(delta);
  x_dot(3) = a;
  return x_dot;
}

LinearizedModel BicycleModel::linearize(
  const StateVec & x_ref, const ControlVec & u_ref, double dt) const
{
  const double theta = x_ref(2);
  const double v = x_ref(3);
  const double delta = u_ref(0);

  const double cos_t = std::cos(theta);
  const double sin_t = std::sin(theta);
  const double tan_d = std::tan(delta);
  const double cos_d_sq = std::cos(delta) * std::cos(delta);

  // Continuous-time Jacobians
  // df/dx:
  // [ 0  0  -v*sin(theta)  cos(theta)          ]
  // [ 0  0   v*cos(theta)  sin(theta)          ]
  // [ 0  0   0              tan(delta)/L        ]
  // [ 0  0   0              0                   ]
  StateMat Ac = StateMat::Zero();
  Ac(0, 2) = -v * sin_t;
  Ac(0, 3) = cos_t;
  Ac(1, 2) = v * cos_t;
  Ac(1, 3) = sin_t;
  Ac(2, 3) = tan_d / wheelbase_;

  // df/du:
  // [ 0                       0 ]
  // [ 0                       0 ]
  // [ v/(L*cos^2(delta))      0 ]
  // [ 0                       1 ]
  InputMat Bc = InputMat::Zero();
  Bc(2, 0) = v / (wheelbase_ * cos_d_sq);
  Bc(3, 1) = 1.0;

  // Forward Euler discretization: A_d = I + dt*Ac, B_d = dt*Bc
  LinearizedModel model;
  model.A = StateMat::Identity() + dt * Ac;
  model.B = dt * Bc;

  // Affine term: g = x_ref + dt*f(x_ref, u_ref) - A_d*x_ref - B_d*u_ref
  StateVec f_ref = dynamics(x_ref, u_ref);
  model.g = x_ref + dt * f_ref - model.A * x_ref - model.B * u_ref;

  return model;
}

}  // namespace mpc_controller

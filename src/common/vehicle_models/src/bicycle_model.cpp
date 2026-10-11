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

#include "vehicle_models/bicycle_model.hpp"

#include <cmath>

namespace vehicle_models
{

BicycleModel::BicycleModel(double wheelbase, double lr)
: wheelbase_(wheelbase)
, lr_(lr > 0.0 ? lr : wheelbase / 2.0)
{}

StateVec BicycleModel::dynamics(const StateVec & x, const ControlVec & u) const
{
  const double theta = x(2);
  const double v = x(3);
  const double delta = u(0);
  const double a = u(1);

  // Body slip angle from the steady-state cornering geometry (CoG reference).
  const double beta = std::atan((lr_ / wheelbase_) * std::tan(delta));

  StateVec x_dot;
  x_dot(0) = v * std::cos(theta + beta);
  x_dot(1) = v * std::sin(theta + beta);
  // theta_dot = (v / lr) * sin(beta) == (v / L) * cos(beta) * tan(delta)
  x_dot(2) = v * std::cos(beta) * std::tan(delta) / wheelbase_;
  x_dot(3) = a;
  return x_dot;
}

StateVec BicycleModel::step(const StateVec & x, const ControlVec & u, double dt) const
{
  // Classic 4th-order Runge-Kutta with the control held constant over the step.
  const StateVec k1 = dynamics(x, u);
  const StateVec k2 = dynamics(x + 0.5 * dt * k1, u);
  const StateVec k3 = dynamics(x + 0.5 * dt * k2, u);
  const StateVec k4 = dynamics(x + dt * k3, u);
  return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

LinearizedModel BicycleModel::linearize(const StateVec & x_ref, const ControlVec & u_ref, double dt) const
{
  // Nominal discrete transition via RK4.
  const StateVec x_next = step(x_ref, u_ref, dt);

  // A_d = d(step)/dx and B_d = d(step)/du via central finite differences.
  // The 4-state / 2-input model makes this cheap (12 RK4 evaluations) and
  // avoids hand-deriving the RK4 stage Jacobians.
  const double eps = 1e-6;

  StateMat A = StateMat::Zero();
  for (int j = 0; j < STATE_DIM; ++j) {
    StateVec xp = x_ref;
    StateVec xm = x_ref;
    xp(j) += eps;
    xm(j) -= eps;
    A.col(j) = (step(xp, u_ref, dt) - step(xm, u_ref, dt)) / (2.0 * eps);
  }

  InputMat B = InputMat::Zero();
  for (int j = 0; j < CONTROL_DIM; ++j) {
    ControlVec up = u_ref;
    ControlVec um = u_ref;
    up(j) += eps;
    um(j) -= eps;
    B.col(j) = (step(x_ref, up, dt) - step(x_ref, um, dt)) / (2.0 * eps);
  }

  LinearizedModel model;
  model.A = A;
  model.B = B;
  // Affine term makes the linearization exact at the operating point:
  // A_d*x_ref + B_d*u_ref + g_d == step(x_ref, u_ref, dt).
  model.g = x_next - A * x_ref - B * u_ref;

  return model;
}

}  // namespace vehicle_models

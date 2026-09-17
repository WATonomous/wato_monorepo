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

#include <cmath>

#include <catch2/catch_all.hpp>
#include <wato_test/wato_test.hpp>

#include "vehicle_models/bicycle_model.hpp"

namespace wato
{

using vehicle_models::BicycleModel;
using vehicle_models::ControlVec;
using vehicle_models::STATE_DIM;
using vehicle_models::StateVec;

TEST_CASE_METHOD(test::TestExecutorFixture, "BicycleModel dynamics - straight line", "[bicycle]")
{
  BicycleModel model(2.5);
  StateVec x;
  x << 0.0, 0.0, 0.0, 5.0;  // facing +x, speed 5 m/s
  ControlVec u;
  u << 0.0, 0.0;  // no steering, no acceleration

  StateVec x_dot = model.dynamics(x, u);

  CHECK(x_dot(0) == Catch::Approx(5.0).epsilon(1e-9));
  CHECK(x_dot(1) == Catch::Approx(0.0).margin(1e-9));
  CHECK(x_dot(2) == Catch::Approx(0.0).margin(1e-9));
  CHECK(x_dot(3) == Catch::Approx(0.0).margin(1e-9));
}

TEST_CASE_METHOD(test::TestExecutorFixture, "BicycleModel dynamics - turning with slip", "[bicycle]")
{
  BicycleModel model(2.5);  // lr defaults to wheelbase/2 = 1.25
  StateVec x;
  x << 0.0, 0.0, 0.0, 5.0;
  ControlVec u;
  u << 0.3, 0.0;  // steering left

  StateVec x_dot = model.dynamics(x, u);

  // Slip rotates the velocity vector by beta > 0, so the forward component
  // drops below v and a positive lateral component appears.
  CHECK(x_dot(0) < 5.0);
  CHECK(x_dot(0) > 4.5);
  CHECK(x_dot(1) > 0.0);   // lateral velocity from body slip
  CHECK(x_dot(2) > 0.0);   // positive yaw rate
  CHECK(x_dot(3) == Catch::Approx(0.0).margin(1e-9));

  // The slip model only rotates the velocity vector; its magnitude stays v.
  CHECK(std::hypot(x_dot(0), x_dot(1)) == Catch::Approx(5.0).epsilon(1e-9));
}

TEST_CASE_METHOD(test::TestExecutorFixture, "BicycleModel linearize - affine term consistency", "[bicycle]")
{
  BicycleModel model(2.5);
  StateVec x_ref;
  x_ref << 1.0, 2.0, 0.5, 3.0;
  ControlVec u_ref;
  u_ref << 0.1, 0.5;
  double dt = 0.1;

  auto lin = model.linearize(x_ref, u_ref, dt);

  // At the operating point, A*x_ref + B*u_ref + g must reproduce the nominal
  // RK4 discrete transition exactly (that is how g is defined).
  StateVec predicted = lin.A * x_ref + lin.B * u_ref + lin.g;
  StateVec expected = model.step(x_ref, u_ref, dt);

  for (int i = 0; i < STATE_DIM; ++i) {
    CHECK(predicted(i) == Catch::Approx(expected(i)).margin(1e-9));
  }
}

TEST_CASE_METHOD(test::TestExecutorFixture, "BicycleModel step - RK4 beats Euler on a curve", "[bicycle]")
{
  BicycleModel model(2.5);
  StateVec x0;
  x0 << 0.0, 0.0, 0.0, 8.0;
  ControlVec u;
  u << 0.2, 0.0;  // constant steering -> curved path
  double dt = 0.5;

  // High-accuracy reference: many tiny RK4 sub-steps.
  StateVec ref = x0;
  const int sub = 2000;
  for (int i = 0; i < sub; ++i) {
    ref = model.step(ref, u, dt / sub);
  }

  StateVec rk4 = model.step(x0, u, dt);
  StateVec euler = x0 + dt * model.dynamics(x0, u);

  double err_rk4 = (rk4 - ref).norm();
  double err_euler = (euler - ref).norm();

  CHECK(err_rk4 < err_euler);  // RK4 is more accurate than forward Euler
  CHECK(err_rk4 < 1e-3);       // single RK4 step is very close to the reference
}

TEST_CASE_METHOD(test::TestExecutorFixture, "BicycleModel linearize - zero state structure", "[bicycle]")
{
  BicycleModel model(2.5);
  StateVec x_ref = StateVec::Zero();
  ControlVec u_ref = ControlVec::Zero();
  double dt = 0.1;

  auto lin = model.linearize(x_ref, u_ref, dt);

  // At zero state (theta=0, v=0, delta=0) the RK4 discretization degenerates to
  // the same structure as a single Euler step: position integrates speed, speed
  // integrates acceleration, and steering has no effect because v = 0.
  CHECK(lin.A(0, 0) == Catch::Approx(1.0).margin(1e-9));
  CHECK(lin.A(1, 1) == Catch::Approx(1.0).margin(1e-9));
  CHECK(lin.A(2, 2) == Catch::Approx(1.0).margin(1e-9));
  CHECK(lin.A(3, 3) == Catch::Approx(1.0).margin(1e-9));
  CHECK(lin.A(0, 3) == Catch::Approx(dt).margin(1e-9));   // d x_next / d v = dt
  CHECK(lin.A(1, 3) == Catch::Approx(0.0).margin(1e-9));  // heading 0 -> no lateral motion

  // B should be near-zero except d v_next / d a = dt.
  CHECK(lin.B(3, 1) == Catch::Approx(dt).margin(1e-9));
  CHECK(lin.B(0, 0) == Catch::Approx(0.0).margin(1e-9));
  CHECK(lin.B(2, 0) == Catch::Approx(0.0).margin(1e-9));  // v=0 so no steering effect
}

}  // namespace wato

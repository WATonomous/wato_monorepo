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

#include "mpc_controller/bicycle_model.hpp"

namespace wato
{

using namespace mpc_controller;

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

TEST_CASE_METHOD(test::TestExecutorFixture, "BicycleModel dynamics - turning", "[bicycle]")
{
  BicycleModel model(2.5);
  StateVec x;
  x << 0.0, 0.0, 0.0, 5.0;
  ControlVec u;
  u << 0.3, 0.0;  // steering right

  StateVec x_dot = model.dynamics(x, u);

  CHECK(x_dot(0) == Catch::Approx(5.0).epsilon(1e-9));  // cos(0)*5
  CHECK(x_dot(1) == Catch::Approx(0.0).margin(1e-9));   // sin(0)*5
  CHECK(x_dot(2) > 0.0);  // positive yaw rate (turning left with positive delta)
  CHECK(x_dot(3) == Catch::Approx(0.0).margin(1e-9));
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

  // At the reference point, A*x_ref + B*u_ref + g should equal x_ref + dt*f(x_ref, u_ref)
  StateVec predicted = lin.A * x_ref + lin.B * u_ref + lin.g;
  StateVec expected = x_ref + dt * model.dynamics(x_ref, u_ref);

  for (int i = 0; i < STATE_DIM; ++i) {
    CHECK(predicted(i) == Catch::Approx(expected(i)).margin(1e-10));
  }
}

TEST_CASE_METHOD(test::TestExecutorFixture, "BicycleModel linearize - identity at zero", "[bicycle]")
{
  BicycleModel model(2.5);
  StateVec x_ref = StateVec::Zero();
  ControlVec u_ref = ControlVec::Zero();
  double dt = 0.1;

  auto lin = model.linearize(x_ref, u_ref, dt);

  // At zero state with zero control, A should be identity (v=0, all Jacobians zero)
  for (int i = 0; i < STATE_DIM; ++i) {
    for (int j = 0; j < STATE_DIM; ++j) {
      double expected = (i == j) ? 1.0 : 0.0;
      CHECK(lin.A(i, j) == Catch::Approx(expected).margin(1e-10));
    }
  }
}

}  // namespace wato

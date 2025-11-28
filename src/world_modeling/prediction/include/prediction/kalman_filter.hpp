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

#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <Eigen/Dense>
#include <vector>
#include <utility>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// KalmanCV7: Constant Velocity Kalman Filter
// State: [x, y, z, vx, vy, vz, yaw] (7 dimensions)
// Measurement: [x, y, z, yaw] (4 dimensions)
struct KalmanCV7 {
  int n = 7;
  int m = 4;
  VectorXd x;
  MatrixXd P;
  MatrixXd F;
  MatrixXd H;
  MatrixXd Q;
  MatrixXd R;
  double dt;

  KalmanCV7(double dt_ = 0.05,
            double q_pos = 0.1,
            double q_vel = 1.0,
            double q_yaw = 0.01,
            double r_pos = 0.5,
            double r_yaw = 0.1);

  void set_dt(double newdt);
  void init_from_measurement(const VectorXd &meas);
  void predict();
  void update(const VectorXd &z);
  std::vector<std::pair<VectorXd, MatrixXd>> rollout_predictions(int steps);
};

// TrackState: represents a single object track
struct TrackState {
  KalmanCV7 kf;
  double last_stamp = 0.0;
  double l = 0.0, w = 0.0, h = 0.0;  // length, width, height
  bool size_known = false;

  TrackState(double dt = 0.05);
};

#endif  // KALMAN_FILTER_HPP

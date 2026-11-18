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

#include <gtest/gtest.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <cmath>
#include <vector>

#include "eidos/utils/biased_relative_pose_factor.hpp"

namespace eidos
{

namespace
{

gtsam::Key X(std::size_t i)
{
  return gtsam::Symbol('x', i);
}
gtsam::Key C(std::size_t i)
{
  return gtsam::Symbol('c', i);
}

/// A drive of 5 m steps: straight, a left turn, straight, a right turn, straight.
std::vector<gtsam::Pose3> drive(std::size_t steps)
{
  std::vector<gtsam::Pose3> poses{gtsam::Pose3()};
  for (std::size_t i = 1; i < steps; ++i) {
    double yaw = 0.0;
    if (i % 40 >= 10 && i % 40 < 18) yaw = 0.05;
    if (i % 40 >= 28 && i % 40 < 36) yaw = -0.05;
    poses.push_back(poses.back() * gtsam::Pose3(gtsam::Rot3::Yaw(yaw), gtsam::Point3(5.0, 0.0, 0.0)));
  }
  return poses;
}

}  // namespace

TEST(BiasedRelativePoseFactor, BiasModelInverts)
{
  const gtsam::Pose3 relative(gtsam::Rot3::Ypr(0.04, 0.01, -0.02), gtsam::Point3(5.0, 0.3, 0.1));
  const gtsam::Vector3 bias(-0.035, 5e-4, -0.04);
  EXPECT_TRUE(unbiasedRelativePose(biasedRelativePose(relative, bias), bias).equals(relative, 1e-9));
  EXPECT_TRUE(biasedRelativePose(relative, gtsam::Vector3::Zero()).equals(relative, 1e-12));
}

TEST(BiasedRelativePoseFactor, ZeroErrorAtTruth)
{
  const gtsam::Pose3 a(gtsam::Rot3::Yaw(0.3), gtsam::Point3(1, 2, 0));
  const gtsam::Pose3 b = a * gtsam::Pose3(gtsam::Rot3::Yaw(0.05), gtsam::Point3(5, 0.1, 0));
  const gtsam::Vector3 bias(-0.03, 4e-4, -0.05);
  const BiasedRelativePoseFactor factor(
    X(0), X(1), C(0), biasedRelativePose(a.between(b), bias), gtsam::noiseModel::Isotropic::Sigma(6, 0.1));
  gtsam::Matrix h1, h2, h3;
  EXPECT_LT(factor.evaluateError(a, b, bias, h1, h2, h3).norm(), 1e-9);
  EXPECT_EQ(h3.rows(), 6);
  EXPECT_EQ(h3.cols(), 3);
  EXPECT_GT(h3.norm(), 0.0);
}

TEST(BiasedRelativePoseFactor, GyroAndGpsRecoverTheBias)
{
  // Biased relative poses (no noise), gyro-like relative rotations (translation free) and GPS-like
  // positions every 4 steps: the graph must find the bias and the true poses.
  const auto truth = drive(120);
  const gtsam::Vector3 bias(-0.035, 4.7e-4, -0.04);
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;
  const auto camera = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector6() << 0.005, 0.005, 0.005, 0.2, 0.2, 0.2).finished());
  const auto gyro = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector6() << 2e-4, 2e-4, 2e-4, 1e3, 1e3, 1e3).finished());
  const auto gps = gtsam::noiseModel::Isotropic::Sigma(3, 0.3);
  graph.addPrior(X(0), truth[0], gtsam::noiseModel::Isotropic::Sigma(6, 1e-4));
  graph.addPrior(C(0), gtsam::Vector3(gtsam::Vector3::Zero()), gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.1, 1e-3, 0.1)));
  initial.insert(X(0), truth[0]);
  initial.insert(C(0), gtsam::Vector3(gtsam::Vector3::Zero()));
  for (std::size_t i = 1; i < truth.size(); ++i) {
    const gtsam::Pose3 relative = truth[i - 1].between(truth[i]);
    graph.add(BiasedRelativePoseFactor(X(i - 1), X(i), C(i), biasedRelativePose(relative, bias), camera));
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(X(i - 1), X(i), relative, gyro));
    graph.add(gtsam::BetweenFactor<gtsam::Vector3>(
      C(i - 1), C(i), gtsam::Vector3::Zero(), gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(1e-3, 1e-5, 1e-3))));
    if (i % 4 == 0) graph.add(gtsam::GPSFactor(X(i), truth[i].translation(), gps));
    initial.insert(X(i), initial.at<gtsam::Pose3>(X(i - 1)) * biasedRelativePose(relative, bias));
    initial.insert(C(i), gtsam::Vector3(gtsam::Vector3::Zero()));
  }
  const gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
  const gtsam::Vector3 estimate = result.at<gtsam::Vector3>(C(truth.size() / 2));
  EXPECT_NEAR(estimate(kTranslationScale), bias(kTranslationScale), 0.003);
  EXPECT_NEAR(estimate(kYawPerMetre), bias(kYawPerMetre), 5e-5);
  EXPECT_NEAR(estimate(kRotationScale), bias(kRotationScale), 0.01);
  double worst = 0.0;
  for (std::size_t i = 0; i < truth.size(); ++i) {
    worst = std::max(worst, (result.at<gtsam::Pose3>(X(i)).translation() - truth[i].translation()).norm());
  }
  EXPECT_LT(worst, 0.3);
}

}  // namespace eidos

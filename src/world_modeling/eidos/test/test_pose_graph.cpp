#include <catch2/catch_all.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include "eidos/pose_graph.hpp"

using namespace eidos;

TEST_CASE("PoseGraph constructs empty", "[pose_graph]") {
  PoseGraph pg;
  REQUIRE(pg.numFactors() == 0);
}

TEST_CASE("PoseGraph add prior and optimize", "[pose_graph]") {
  PoseGraph pg;

  gtsam::NonlinearFactorGraph factors;
  gtsam::Values initial;

  auto noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.3, 0.3, 0.3).finished());

  gtsam::Pose3 origin = gtsam::Pose3::Identity();
  factors.add(gtsam::PriorFactor<gtsam::Pose3>(0, origin, noise));
  initial.insert(0, origin);

  auto result = pg.update(factors, initial);

  REQUIRE(result.exists(0));
  gtsam::Pose3 optimized = result.at<gtsam::Pose3>(0);
  REQUIRE(optimized.translation().x() == Catch::Approx(0.0).margin(1e-3));
  REQUIRE(optimized.translation().y() == Catch::Approx(0.0).margin(1e-3));
  REQUIRE(optimized.translation().z() == Catch::Approx(0.0).margin(1e-3));
}

TEST_CASE("PoseGraph getOptimizedPose", "[pose_graph]") {
  PoseGraph pg;

  gtsam::NonlinearFactorGraph factors;
  gtsam::Values initial;

  auto noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.3, 0.3, 0.3).finished());

  gtsam::Pose3 p0 = gtsam::Pose3::Identity();
  factors.add(gtsam::PriorFactor<gtsam::Pose3>(0, p0, noise));
  initial.insert(0, p0);

  pg.update(factors, initial);
  gtsam::Pose3 result = pg.getOptimizedPose(0);
  REQUIRE(result.translation().norm() == Catch::Approx(0.0).margin(1e-3));
}

TEST_CASE("PoseGraph two poses with BetweenFactor", "[pose_graph]") {
  PoseGraph pg;

  auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished());
  auto odom_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.3, 0.3, 0.3).finished());

  gtsam::Pose3 p0 = gtsam::Pose3::Identity();
  gtsam::Pose3 delta = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0));

  // First update: prior + first pose
  {
    gtsam::NonlinearFactorGraph factors;
    gtsam::Values initial;
    factors.add(gtsam::PriorFactor<gtsam::Pose3>(0, p0, prior_noise));
    initial.insert(0, p0);
    pg.update(factors, initial);
  }

  // Second update: between factor + second pose
  {
    gtsam::NonlinearFactorGraph factors;
    gtsam::Values initial;
    factors.add(gtsam::BetweenFactor<gtsam::Pose3>(0, 1, delta, odom_noise));
    initial.insert(1, p0.compose(delta));
    pg.update(factors, initial);
  }

  gtsam::Pose3 result0 = pg.getOptimizedPose(0);
  gtsam::Pose3 result1 = pg.getOptimizedPose(1);

  REQUIRE(result0.translation().x() == Catch::Approx(0.0).margin(0.1));
  REQUIRE(result1.translation().x() == Catch::Approx(1.0).margin(0.1));
  REQUIRE(result1.translation().y() == Catch::Approx(0.0).margin(0.1));
}

TEST_CASE("PoseGraph numFactors increases", "[pose_graph]") {
  PoseGraph pg;

  auto noise = gtsam::noiseModel::Isotropic::Sigma(6, 0.1);
  gtsam::Pose3 origin = gtsam::Pose3::Identity();

  gtsam::NonlinearFactorGraph factors;
  gtsam::Values initial;
  factors.add(gtsam::PriorFactor<gtsam::Pose3>(0, origin, noise));
  initial.insert(0, origin);
  pg.update(factors, initial);

  REQUIRE(pg.numFactors() > 0);
}

TEST_CASE("PoseGraph reset clears state", "[pose_graph]") {
  PoseGraph pg;

  auto noise = gtsam::noiseModel::Isotropic::Sigma(6, 0.1);
  gtsam::Pose3 origin = gtsam::Pose3::Identity();

  gtsam::NonlinearFactorGraph factors;
  gtsam::Values initial;
  factors.add(gtsam::PriorFactor<gtsam::Pose3>(0, origin, noise));
  initial.insert(0, origin);
  pg.update(factors, initial);

  REQUIRE(pg.numFactors() > 0);
  pg.reset();
  REQUIRE(pg.numFactors() == 0);
}

TEST_CASE("PoseGraph updateExtra does not crash", "[pose_graph]") {
  PoseGraph pg;

  auto noise = gtsam::noiseModel::Isotropic::Sigma(6, 0.1);
  gtsam::Pose3 origin = gtsam::Pose3::Identity();

  gtsam::NonlinearFactorGraph factors;
  gtsam::Values initial;
  factors.add(gtsam::PriorFactor<gtsam::Pose3>(0, origin, noise));
  initial.insert(0, origin);
  pg.update(factors, initial);

  REQUIRE_NOTHROW(pg.updateExtra(5));
}

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

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <memory>

namespace eidos
{

/**
 * @brief Wrapper around GTSAM's ISAM2 incremental optimizer.
 *
 * Not thread-safe — only called from the SLAM loop thread via Estimator.
 * Estimator serializes all access, so no mutex needed.
 */
class PoseGraph
{
public:
  PoseGraph();

  /// Add factors and initial values, then optimize.
  /// @return Updated optimized values.
  gtsam::Values update(
    const gtsam::NonlinearFactorGraph & factors, const gtsam::Values & initial_values, int num_iterations = 2);

  /// Run extra ISAM2 update iterations (e.g. after loop closure).
  void updateExtra(int num_iterations);

  /// Get all current optimized values.
  gtsam::Values getOptimizedValues() const;

  /// Get the optimized pose for a given GTSAM key.
  gtsam::Pose3 getOptimizedPose(gtsam::Key key) const;

  /// Get the marginal covariance for a given GTSAM key.
  Eigen::MatrixXd getMarginalCovariance(gtsam::Key key) const;

  /// Get the total number of factors.
  int numFactors() const;

  /// Reset the optimizer with new ISAM2 parameters.
  void reset(double relinearize_threshold = 0.1, int relinearize_skip = 1);

private:
  std::unique_ptr<gtsam::ISAM2> isam_;
  gtsam::Values current_estimate_;
};

}  // namespace eidos

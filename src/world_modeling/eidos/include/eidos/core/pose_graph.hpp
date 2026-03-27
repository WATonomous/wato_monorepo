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
  /// @brief Construct the PoseGraph with default ISAM2 parameters.
  PoseGraph();

  /**
   * @brief Add factors and initial values to the ISAM2 graph, then optimize.
   * @param factors        New factors to incorporate.
   * @param initial_values Initial value estimates for new variables.
   * @param num_iterations Number of ISAM2 update iterations.
   * @return All optimized values after the update.
   * @note Not thread-safe -- only call from the SLAM loop thread.
   */
  gtsam::Values update(
    const gtsam::NonlinearFactorGraph & factors, const gtsam::Values & initial_values, int num_iterations = 2);

  /**
   * @brief Run additional ISAM2 update iterations without adding new factors.
   *
   * Useful for improving convergence after loop closures or corrections.
   * @param num_iterations Number of extra iterations to run.
   */
  void updateExtra(int num_iterations);

  /**
   * @brief Get all current optimized values from the ISAM2 estimate.
   * @return A copy of all optimized Values.
   */
  gtsam::Values getOptimizedValues() const;

  /**
   * @brief Get the optimized Pose3 for a given GTSAM key.
   * @param key The GTSAM key to look up.
   * @return The optimized Pose3 for that key.
   */
  gtsam::Pose3 getOptimizedPose(gtsam::Key key) const;

  /**
   * @brief Compute the 6x6 marginal covariance for a given GTSAM key.
   * @param key The GTSAM key to compute covariance for.
   * @return The marginal covariance matrix.
   */
  Eigen::MatrixXd getMarginalCovariance(gtsam::Key key) const;

  /**
   * @brief Get the total number of factors in the ISAM2 graph.
   * @return Number of factors.
   */
  int numFactors() const;

  /**
   * @brief Reset the ISAM2 optimizer with new parameters.
   *
   * Clears all factors and values, creating a fresh ISAM2 instance.
   * @param relinearize_threshold ISAM2 relinearization threshold (default 0.1).
   * @param relinearize_skip      Number of updates between relinearizations (default 1).
   */
  void reset(double relinearize_threshold = 0.1, int relinearize_skip = 1);

private:
  std::unique_ptr<gtsam::ISAM2> isam_;
  gtsam::Values current_estimate_;
};

}  // namespace eidos

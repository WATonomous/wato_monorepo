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
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <memory>
#include <optional>
#include <string>

#include "eidos/core/pose_graph.hpp"
#include "eidos/utils/atomic_slot.hpp"
#include "eidos/utils/lock_free_pose.hpp"

namespace eidos
{

/**
 * @brief ISAM2 optimization engine. No orchestration — EidosNode tells it
 * when to optimize.
 *
 * After each optimization, writes the latest pose to a LockFreePose that
 * eidos_transform can read without blocking.
 *
 * Only called from the SLAM loop thread — not thread-safe on its own.
 */
class Estimator
{
public:
  Estimator();

  void configure(
    double relinearize_threshold,
    int relinearize_skip,
    const std::vector<double> & prior_pose_cov,
    int update_iterations,
    int correction_iterations,
    int loop_closure_iterations);
  void reset();

  /**
   * @brief Add factors and values, run ISAM2 optimization.
   * Updates the internal LockFreePose with the latest optimized pose.
   * @return All optimized values.
   */
  gtsam::Values optimize(
    const gtsam::NonlinearFactorGraph & factors,
    const gtsam::Values & values,
    gtsam::Key latest_key,
    int num_iterations = 2);

  /// Run additional ISAM2 iterations (e.g. after loop closure or GPS correction).
  void optimizeExtra(int num_iterations, gtsam::Key latest_key);

  /// Get the latest optimized pose (thread-safe, lock-free).
  const LockFreePose & getOptimizedPose() const
  {
    return optimized_pose_;
  }

  /// Get the latest optimized values (thread-safe, lock-free).
  /// Written after each optimize(), read by VisualizationRunner.
  const AtomicSlot<gtsam::Values> & getOptimizedValues() const
  {
    return optimized_values_;
  }

  /// Get optimized pose for a specific key. Only call from SLAM loop thread.
  gtsam::Pose3 getPose(gtsam::Key key) const;

  /// Get all optimized values. Only call from SLAM loop thread.
  gtsam::Values getValues() const;

  /// Get marginal covariance for a key. Only call from SLAM loop thread.
  std::optional<Eigen::MatrixXd> getCovariance(gtsam::Key key) const;

  int numFactors() const;

  // ---- State timeline (tracks ISAM2 graph keys) ----
  gtsam::Key getLastStateKey() const
  {
    return last_state_key_;
  }

  uint64_t getNextStateIndex() const
  {
    return next_state_index_;
  }

  bool hasState() const
  {
    return has_last_state_;
  }

  /// Allocate the next GTSAM key and record it. Returns the new key.
  gtsam::Key createState(double timestamp, const std::string & owner);

  int getUpdateIterations() const
  {
    return update_iterations_;
  }

  int getCorrectionIterations() const
  {
    return correction_iterations_;
  }

  int getLoopClosureIterations() const
  {
    return loop_closure_iterations_;
  }

  const std::vector<double> & getPriorPoseCov() const
  {
    return prior_pose_cov_;
  }

private:
  PoseGraph pose_graph_;
  LockFreePose optimized_pose_;  ///< Written after optimize, read by eidos_transform
  AtomicSlot<gtsam::Values> optimized_values_;  ///< Written after optimize, read by VisualizationRunner

  int update_iterations_ = 2;  ///< Normal ISAM2 iterations per update
  int correction_iterations_ = 5;  ///< Extra iterations after GPS correction
  int loop_closure_iterations_ = 20;  ///< Extra iterations after loop closure
  std::vector<double> prior_pose_cov_;  ///< State 0 prior noise [x,y,z,roll,pitch,yaw]

  // ---- State timeline ----
  uint64_t next_state_index_ = 0;  ///< Monotonic counter for Symbol('x', N)
  gtsam::Key last_state_key_ = 0;
  double last_state_timestamp_ = 0.0;
  std::string last_state_owner_;
  bool has_last_state_ = false;
};

}  // namespace eidos

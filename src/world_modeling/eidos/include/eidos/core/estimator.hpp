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
#include <vector>

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
  /// @brief Default constructor. Call configure() before use.
  Estimator();

  /**
   * @brief Configure ISAM2 parameters and iteration counts.
   *
   * Must be called once before any optimization. Resets the underlying
   * PoseGraph with the given relinearization settings.
   * @param relinearize_threshold ISAM2 relinearization threshold.
   * @param relinearize_skip      Number of updates to skip between relinearizations.
   * @param prior_pose_cov        Diagonal variances for the state-0 prior
   *                              noise model [x, y, z, roll, pitch, yaw].
   * @param update_iterations     Normal ISAM2 iterations per optimize() call.
   * @param correction_iterations Extra iterations after a GPS-style correction.
   * @param loop_closure_iterations Extra iterations after a loop closure.
   */
  void configure(
    double relinearize_threshold,
    int relinearize_skip,
    const std::vector<double> & prior_pose_cov,
    int update_iterations,
    int correction_iterations,
    int loop_closure_iterations);

  /**
   * @brief Reset the estimator to its initial state.
   *
   * Clears the ISAM2 graph, resets the state timeline, and invalidates
   * the optimized pose.
   */
  void reset();

  /**
   * @brief Add factors and values, run ISAM2 optimization.
   *
   * Updates the internal LockFreePose and AtomicSlot<Values> with the
   * latest optimized results so downstream readers can access them lock-free.
   * @param factors        New factors to add to the graph.
   * @param values         Initial value estimates for any new variables.
   * @param latest_key     The key of the most recent state (used to extract
   *                       the optimized pose for the LockFreePose slot).
   * @param num_iterations Number of ISAM2 update iterations to run.
   * @return All optimized values after the update.
   */
  gtsam::Values optimize(
    const gtsam::NonlinearFactorGraph & factors,
    const gtsam::Values & values,
    gtsam::Key latest_key,
    int num_iterations = 2);

  /**
   * @brief Run additional ISAM2 iterations for better convergence.
   *
   * Typically called after loop closure or GPS correction. Updates the
   * LockFreePose and AtomicSlot with the re-optimized result.
   * @param num_iterations Number of extra ISAM2 iterations.
   * @param latest_key     Key of the latest state to extract the updated pose.
   */
  void optimizeExtra(int num_iterations, gtsam::Key latest_key);

  /**
   * @brief Add factors to the graph without creating new state variables.
   *
   * Used for standalone constraints (e.g. loop closure BetweenFactors)
   * that reference already-existing keys.
   * @param factors New factors to add.
   * @param values  Any auxiliary values needed (may be empty).
   */
  void addFactorsOnly(const gtsam::NonlinearFactorGraph & factors, const gtsam::Values & values);

  /**
   * @brief Get the lock-free slot holding the latest optimized pose.
   * @return Reference to the LockFreePose, readable from any thread.
   * @note Thread-safe: uses atomic triple-buffering internally.
   */
  const LockFreePose & getOptimizedPose() const
  {
    return optimized_pose_;
  }

  /**
   * @brief Get the lock-free slot holding the latest optimized values.
   * @return Reference to the AtomicSlot, readable from any thread.
   * @note Written after each optimize(), consumed by VisualizationRunner.
   */
  const AtomicSlot<gtsam::Values> & getOptimizedValues() const
  {
    return optimized_values_;
  }

  /**
   * @brief Get the optimized pose for a specific GTSAM key.
   * @param key The GTSAM key to look up.
   * @return The optimized Pose3 for that key.
   * @note Only call from the SLAM loop thread.
   */
  gtsam::Pose3 getPose(gtsam::Key key) const;

  /**
   * @brief Get all current optimized values from the ISAM2 graph.
   * @return A copy of all optimized Values.
   * @note Only call from the SLAM loop thread.
   */
  gtsam::Values getValues() const;

  /**
   * @brief Get the 6x6 marginal covariance matrix for a given key.
   * @param key The GTSAM key to compute covariance for.
   * @return The covariance matrix, or std::nullopt if computation fails.
   * @note Only call from the SLAM loop thread.
   */
  std::optional<Eigen::MatrixXd> getCovariance(gtsam::Key key) const;

  /// @brief Get the total number of factors in the ISAM2 graph.
  /// @return Number of factors.
  int numFactors() const;

  // ---- State timeline (tracks ISAM2 graph keys) ----

  /// @brief Get the GTSAM key of the most recently created state.
  /// @return The last state key.
  gtsam::Key getLastStateKey() const
  {
    return last_state_key_;
  }

  /// @brief Get the index that will be used for the next Symbol('x', N).
  /// @return The next monotonically increasing state index.
  uint64_t getNextStateIndex() const
  {
    return next_state_index_;
  }

  /// @brief Check whether at least one state has been created.
  /// @return True if createState() has been called at least once.
  bool hasState() const
  {
    return has_last_state_;
  }

  /**
   * @brief Allocate the next GTSAM key Symbol('x', N) and record it as
   *        the latest state.
   * @param timestamp The timestamp associated with this state.
   * @param owner     Name of the plugin that owns this state.
   * @return The newly created GTSAM key.
   */
  gtsam::Key createState(double timestamp, const std::string & owner);

  /// @brief Get the configured number of normal ISAM2 update iterations.
  /// @return Number of update iterations.
  int getUpdateIterations() const
  {
    return update_iterations_;
  }

  /// @brief Get the configured number of extra iterations after a GPS-style correction.
  /// @return Number of correction iterations.
  int getCorrectionIterations() const
  {
    return correction_iterations_;
  }

  /// @brief Get the configured number of extra iterations after loop closure.
  /// @return Number of loop closure iterations.
  int getLoopClosureIterations() const
  {
    return loop_closure_iterations_;
  }

  /// @brief Get the diagonal prior pose covariance [x, y, z, roll, pitch, yaw].
  /// @return Reference to the prior pose covariance vector.
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

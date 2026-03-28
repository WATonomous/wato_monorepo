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
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "eidos/utils/atomic_slot.hpp"
#include "eidos/utils/lock_free_pose.hpp"

namespace eidos
{

/**
 * @brief ISAM2 optimization engine.
 *
 * Wraps GTSAM's ISAM2 incremental optimizer. EidosNode feeds it factors and
 * tells it when to optimize. After each optimization, writes the latest pose
 * to a LockFreePose and the full values to an AtomicSlot for lock-free reads.
 *
 * Not thread-safe — only called from the SLAM loop thread.
 */
class GraphOptimizer
{
public:
  /// @brief Default constructor. Call configure() before use.
  GraphOptimizer();

  /**
   * @brief Configure ISAM2 parameters.
   *
   * Must be called once before any optimization. Creates a fresh ISAM2
   * instance with the given relinearization settings.
   * @param logger                ROS logger for debug/info/warn output.
   * @param relinearize_threshold ISAM2 relinearization threshold.
   * @param relinearize_skip      Number of updates to skip between relinearizations.
   * @param prior_pose_cov        Diagonal variances for the state-0 prior
   *                              noise model [x, y, z, roll, pitch, yaw].
   */
  void configure(
    rclcpp::Logger logger,
    double relinearize_threshold,
    int relinearize_skip,
    const std::vector<double> & prior_pose_cov);

  /**
   * @brief Reset the optimizer to its initial state.
   *
   * Clears the ISAM2 graph, resets the state timeline, and invalidates
   * the optimized pose.
   */
  void reset();

  /**
   * @brief Add factors and values, run ISAM2 optimization.
   *
   * Updates the internal LockFreePose and AtomicSlot<Values> with the
   * latest optimized results so downstream readers can access them.
   * @param factors          New factors to add to the graph.
   * @param values           Initial value estimates for any new variables.
   * @param latest_key       The key of the most recent state (used to extract
   *                         the optimized pose for the LockFreePose slot).
   * @param num_iterations   Number of ISAM2 update iterations.
   * @param extra_iterations Additional convergence iterations (default 0).
   * @return All optimized values after the update.
   */
  gtsam::Values optimize(
    const gtsam::NonlinearFactorGraph & factors,
    const gtsam::Values & values,
    gtsam::Key latest_key,
    int num_iterations,
    int extra_iterations = 0);

  /// @brief Get the lock-free slot holding the latest optimized pose.
  /// @return Reference to the LockFreePose, readable from any thread.
  const LockFreePose & getOptimizedPose() const
  {
    return optimized_pose_;
  }

  /// @brief Get the lock-free slot holding the latest optimized values.
  /// @return Reference to the AtomicSlot, readable from any thread.
  const AtomicSlot<gtsam::Values> & getOptimizedValues() const
  {
    return optimized_values_;
  }

  /// @brief Get the optimized pose for a specific GTSAM key.
  /// @param key The GTSAM key to look up.
  /// @return The optimized Pose3 for that key.
  gtsam::Pose3 getPose(gtsam::Key key) const;

  /// @brief Get all current optimized values from the ISAM2 graph.
  /// @return A copy of all optimized Values.
  gtsam::Values getValues() const;

  /// @brief Get the 6x6 marginal covariance matrix for a given key.
  /// @param key The GTSAM key to compute covariance for.
  /// @return The covariance matrix, or std::nullopt if computation fails.
  std::optional<Eigen::MatrixXd> getCovariance(gtsam::Key key) const;

  /// @brief Get the total number of factors in the ISAM2 graph.
  int numFactors() const;

  // ---- State timeline ----

  /// @brief Get the GTSAM key of the most recently created state.
  gtsam::Key getLastStateKey() const
  {
    return last_state_key_;
  }

  /// @brief Get the index that will be used for the next Symbol('x', N).
  uint64_t getNextStateIndex() const
  {
    return next_state_index_;
  }

  /// @brief Check whether at least one state has been created.
  bool hasState() const
  {
    return has_last_state_;
  }

  /// @brief Allocate the next GTSAM key Symbol('x', N) and record it.
  /// @param timestamp The timestamp associated with this state.
  /// @param owner     Name of the plugin that owns this state.
  /// @return The newly created GTSAM key.
  gtsam::Key createState(double timestamp, const std::string & owner);

  /// @brief Get the diagonal prior pose covariance [x, y, z, roll, pitch, yaw].
  const std::vector<double> & getPriorPoseCov() const
  {
    return prior_pose_cov_;
  }

private:
  // ---- ISAM2 ----
  std::unique_ptr<gtsam::ISAM2> isam_;
  gtsam::Values current_estimate_;
  double relinearize_threshold_ = 0.1;
  int relinearize_skip_ = 1;

  // ---- Lock-free outputs ----
  LockFreePose optimized_pose_;
  AtomicSlot<gtsam::Values> optimized_values_;

  // ---- Config ----
  rclcpp::Logger logger_{rclcpp::get_logger("graph_optimizer")};
  std::vector<double> prior_pose_cov_;

  // ---- State timeline ----
  uint64_t next_state_index_ = 0;
  gtsam::Key last_state_key_ = 0;
  double last_state_timestamp_ = 0.0;
  std::string last_state_owner_;
  bool has_last_state_ = false;
};

}  // namespace eidos

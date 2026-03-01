#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

namespace eidos {

/**
 * @brief Wrapper around GTSAM's ISAM2 incremental optimizer.
 */
class PoseGraph {
public:
  PoseGraph();

  /**
   * @brief Add factors and initial values, then optimize.
   * @param factors New factors to add.
   * @param initial_values Initial values for new variables.
   * @return Updated optimized values.
   */
  gtsam::Values update(
      const gtsam::NonlinearFactorGraph& factors,
      const gtsam::Values& initial_values);

  /**
   * @brief Run extra ISAM2 update iterations (e.g. after loop closure).
   * @param num_iterations Number of additional iterations.
   */
  void updateExtra(int num_iterations);

  /**
   * @brief Get all current optimized values.
   */
  gtsam::Values getOptimizedValues() const;

  /**
   * @brief Get the optimized pose for a given key.
   */
  gtsam::Pose3 getOptimizedPose(int index) const;

  /**
   * @brief Get the marginal covariance for a given key.
   */
  Eigen::MatrixXd getMarginalCovariance(int index) const;

  /**
   * @brief Get the total number of factors.
   */
  int numFactors() const;

  /**
   * @brief Reset the optimizer.
   */
  void reset();

private:
  std::unique_ptr<gtsam::ISAM2> isam_;
  gtsam::Values current_estimate_;
  mutable std::mutex mtx_;
};

}  // namespace eidos

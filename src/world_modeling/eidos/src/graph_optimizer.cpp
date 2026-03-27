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

#include "eidos/core/graph_optimizer.hpp"

#include <memory>
#include <string>
#include <vector>

namespace eidos
{

GraphOptimizer::GraphOptimizer()
{
  gtsam::ISAM2Params params;
  isam_ = std::make_unique<gtsam::ISAM2>(params);
}

void GraphOptimizer::configure(
  rclcpp::Logger logger, double relinearize_threshold, int relinearize_skip, const std::vector<double> & prior_pose_cov)
{
  logger_ = logger;
  relinearize_threshold_ = relinearize_threshold;
  relinearize_skip_ = relinearize_skip;
  prior_pose_cov_ = prior_pose_cov;

  gtsam::ISAM2Params params;
  params.relinearizeThreshold = relinearize_threshold_;
  params.relinearizeSkip = relinearize_skip_;
  isam_ = std::make_unique<gtsam::ISAM2>(params);
  current_estimate_ = gtsam::Values();

  RCLCPP_INFO(
    logger_,
    "\033[36m[GraphOptimizer]\033[0m Configured (relinearize_threshold=%.4f, relinearize_skip=%d)",
    relinearize_threshold,
    relinearize_skip);
}

void GraphOptimizer::reset()
{
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = relinearize_threshold_;
  params.relinearizeSkip = relinearize_skip_;
  isam_ = std::make_unique<gtsam::ISAM2>(params);
  current_estimate_ = gtsam::Values();

  next_state_index_ = 0;
  last_state_key_ = 0;
  last_state_timestamp_ = 0.0;
  last_state_owner_.clear();
  has_last_state_ = false;

  RCLCPP_INFO(logger_, "\033[36m[GraphOptimizer]\033[0m Reset (graph cleared, state index reset to 0)");
}

gtsam::Key GraphOptimizer::createState(double timestamp, const std::string & owner)
{
  gtsam::Key key = gtsam::Symbol('x', next_state_index_++);
  last_state_key_ = key;
  last_state_timestamp_ = timestamp;
  last_state_owner_ = owner;
  has_last_state_ = true;
  return key;
}

gtsam::Values GraphOptimizer::optimize(
  const gtsam::NonlinearFactorGraph & factors,
  const gtsam::Values & values,
  gtsam::Key latest_key,
  int num_iterations,
  int extra_iterations)
{
  isam_->update(factors, values);
  for (int i = 1; i < num_iterations; i++) {
    isam_->update();
  }

  if (extra_iterations > 0) {
    RCLCPP_INFO(
      logger_, "\033[36m[GraphOptimizer]\033[0m Running %d extra iterations for convergence", extra_iterations);
    for (int i = 0; i < extra_iterations; i++) {
      isam_->update();
    }
  }

  current_estimate_ = isam_->calculateEstimate();

  if (current_estimate_.exists(latest_key)) {
    optimized_pose_.store(current_estimate_.at<gtsam::Pose3>(latest_key));
  }
  optimized_values_.store(current_estimate_);

  return current_estimate_;
}

gtsam::Pose3 GraphOptimizer::getPose(gtsam::Key key) const
{
  return current_estimate_.at<gtsam::Pose3>(key);
}

gtsam::Values GraphOptimizer::getValues() const
{
  return current_estimate_;
}

std::optional<Eigen::MatrixXd> GraphOptimizer::getCovariance(gtsam::Key key) const
{
  try {
    return isam_->marginalCovariance(key);
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      logger_, "\033[36m[GraphOptimizer]\033[0m Failed to compute marginal covariance for key %lu: %s", key, e.what());
    return std::nullopt;
  } catch (...) {
    RCLCPP_WARN(logger_, "\033[36m[GraphOptimizer]\033[0m Failed to compute marginal covariance for key %lu", key);
    return std::nullopt;
  }
}

int GraphOptimizer::numFactors() const
{
  return static_cast<int>(isam_->getFactorsUnsafe().size());
}

}  // namespace eidos

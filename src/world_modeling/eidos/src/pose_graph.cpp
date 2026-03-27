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

#include "eidos/core/pose_graph.hpp"

#include <memory>

namespace eidos
{

PoseGraph::PoseGraph()
{
  reset();
}

void PoseGraph::reset(double relinearize_threshold, int relinearize_skip)
{
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = relinearize_threshold;
  params.relinearizeSkip = relinearize_skip;
  isam_ = std::make_unique<gtsam::ISAM2>(params);
  current_estimate_ = gtsam::Values();
}

gtsam::Values PoseGraph::update(
  const gtsam::NonlinearFactorGraph & factors, const gtsam::Values & initial_values, int num_iterations)
{
  isam_->update(factors, initial_values);
  for (int i = 1; i < num_iterations; i++) {
    isam_->update();
  }
  current_estimate_ = isam_->calculateEstimate();
  return current_estimate_;
}

void PoseGraph::updateExtra(int num_iterations)
{
  for (int i = 0; i < num_iterations; i++) {
    isam_->update();
  }
  current_estimate_ = isam_->calculateEstimate();
}

gtsam::Values PoseGraph::getOptimizedValues() const
{
  return current_estimate_;
}

gtsam::Pose3 PoseGraph::getOptimizedPose(gtsam::Key key) const
{
  return current_estimate_.at<gtsam::Pose3>(key);
}

Eigen::MatrixXd PoseGraph::getMarginalCovariance(gtsam::Key key) const
{
  return isam_->marginalCovariance(key);
}

int PoseGraph::numFactors() const
{
  return static_cast<int>(isam_->getFactorsUnsafe().size());
}

}  // namespace eidos

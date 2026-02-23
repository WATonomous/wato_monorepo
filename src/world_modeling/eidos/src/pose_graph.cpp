#include "eidos/pose_graph.hpp"

#include <gtsam/inference/Symbol.h>

namespace eidos {

PoseGraph::PoseGraph() {
  reset();
}

void PoseGraph::reset() {
  std::lock_guard<std::mutex> lock(mtx_);
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = 0.1;
  params.relinearizeSkip = 1;
  isam_ = std::make_unique<gtsam::ISAM2>(params);
  current_estimate_ = gtsam::Values();
}

gtsam::Values PoseGraph::update(
    const gtsam::NonlinearFactorGraph& factors,
    const gtsam::Values& initial_values) {
  std::lock_guard<std::mutex> lock(mtx_);
  isam_->update(factors, initial_values);
  isam_->update();  // Second iteration for better convergence
  current_estimate_ = isam_->calculateEstimate();
  return current_estimate_;
}

void PoseGraph::updateExtra(int num_iterations) {
  std::lock_guard<std::mutex> lock(mtx_);
  for (int i = 0; i < num_iterations; i++) {
    isam_->update();
  }
  current_estimate_ = isam_->calculateEstimate();
}

gtsam::Values PoseGraph::getOptimizedValues() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return current_estimate_;
}

gtsam::Pose3 PoseGraph::getOptimizedPose(int index) const {
  std::lock_guard<std::mutex> lock(mtx_);
  return current_estimate_.at<gtsam::Pose3>(index);
}

Eigen::MatrixXd PoseGraph::getMarginalCovariance(int index) const {
  std::lock_guard<std::mutex> lock(mtx_);
  return isam_->marginalCovariance(index);
}

int PoseGraph::numFactors() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return static_cast<int>(isam_->getFactorsUnsafe().size());
}

}  // namespace eidos

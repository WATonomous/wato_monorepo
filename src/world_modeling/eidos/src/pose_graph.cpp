#include "eidos/pose_graph.hpp"

#include <gtsam/inference/Symbol.h>

namespace eidos {

PoseGraph::PoseGraph() {
  reset();
}

void PoseGraph::reset(double relinearize_threshold, int relinearize_skip) {
  std::lock_guard<std::mutex> lock(mtx_);
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = relinearize_threshold;
  params.relinearizeSkip = relinearize_skip;
  isam_ = std::make_unique<gtsam::ISAM2>(params);
  current_estimate_ = gtsam::Values();
}

gtsam::Values PoseGraph::update(
    const gtsam::NonlinearFactorGraph& factors,
    const gtsam::Values& initial_values,
    int num_iterations) {
  std::lock_guard<std::mutex> lock(mtx_);
  isam_->update(factors, initial_values);
  for (int i = 1; i < num_iterations; i++) {
    isam_->update();
  }
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

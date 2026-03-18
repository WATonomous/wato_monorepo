#include "eidos/core/pose_graph.hpp"

namespace eidos {

PoseGraph::PoseGraph() {
  reset();
}

void PoseGraph::reset(double relinearize_threshold, int relinearize_skip) {
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
  isam_->update(factors, initial_values);
  for (int i = 1; i < num_iterations; i++) {
    isam_->update();
  }
  current_estimate_ = isam_->calculateEstimate();
  return current_estimate_;
}

void PoseGraph::updateExtra(int num_iterations) {
  for (int i = 0; i < num_iterations; i++) {
    isam_->update();
  }
  current_estimate_ = isam_->calculateEstimate();
}

gtsam::Values PoseGraph::getOptimizedValues() const {
  return current_estimate_;
}

gtsam::Pose3 PoseGraph::getOptimizedPose(gtsam::Key key) const {
  return current_estimate_.at<gtsam::Pose3>(key);
}

Eigen::MatrixXd PoseGraph::getMarginalCovariance(gtsam::Key key) const {
  return isam_->marginalCovariance(key);
}

int PoseGraph::numFactors() const {
  return static_cast<int>(isam_->getFactorsUnsafe().size());
}

}  // namespace eidos

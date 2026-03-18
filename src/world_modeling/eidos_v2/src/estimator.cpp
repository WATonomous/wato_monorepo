#include "eidos/core/estimator.hpp"

namespace eidos {

Estimator::Estimator() = default;

void Estimator::configure(double relinearize_threshold, int relinearize_skip,
                          const std::vector<double>& prior_pose_cov,
                          int update_iterations, int correction_iterations,
                          int loop_closure_iterations) {
  pose_graph_.reset(relinearize_threshold, relinearize_skip);
  prior_pose_cov_ = prior_pose_cov;
  update_iterations_ = update_iterations;
  correction_iterations_ = correction_iterations;
  loop_closure_iterations_ = loop_closure_iterations;
}

void Estimator::reset() {
  pose_graph_.reset();
  next_state_index_ = 0;
  last_state_key_ = 0;
  last_state_timestamp_ = 0.0;
  last_state_owner_.clear();
  has_last_state_ = false;
}

gtsam::Key Estimator::createState(double timestamp, const std::string& owner) {
  gtsam::Key key = gtsam::Symbol('x', next_state_index_++);
  last_state_key_ = key;
  last_state_timestamp_ = timestamp;
  last_state_owner_ = owner;
  has_last_state_ = true;
  return key;
}

gtsam::Values Estimator::optimize(
    const gtsam::NonlinearFactorGraph& factors,
    const gtsam::Values& values,
    gtsam::Key latest_key,
    int num_iterations) {
  auto result = pose_graph_.update(factors, values, num_iterations);

  if (result.exists(latest_key)) {
    optimized_pose_.store(result.at<gtsam::Pose3>(latest_key));
  }
  optimized_values_.store(result);

  return result;
}

void Estimator::optimizeExtra(int num_iterations, gtsam::Key latest_key) {
  pose_graph_.updateExtra(num_iterations);
  auto values = pose_graph_.getOptimizedValues();
  if (values.exists(latest_key)) {
    optimized_pose_.store(values.at<gtsam::Pose3>(latest_key));
  }
  optimized_values_.store(values);
}

gtsam::Pose3 Estimator::getPose(gtsam::Key key) const {
  return pose_graph_.getOptimizedPose(key);
}

gtsam::Values Estimator::getValues() const {
  return pose_graph_.getOptimizedValues();
}

std::optional<Eigen::MatrixXd> Estimator::getCovariance(gtsam::Key key) const {
  try {
    return pose_graph_.getMarginalCovariance(key);
  } catch (...) {
    return std::nullopt;
  }
}

int Estimator::numFactors() const {
  return pose_graph_.numFactors();
}

}  // namespace eidos

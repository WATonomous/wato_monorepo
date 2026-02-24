#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "eidos/plugins/base_factor_plugin.hpp"
#include "eidos/types.hpp"

namespace eidos {

/**
 * @brief Euclidean distance-based loop closure factor plugin.
 *
 * Runs a background thread that searches for loop closure candidates
 * using KD-tree radius search and validates them using ICP alignment.
 */
class EuclideanDistanceLoopClosureFactor : public FactorPlugin {
public:
  EuclideanDistanceLoopClosureFactor() = default;
  ~EuclideanDistanceLoopClosureFactor() override;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void reset() override;

  std::optional<gtsam::Pose3> processFrame(double timestamp) override;
  std::vector<gtsam::NonlinearFactor::shared_ptr> getFactors(
      int state_index, const gtsam::Pose3& state_pose, double timestamp) override;

private:
  // ---- Background thread ----
  void loopClosureThread();
  void performLoopClosure();
  bool detectLoopClosureDistance(int* latest_id, int* closest_id);
  void loopFindNearKeyframes(
      pcl::PointCloud<PointType>::Ptr& near_keyframes,
      int key, int search_num);

  // ---- Queued factors ----
  struct LoopConstraint {
    int from_index;
    int to_index;
    gtsam::Pose3 relative_pose;
    gtsam::noiseModel::Base::shared_ptr noise;
  };

  std::vector<LoopConstraint> loop_queue_;
  std::mutex loop_queue_mtx_;

  // ---- Loop closure state ----
  std::map<int, int> loop_index_container_;
  std::thread loop_thread_;
  std::atomic<bool> running_{false};
  bool active_ = false;

  // ---- Parameters (populated from ROS params in onInitialize) ----
  float frequency_;
  float search_radius_;
  float search_time_diff_;
  int search_num_;
  float fitness_score_;
  float mapping_surf_leaf_size_;
  std::string pointcloud_from_ = "lidar_kep_factor";
};

}  // namespace eidos

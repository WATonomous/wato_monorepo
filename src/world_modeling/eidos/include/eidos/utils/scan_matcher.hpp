#pragma once

#include <Eigen/Core>

#include <opencv2/opencv.hpp>

#include "eidos/types.hpp"

namespace eidos {

/**
 * @brief Edge-Plane scan-to-map matcher (LOAM-style).
 *
 * Takes corner/surface feature clouds and a local map,
 * performs iterative Levenberg-Marquardt optimization to find
 * the best alignment.
 */
class ScanMatcher {
public:
  struct Config {
    int edge_feature_min_valid = 10;
    int surf_feature_min_valid = 100;
    int max_iterations = 30;
    int num_cores = 4;
    float imu_rpy_weight = 0.01;
    float z_tolerance = 1000.0;
    float rotation_tolerance = 1000.0;

    // KNN search: max squared distance for valid nearest neighbors (m^2)
    float knn_max_sq_dist = 1.0f;
    // Eigenvalue ratio: first/second must exceed this for edge structure
    float edge_eigenvalue_ratio = 3.0f;
    // Scale factor for constructing edge line endpoints from eigenvector
    float edge_line_scale = 0.1f;
    // LOAM distance weighting: s = 1 - damping * |distance|
    float distance_damping = 0.9f;
    // Minimum weight to accept a point correspondence
    float min_correspondence_weight = 0.1f;
    // Max plane fitting residual for surface point validity (m)
    float max_plane_residual = 0.2f;
    // Degenerate eigenvalue threshold for LM optimization
    float degenerate_eigenvalue_threshold = 100.0f;
    // LM convergence: max rotation delta (degrees) and translation delta (cm)
    float convergence_rotation_deg = 0.05f;
    float convergence_translation_cm = 0.05f;
    // Minimum valid correspondences for LM optimization
    int min_correspondences = 50;
  };

  explicit ScanMatcher(const Config& config);

  /**
   * @brief Result of a scan matching operation.
   */
  struct Result {
    float transform[6] = {0};  // roll, pitch, yaw, x, y, z
    bool converged = false;
    bool degenerate = false;
    Eigen::Matrix<float, 6, 6> degenerate_projection;
  };

  /**
   * @brief Match corner/surface features against a local map.
   *
   * @param corner_cloud Downsampled corner features.
   * @param surface_cloud Downsampled surface features.
   * @param corner_map Corner map point cloud (with KD-tree).
   * @param surface_map Surface map point cloud (with KD-tree).
   * @param initial_guess Initial pose guess [roll,pitch,yaw,x,y,z].
   * @param imu_roll IMU roll for gravity alignment (optional).
   * @param imu_pitch IMU pitch for gravity alignment (optional).
   * @param use_imu_alignment Whether to use IMU for roll/pitch correction.
   * @return The optimized transform and convergence info.
   */
  Result match(
      const pcl::PointCloud<PointType>::Ptr& corner_cloud,
      const pcl::PointCloud<PointType>::Ptr& surface_cloud,
      const pcl::PointCloud<PointType>::Ptr& corner_map,
      const pcl::PointCloud<PointType>::Ptr& surface_map,
      const pcl::KdTreeFLANN<PointType>::Ptr& corner_kdtree,
      const pcl::KdTreeFLANN<PointType>::Ptr& surface_kdtree,
      const float initial_guess[6],
      float imu_roll = 0.0f,
      float imu_pitch = 0.0f,
      bool use_imu_alignment = false);

private:
  void cornerOptimization(
      const pcl::PointCloud<PointType>::Ptr& corner_cloud,
      const pcl::PointCloud<PointType>::Ptr& corner_map,
      const pcl::KdTreeFLANN<PointType>::Ptr& corner_kdtree,
      std::vector<PointType>& ori_vec,
      std::vector<PointType>& coeff_vec,
      std::vector<bool>& flag_vec);

  void surfOptimization(
      const pcl::PointCloud<PointType>::Ptr& surface_cloud,
      const pcl::PointCloud<PointType>::Ptr& surface_map,
      const pcl::KdTreeFLANN<PointType>::Ptr& surface_kdtree,
      std::vector<PointType>& ori_vec,
      std::vector<PointType>& coeff_vec,
      std::vector<bool>& flag_vec);

  bool LMOptimization(
      const pcl::PointCloud<PointType>::Ptr& ori_cloud,
      const pcl::PointCloud<PointType>::Ptr& coeff_cloud,
      float transform[6],
      int iter_count,
      bool& is_degenerate,
      Eigen::Matrix<float, 6, 6>& mat_p);

  void pointAssociateToMap(const PointType* pi, PointType* po) const;

  Config config_;
  Eigen::Affine3f trans_point_associate_to_map_;
};

}  // namespace eidos

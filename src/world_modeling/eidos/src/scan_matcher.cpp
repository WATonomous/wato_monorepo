#include "eidos/utils/scan_matcher.hpp"

#include <omp.h>

#include <pcl/common/angles.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace eidos {

ScanMatcher::ScanMatcher(const Config& config) : config_(config) {}

void ScanMatcher::pointAssociateToMap(
    const PointType* pi, PointType* po) const {
  po->x = trans_point_associate_to_map_(0, 0) * pi->x +
           trans_point_associate_to_map_(0, 1) * pi->y +
           trans_point_associate_to_map_(0, 2) * pi->z +
           trans_point_associate_to_map_(0, 3);
  po->y = trans_point_associate_to_map_(1, 0) * pi->x +
           trans_point_associate_to_map_(1, 1) * pi->y +
           trans_point_associate_to_map_(1, 2) * pi->z +
           trans_point_associate_to_map_(1, 3);
  po->z = trans_point_associate_to_map_(2, 0) * pi->x +
           trans_point_associate_to_map_(2, 1) * pi->y +
           trans_point_associate_to_map_(2, 2) * pi->z +
           trans_point_associate_to_map_(2, 3);
  po->intensity = pi->intensity;
}

ScanMatcher::Result ScanMatcher::match(
    const pcl::PointCloud<PointType>::Ptr& corner_cloud,
    const pcl::PointCloud<PointType>::Ptr& surface_cloud,
    const pcl::PointCloud<PointType>::Ptr& corner_map,
    const pcl::PointCloud<PointType>::Ptr& surface_map,
    const pcl::KdTreeFLANN<PointType>::Ptr& corner_kdtree,
    const pcl::KdTreeFLANN<PointType>::Ptr& surface_kdtree,
    const float initial_guess[6],
    float imu_roll, float imu_pitch, bool use_imu_alignment) {
  Result result;
  std::copy(initial_guess, initial_guess + 6, result.transform);

  int corner_num = static_cast<int>(corner_cloud->size());
  int surface_num = static_cast<int>(surface_cloud->size());

  if (corner_num < config_.edge_feature_min_valid ||
      surface_num < config_.surf_feature_min_valid) {
    result.converged = false;
    return result;
  }

  bool is_degenerate = false;
  Eigen::Matrix<float, 6, 6> mat_p;
  mat_p.setZero();

  // Allocate per-point buffers
  std::vector<PointType> corner_ori_vec(corner_num);
  std::vector<PointType> corner_coeff_vec(corner_num);
  std::vector<bool> corner_flag(corner_num, false);

  std::vector<PointType> surface_ori_vec(surface_num);
  std::vector<PointType> surface_coeff_vec(surface_num);
  std::vector<bool> surface_flag(surface_num, false);

  for (int iter = 0; iter < config_.max_iterations; iter++) {
    std::fill(corner_flag.begin(), corner_flag.end(), false);
    std::fill(surface_flag.begin(), surface_flag.end(), false);

    trans_point_associate_to_map_ = rpyxyzToAffine3f(result.transform);

    // Corner optimization
    #pragma omp parallel for num_threads(config_.num_cores)
    for (int i = 0; i < corner_num; i++) {
      PointType point_ori = corner_cloud->points[i];
      PointType point_sel;
      pointAssociateToMap(&point_ori, &point_sel);

      std::vector<int> search_ind;
      std::vector<float> search_sq_dis;
      corner_kdtree->nearestKSearch(point_sel, 5, search_ind, search_sq_dis);

      if (search_sq_dis[4] < config_.knn_max_sq_dist) {
        // Compute covariance matrix to check for edge structure
        float cx = 0, cy = 0, cz = 0;
        for (int j = 0; j < 5; j++) {
          cx += corner_map->points[search_ind[j]].x;
          cy += corner_map->points[search_ind[j]].y;
          cz += corner_map->points[search_ind[j]].z;
        }
        cx /= 5; cy /= 5; cz /= 5;

        cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
        float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
        for (int j = 0; j < 5; j++) {
          float ax = corner_map->points[search_ind[j]].x - cx;
          float ay = corner_map->points[search_ind[j]].y - cy;
          float az = corner_map->points[search_ind[j]].z - cz;
          a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
          a22 += ay * ay; a23 += ay * az; a33 += az * az;
        }
        a11 /= 5; a12 /= 5; a13 /= 5; a22 /= 5; a23 /= 5; a33 /= 5;

        matA1.at<float>(0, 0) = a11; matA1.at<float>(0, 1) = a12; matA1.at<float>(0, 2) = a13;
        matA1.at<float>(1, 0) = a12; matA1.at<float>(1, 1) = a22; matA1.at<float>(1, 2) = a23;
        matA1.at<float>(2, 0) = a13; matA1.at<float>(2, 1) = a23; matA1.at<float>(2, 2) = a33;

        cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));
        cv::eigen(matA1, matD1, matV1);

        if (matD1.at<float>(0, 0) > config_.edge_eigenvalue_ratio * matD1.at<float>(0, 1)) {
          float x0 = point_sel.x, y0 = point_sel.y, z0 = point_sel.z;
          float x1 = cx + config_.edge_line_scale * matV1.at<float>(0, 0);
          float y1 = cy + config_.edge_line_scale * matV1.at<float>(0, 1);
          float z1 = cz + config_.edge_line_scale * matV1.at<float>(0, 2);
          float x2 = cx - config_.edge_line_scale * matV1.at<float>(0, 0);
          float y2 = cy - config_.edge_line_scale * matV1.at<float>(0, 1);
          float z2 = cz - config_.edge_line_scale * matV1.at<float>(0, 2);

          float a012 = std::sqrt(
              ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) *
                  ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
              ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) *
                  ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
              ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) *
                  ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

          float l12 = std::sqrt(
              (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) +
              (z1 - z2) * (z1 - z2));

          if (a012 < 1e-8f || l12 < 1e-8f) continue;

          float la = ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
                       (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) /
                      a012 / l12;
          float lb = -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) -
                        (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
                      a012 / l12;
          float lc = -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
                        (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
                      a012 / l12;

          float ld2 = a012 / l12;
          float s = 1 - config_.distance_damping * std::abs(ld2);

          PointType coeff;
          coeff.x = s * la;
          coeff.y = s * lb;
          coeff.z = s * lc;
          coeff.intensity = s * ld2;

          if (s > config_.min_correspondence_weight) {
            corner_ori_vec[i] = point_ori;
            corner_coeff_vec[i] = coeff;
            corner_flag[i] = true;
          }
        }
      }
    }

    // Surface optimization
    #pragma omp parallel for num_threads(config_.num_cores)
    for (int i = 0; i < surface_num; i++) {
      PointType point_ori = surface_cloud->points[i];
      PointType point_sel;
      pointAssociateToMap(&point_ori, &point_sel);

      std::vector<int> search_ind;
      std::vector<float> search_sq_dis;
      surface_kdtree->nearestKSearch(point_sel, 5, search_ind, search_sq_dis);

      if (search_sq_dis[4] < config_.knn_max_sq_dist) {
        Eigen::Matrix<float, 5, 3> matA0;
        Eigen::Matrix<float, 5, 1> matB0;
        matA0.setZero();
        matB0.fill(-1);

        for (int j = 0; j < 5; j++) {
          matA0(j, 0) = surface_map->points[search_ind[j]].x;
          matA0(j, 1) = surface_map->points[search_ind[j]].y;
          matA0(j, 2) = surface_map->points[search_ind[j]].z;
        }

        Eigen::Vector3f matX0 = matA0.colPivHouseholderQr().solve(matB0);
        float pa = matX0(0), pb = matX0(1), pc = matX0(2), pd = 1;
        float ps = std::sqrt(pa * pa + pb * pb + pc * pc);
        if (ps < 1e-8f) continue;
        pa /= ps; pb /= ps; pc /= ps; pd /= ps;

        bool plane_valid = true;
        for (int j = 0; j < 5; j++) {
          if (std::abs(pa * surface_map->points[search_ind[j]].x +
                       pb * surface_map->points[search_ind[j]].y +
                       pc * surface_map->points[search_ind[j]].z + pd) > config_.max_plane_residual) {
            plane_valid = false;
            break;
          }
        }

        if (plane_valid) {
          float pd2 = pa * point_sel.x + pb * point_sel.y + pc * point_sel.z + pd;
          float s = 1 - config_.distance_damping * std::abs(pd2) /
                    std::sqrt(std::sqrt(point_ori.x * point_ori.x +
                                        point_ori.y * point_ori.y +
                                        point_ori.z * point_ori.z));

          PointType coeff;
          coeff.x = s * pa;
          coeff.y = s * pb;
          coeff.z = s * pc;
          coeff.intensity = s * pd2;

          if (s > config_.min_correspondence_weight) {
            surface_ori_vec[i] = point_ori;
            surface_coeff_vec[i] = coeff;
            surface_flag[i] = true;
          }
        }
      }
    }

    // Combine coefficients
    auto laser_cloud_ori = pcl::make_shared<pcl::PointCloud<PointType>>();
    auto coeff_sel = pcl::make_shared<pcl::PointCloud<PointType>>();

    for (int i = 0; i < corner_num; i++) {
      if (corner_flag[i]) {
        laser_cloud_ori->push_back(corner_ori_vec[i]);
        coeff_sel->push_back(corner_coeff_vec[i]);
      }
    }
    for (int i = 0; i < surface_num; i++) {
      if (surface_flag[i]) {
        laser_cloud_ori->push_back(surface_ori_vec[i]);
        coeff_sel->push_back(surface_coeff_vec[i]);
      }
    }

    // LM optimization step
    if (LMOptimization(laser_cloud_ori, coeff_sel,
                        result.transform, iter, is_degenerate, mat_p)) {
      result.converged = true;
      break;
    }
  }

  // Apply IMU alignment for roll/pitch if requested
  if (use_imu_alignment && std::abs(imu_pitch) < 1.4f) {
    float imu_weight = config_.imu_rpy_weight;
    tf2::Quaternion imu_q, transform_q;
    double mid_roll, mid_pitch, mid_yaw;

    transform_q.setRPY(result.transform[0], 0, 0);
    imu_q.setRPY(imu_roll, 0, 0);
    tf2::Matrix3x3(transform_q.slerp(imu_q, imu_weight))
        .getRPY(mid_roll, mid_pitch, mid_yaw);
    result.transform[0] = static_cast<float>(mid_roll);

    transform_q.setRPY(0, result.transform[1], 0);
    imu_q.setRPY(0, imu_pitch, 0);
    tf2::Matrix3x3(transform_q.slerp(imu_q, imu_weight))
        .getRPY(mid_roll, mid_pitch, mid_yaw);
    result.transform[1] = static_cast<float>(mid_pitch);
  }

  // Apply constraints
  result.transform[0] = std::clamp(result.transform[0],
                                    -config_.rotation_tolerance,
                                    config_.rotation_tolerance);
  result.transform[1] = std::clamp(result.transform[1],
                                    -config_.rotation_tolerance,
                                    config_.rotation_tolerance);
  result.transform[5] = std::clamp(result.transform[5],
                                    -config_.z_tolerance,
                                    config_.z_tolerance);

  result.degenerate = is_degenerate;
  result.degenerate_projection = mat_p;
  return result;
}

bool ScanMatcher::LMOptimization(
    const pcl::PointCloud<PointType>::Ptr& ori_cloud,
    const pcl::PointCloud<PointType>::Ptr& coeff_cloud,
    float transform[6],
    int iter_count,
    bool& is_degenerate,
    Eigen::Matrix<float, 6, 6>& mat_p) {
  // Coordinate transformation: lidar <-> camera convention from LOAM
  float srx = std::sin(transform[1]);
  float crx = std::cos(transform[1]);
  float sry = std::sin(transform[2]);
  float cry = std::cos(transform[2]);
  float srz = std::sin(transform[0]);
  float crz = std::cos(transform[0]);

  int sel_num = static_cast<int>(ori_cloud->size());
  if (sel_num < config_.min_correspondences) return false;

  cv::Mat matA(sel_num, 6, CV_32F, cv::Scalar::all(0));
  cv::Mat matAt(6, sel_num, CV_32F, cv::Scalar::all(0));
  cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
  cv::Mat matB(sel_num, 1, CV_32F, cv::Scalar::all(0));
  cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
  cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
  cv::Mat matP_cv(6, 6, CV_32F, cv::Scalar::all(0));

  for (int i = 0; i < sel_num; i++) {
    // lidar -> camera
    PointType point_ori;
    point_ori.x = ori_cloud->points[i].y;
    point_ori.y = ori_cloud->points[i].z;
    point_ori.z = ori_cloud->points[i].x;

    PointType coeff;
    coeff.x = coeff_cloud->points[i].y;
    coeff.y = coeff_cloud->points[i].z;
    coeff.z = coeff_cloud->points[i].x;
    coeff.intensity = coeff_cloud->points[i].intensity;

    float arx = (crx * sry * srz * point_ori.x +
                  crx * crz * sry * point_ori.y -
                  srx * sry * point_ori.z) * coeff.x +
                (-srx * srz * point_ori.x -
                  crz * srx * point_ori.y -
                  crx * point_ori.z) * coeff.y +
                (crx * cry * srz * point_ori.x +
                  crx * cry * crz * point_ori.y -
                  cry * srx * point_ori.z) * coeff.z;

    float ary = ((cry * srx * srz - crz * sry) * point_ori.x +
                  (sry * srz + cry * crz * srx) * point_ori.y +
                  crx * cry * point_ori.z) * coeff.x +
                ((-cry * crz - srx * sry * srz) * point_ori.x +
                  (cry * srz - crz * srx * sry) * point_ori.y -
                  crx * sry * point_ori.z) * coeff.z;

    float arz = ((crz * srx * sry - cry * srz) * point_ori.x +
                  (-cry * crz - srx * sry * srz) * point_ori.y) * coeff.x +
                (crx * crz * point_ori.x -
                  crx * srz * point_ori.y) * coeff.y +
                ((sry * srz + cry * crz * srx) * point_ori.x +
                  (crz * sry - cry * srx * srz) * point_ori.y) * coeff.z;

    matA.at<float>(i, 0) = arz;
    matA.at<float>(i, 1) = arx;
    matA.at<float>(i, 2) = ary;
    matA.at<float>(i, 3) = coeff.z;
    matA.at<float>(i, 4) = coeff.x;
    matA.at<float>(i, 5) = coeff.y;
    matB.at<float>(i, 0) = -coeff.intensity;
  }

  cv::transpose(matA, matAt);
  matAtA = matAt * matA;
  matAtB = matAt * matB;
  cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

  if (iter_count == 0) {
    cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));
    cv::eigen(matAtA, matE, matV);
    matV.copyTo(matV2);

    is_degenerate = false;
    float eigen_thre[6] = {
        config_.degenerate_eigenvalue_threshold, config_.degenerate_eigenvalue_threshold,
        config_.degenerate_eigenvalue_threshold, config_.degenerate_eigenvalue_threshold,
        config_.degenerate_eigenvalue_threshold, config_.degenerate_eigenvalue_threshold};
    for (int i = 5; i >= 0; i--) {
      if (matE.at<float>(0, i) < eigen_thre[i]) {
        for (int j = 0; j < 6; j++) matV2.at<float>(i, j) = 0;
        is_degenerate = true;
      } else {
        break;
      }
    }
    matP_cv = matV.inv() * matV2;

    // Copy to Eigen matrix
    for (int r = 0; r < 6; r++)
      for (int c = 0; c < 6; c++)
        mat_p(r, c) = matP_cv.at<float>(r, c);
  }

  if (is_degenerate) {
    cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
    matX.copyTo(matX2);
    matX = matP_cv * matX2;
  }

  transform[0] += matX.at<float>(0, 0);
  transform[1] += matX.at<float>(1, 0);
  transform[2] += matX.at<float>(2, 0);
  transform[3] += matX.at<float>(3, 0);
  transform[4] += matX.at<float>(4, 0);
  transform[5] += matX.at<float>(5, 0);

  // Convergence check: rotation delta in degrees, translation delta in centimeters
  float delta_r = std::sqrt(
      std::pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
      std::pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
      std::pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
  float delta_t = std::sqrt(
      std::pow(matX.at<float>(3, 0) * 100.0f, 2) +
      std::pow(matX.at<float>(4, 0) * 100.0f, 2) +
      std::pow(matX.at<float>(5, 0) * 100.0f, 2));

  return (delta_r < config_.convergence_rotation_deg &&
          delta_t < config_.convergence_translation_cm);
}

}  // namespace eidos

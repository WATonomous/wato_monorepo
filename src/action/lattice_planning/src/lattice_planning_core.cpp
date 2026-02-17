#include <iostream>
#include <cmath>

#include "lattice_planning/lattice_planning_core.hpp"

LatticePlanningCore::LatticePlanningCore(const SpiralCoeffConstants& constants, const PathGenParams& params)
  : spiral_constants_(constants), pg_params_(params) {
}

/**
 * @brief Calculate Euclidean distance between two 2D points.
 * @param x1 X-coordinate of first point.
 * @param y1 Y-coordinate of first point.
 * @param x2 X-coordinate of second point.
 * @param y2 Y-coordinate of second point.
 * @return Euclidean distance.
 */
double LatticePlanningCore::get_euc_dist(double x1, double y1, double x2, double y2){
  return std::hypot(x2 - x1, y2 - y1);
}

/**
 * @brief Calculate angle from first point to second point.
 * @param x1 X-coordinate of first point.
 * @param y1 Y-coordinate of first point.
 * @param x2 X-coordinate of second point.
 * @param y2 Y-coordinate of second point.
 * @return Angle in radians.
 */
double LatticePlanningCore::get_angle_from_pts(double x1, double y1, double x2, double y2){
  return std::atan2(y2 - y1, x2 - x1);
}

/**
 * @brief Normalize angle to [-pi, pi].
 * @param angle Input angle in radians.
 * @return Normalized angle.
 */
double LatticePlanningCore::normalise_angle(double angle){
    return std::remainder(angle, 2.0 * M_PI);
}

/**
 * @brief Calculate cubic spiral polynomial coefficients from boundary conditions.
 * 
 * Computes coefficients for k(s) = c0 + c1*s + c2*s^2 + c3*s^3 using a cubic interpolation scheme
 * that ensures smooth curvature transitions between waypoints.
 * 
 * @param params Map of spiral parameters (K0, K1, K2, K3, SF).
 * @param coeffs Output map of cubic polynomial coefficients (C0, C1, C2, C3).
 */
void LatticePlanningCore::calculate_spiral_coeff(
    const std::unordered_map<SpiralParam, double>& params, 
    std::unordered_map<SpiralCoeff, double>& coeffs)
{
  const double k0 = params.at(SpiralParam::K0);
  const double k1 = params.at(SpiralParam::K1);
  const double k2 = params.at(SpiralParam::K2);
  const double k3 = params.at(SpiralParam::K3);
  const double sf = params.at(SpiralParam::SF);
  
  // C0 = k0
  coeffs[SpiralCoeff::C0] = k0;
  
  // C1 = (c1_k0_coeff * k0 + c1_k1_coeff * k1 + c1_k2_coeff * k2 + c1_k3_coeff * k3) / (c1_divisor * sf)
  coeffs[SpiralCoeff::C1] = (
    spiral_constants_.c1_k0_coeff * k0 + 
    spiral_constants_.c1_k1_coeff * k1 + 
    spiral_constants_.c1_k2_coeff * k2 + 
    spiral_constants_.c1_k3_coeff * k3
  ) / (spiral_constants_.c1_divisor * sf);
  
  // C2 = (c2_k0_coeff * k0 + c2_k1_coeff * k1 + c2_k2_coeff * k2 + c2_k3_coeff * k3) / (c2_divisor * sf^2)
  coeffs[SpiralCoeff::C2] = (
    spiral_constants_.c2_k0_coeff * k0 + 
    spiral_constants_.c2_k1_coeff * k1 + 
    spiral_constants_.c2_k2_coeff * k2 + 
    spiral_constants_.c2_k3_coeff * k3
  ) / (spiral_constants_.c2_divisor * sf * sf);
  
  // C3 = (c3_k0_coeff * k0 + c3_k1_coeff * k1 + c3_k2_coeff * k2 + c3_k3_coeff * k3) / (c3_divisor * sf^3)
  coeffs[SpiralCoeff::C3] = (
    spiral_constants_.c3_k0_coeff * k0 + 
    spiral_constants_.c3_k1_coeff * k1 + 
    spiral_constants_.c3_k2_coeff * k2 + 
    spiral_constants_.c3_k3_coeff * k3
  ) / (spiral_constants_.c3_divisor * sf * sf * sf);
}

/**
 * @brief Select path with lowest cost from candidate set.
 * @param paths Vector of candidate paths.
 * @param preferred_lanelets Map of preferred lanelet IDs.
 * @param cf_params Cost function parameters.
 * @return Path with minimum cost.
 */
Path LatticePlanningCore::get_lowest_cost_path(
  const std::vector<Path> & paths, 
  const std::unordered_map<int64_t, int> & preferred_lanelets, 
  const CostFunctionParams & cf_params
){
  Path lowest_cost_path = paths[0];
  double prev_cost = path_cost_function(paths[0], preferred_lanelets.count(paths[0].target_lanelet_id) >= 1, cf_params);

  for(size_t i = 1; i < paths.size(); i++){
    bool preferred_lane = preferred_lanelets.count(paths[i].target_lanelet_id) >= 1;
    double path_cost = path_cost_function(paths[i], preferred_lane, cf_params);

    if(path_cost < prev_cost){
      lowest_cost_path = paths[i];
      prev_cost = path_cost;
    }
  }
  return lowest_cost_path;
}

/**
 * @brief Compute cost of a path based on curvature changes, lateral movement, and lane preference.
 * @param path Path to evaluate.
 * @param preferred_lane Whether the path is in a preferred lane.
 * @param params Cost function weights and thresholds.
 * @return Total path cost.
 */
double LatticePlanningCore::path_cost_function(    
  const Path & path,
  bool preferred_lane,
  CostFunctionParams params
){
  double path_cost = 0.0;
  double prev_kappa = std::numeric_limits<double>::quiet_NaN();

  if(!preferred_lane){
    path_cost += params.preferred_lane_cost;
  }

  for(const auto & pt: path.path){
    if(!std::isnan(prev_kappa)){
      double curvature_change = fabs(pt.kappa - prev_kappa);
      if(curvature_change > params.max_curvature_change){
        path_cost += params.physical_limits_weight * (curvature_change - params.max_curvature_change);
      }
    }

    path_cost += params.lateral_movement_weight * fabs(pt.kappa);  
    
    prev_kappa = pt.kappa;
  }

  return path_cost;
}

/**
 * @brief Generate discrete path points along a cubic spiral by integrating curvature.
 * 
 * Uses forward Euler integration to compute (x, y, theta, kappa) at each step along the arc length,
 * where curvature evolves according to k(s) = c0 + c1*s + c2*s^2 + c3*s^3.
 * 
 * @param start Starting path point (x, y, theta, kappa).
 * @param steps Number of discrete integration steps.
 * @param sf Total arc length.
 * @param coeffs Map of spiral polynomial coefficients (C0, C1, C2, C3).
 * @param path Output vector of path points.
 */
void LatticePlanningCore::generate_spiral(
  PathPoint start, 
  int steps, 
  double sf, 
  const std::unordered_map<SpiralCoeff, double>& coeffs, 
  std::vector<PathPoint>& path)
{
  const double c0 = coeffs.at(SpiralCoeff::C0);
  const double c1 = coeffs.at(SpiralCoeff::C1);
  const double c2 = coeffs.at(SpiralCoeff::C2);
  const double c3 = coeffs.at(SpiralCoeff::C3);
  
  double ds = sf / steps;
  double x = start.x, y = start.y, theta = start.theta;

  path.push_back(start);

  for (int i = 1; i <= steps; ++i) {
      double s = i * ds;
      double kappa = c0 + c1 * s + c2 * s * s + c3 * s * s * s;
      theta += kappa * ds;
      x += cos(theta) * ds;
      y += sin(theta) * ds;
      PathPoint point = {x, y, theta, kappa};
      path.push_back(point);
  }
}

/**
 * @brief Compute 3-DOF error vector (x, y, theta) between actual and target path points.
 * @param actual Achieved path point.
 * @param target Desired path point.
 * @return Error vector [dx, dy, dtheta].
 */
Eigen::Vector3d LatticePlanningCore::compute_error_3dof(
  const PathPoint& actual, 
  const PathPoint& target)
{
  Eigen::Vector3d error;
  error << actual.x - target.x,
           actual.y - target.y,
           normalise_angle(actual.theta - target.theta);
  return error;
}

/**
 * @brief Generate smooth path between start and target using damped Newton optimization on cubic spiral parameters.
 * 
 * Fits a cubic spiral k(s) = c0 + c1*s + c2*s^2 + c3*s^3 connecting start to target by iteratively refining
 * three parameters: intermediate curvatures k1, k2 (at 1/3 and 2/3 arc length), and total arc length sf.
 * Uses damped Newton's method with finite-difference Jacobian to minimize 3-DOF pose error (x, y, theta).
 * Curvatures are clamped to vehicle physical limits to ensure feasibility.
 * 
 * @param start Initial path point (position, heading, curvature).
 * @param target Target path point to reach.
 * @param pg_params_ Parameters controlling optimization (max iterations, tolerance, damping, step limits).
 * @return Vector of path points forming the spiral, or empty vector if optimization fails to converge.
 */
std::vector<PathPoint> LatticePlanningCore::generate_path(
  PathPoint start, 
  PathPoint target)
{
  std::vector<PathPoint> path;
  std::unordered_map<SpiralCoeff, double> spiral_coeffs;
  std::unordered_map<SpiralParam, double> spiral_params;

  double norm = std::numeric_limits<double>::infinity();
  double sf_est = get_euc_dist(start.x, start.y, target.x, target.y);

  // Initialize spiral parameters
  spiral_params[SpiralParam::K0] = start.kappa;
  spiral_params[SpiralParam::K1] = start.kappa + (target.kappa - start.kappa) * (1.0 / 3.0);
  spiral_params[SpiralParam::K2] = start.kappa + (target.kappa - start.kappa) * (2.0 / 3.0);
  spiral_params[SpiralParam::K3] = target.kappa;
  spiral_params[SpiralParam::SF] = sf_est;
    
  double max_kappa = std::max({std::abs(start.kappa), std::abs(target.kappa), 0.5});

  for(int i = 0; i < pg_params_.max_iterations; i++){
      path.clear();

      calculate_spiral_coeff(spiral_params, spiral_coeffs);
      generate_spiral(start, pg_params_.steps, spiral_params[SpiralParam::SF], spiral_coeffs, path);

      PathPoint final_state = path.back();

      Eigen::Vector3d error = compute_error_3dof(final_state, target);
      norm = error.norm();

      if (norm < pg_params_.tolerance) break;

      Eigen::Vector3d params(
        spiral_params[SpiralParam::K1], 
        spiral_params[SpiralParam::K2], 
        spiral_params[SpiralParam::SF]
      );
      Eigen::Matrix3d J = compute_jacobian_3dof(params, error, start, target, pg_params_.steps);
      
      double det = J.determinant();
      if (std::abs(det) < 1e-10) {
        std::cerr << "Singular Jacobian (det=" << det << ") at iter " << i << "\n";
        path.clear();
        return path;
      }
      
      Eigen::Vector3d delta = J.inverse() * error;
      delta *= pg_params_.newton_damping;
      
      if (delta.norm() > pg_params_.max_step_size) {
        delta = delta.normalized() * pg_params_.max_step_size;
      }

      params -= delta;
      spiral_params[SpiralParam::K1] = params(0);
      spiral_params[SpiralParam::K2] = params(1);
      spiral_params[SpiralParam::SF] = std::max(0.5, params(2));
      
      spiral_params[SpiralParam::K1] = std::clamp(spiral_params[SpiralParam::K1], -max_kappa, max_kappa);
      spiral_params[SpiralParam::K2] = std::clamp(spiral_params[SpiralParam::K2], -max_kappa, max_kappa);
  }
  
  if (norm > pg_params_.tolerance){ 
      std::cerr << "Failed to converge. Final error: " << norm << "\n";
      path.clear();
  }

  return path;
}

/**
 * @brief Compute 3x3 Jacobian matrix d(x,y,theta)/d(k1,k2,sf) using finite differences.
 * 
 * Evaluates sensitivity of final pose error to spiral parameters by perturbing each parameter
 * and measuring the resulting change in endpoint position and heading.
 * 
 * @param p Current parameter vector [k1, k2, sf].
 * @param error Current 3-DOF error vector.
 * @param start Starting path point.
 * @param target Target path point.
 * @param steps Number of spiral discretization steps.
 * @return 3x3 Jacobian matrix for Newton optimization.
 */
Eigen::Matrix3d LatticePlanningCore::compute_jacobian_3dof(
  const Eigen::Vector3d& p, 
  const Eigen::Vector3d& error,
  const PathPoint& start,
  const PathPoint& target,
  int steps) 
{
  Eigen::Matrix3d jacobian = Eigen::Matrix3d::Zero();
  const double delta = 1e-6;
  
  for (int j = 0; j < 3; ++j) {
      Eigen::Vector3d p_perturbed = p;
      p_perturbed(j) += delta;
      
      std::vector<PathPoint> temp_path;
      std::unordered_map<SpiralCoeff, double> spiral_coeffs;
      std::unordered_map<SpiralParam, double> spiral_params;
      
      spiral_params[SpiralParam::K0] = start.kappa;
      spiral_params[SpiralParam::K1] = p_perturbed(0);
      spiral_params[SpiralParam::K2] = p_perturbed(1);
      spiral_params[SpiralParam::K3] = target.kappa;
      spiral_params[SpiralParam::SF] = p_perturbed(2);

      calculate_spiral_coeff(spiral_params, spiral_coeffs);
      generate_spiral(start, steps, p_perturbed(2), spiral_coeffs, temp_path);

      PathPoint final_state = temp_path.back();
      Eigen::Vector3d error_perturbed = compute_error_3dof(final_state, target);
      
      jacobian.col(j) = (error_perturbed - error) / delta;
  }
  
  return jacobian;
}
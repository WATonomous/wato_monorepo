#include <iostream>
#include <cmath>

#include "local_planning/local_planner_core.hpp"


LocalPlannerCore::LocalPlannerCore() = default;

double LocalPlannerCore::get_euc_dist(double x1, double y1, double x2, double y2){
  return std::hypot(x2 - x1, y2 - y1);
}

double LocalPlannerCore::get_angle_from_pts(double x1, double y1, double x2, double y2){
  return std::atan2(y2 - y1, x2 - x1);
}

double LocalPlannerCore::normalise_angle(double angle){
    return std::remainder(angle, 2.0 * M_PI);
}

void LocalPlannerCore::calculate_spiral_coeff(const double p[5], double (&coeffs)[4]){
  coeffs[0] = p[0];
  coeffs[1] = (-11 * p[0] + 18 * p[1] - 9 * p[2] + 2 * p[3])  / (2 * p[4]);
  coeffs[2] = (9 * (2 * p[0] - 5 * p[1] + 4 * p[2] - p[3]))   / (2 * p[4] * p[4]);
  coeffs[3] = (-9 * (p[0] - 3 * p[1] + 3 * p[2] - p[3]))      / (2 * p[4] * p[4] * p[4]);
}

void LocalPlannerCore::generate_spiral(
  PathPoint start, 
  int steps, 
  double sf, 
  double c[4], 
  std::vector<PathPoint>& path)
{
  double ds = sf / steps;
  double x = start.x, y = start.y, theta = start.theta;

  path.push_back(start);

  for (int i = 1; i <= steps; ++i) {
      double s = i * ds;
      double kappa = c[0] + c[1] * s + c[2] * s * s + c[3] * s * s * s;
      theta += kappa * ds;
      x += cos(theta) * ds;
      y += sin(theta) * ds;
      PathPoint point = {x,y,theta,kappa};
      path.push_back(point);
  }
}


std::vector<PathPoint> LocalPlannerCore::generate_path(
  PathPoint start, 
  PathPoint target,
  PathGenParams pg_params)
{
  std::vector<PathPoint> path;

  double norm = std::numeric_limits<double>::infinity();
  double spiral_coeffs[4];
  double spiral_params[5];

  // Initial arc length estimate
  double sf_est = get_euc_dist(start.x, start.y, target.x, target.y);
  

  // Set intial spiral parameters
  spiral_params[0] = start.kappa;
  spiral_params[1] = start.kappa + (target.kappa - start.kappa) * 0.33;  // 1/3 point
  spiral_params[2] = start.kappa + (target.kappa - start.kappa) * 0.67;  // 2/3 point
  spiral_params[3] = target.kappa;
  spiral_params[4] = sf_est;
    
  double max_kappa = std::max({std::abs(start.kappa), std::abs(target.kappa), 0.5});

  for(int i = 0; i < pg_params.max_iterations; i++){
      path.clear();

      calculate_spiral_coeff(spiral_params, spiral_coeffs);
      generate_spiral(start, pg_params.steps, spiral_params[4], spiral_coeffs, path);

      PathPoint final_state = path.back();

      Eigen::Vector3d error;
      error << final_state.x - target.x,
               final_state.y - target.y,
               normalise_angle(final_state.theta - target.theta);
      
      norm = error.norm();

      if (norm < pg_params.tolerance) break;

      Eigen::Vector3d params(spiral_params[1], spiral_params[2], spiral_params[4]);
      
      Eigen::Matrix3d J = compute_jacobian_3dof(params, error, start, target, pg_params.steps);
      
      double det = J.determinant();
      if (std::abs(det) < 1e-10) {
        std::cerr << "Singular Jacobian (det=" << det << ") at iter " << i << "\n";
        path.clear();
        return path;
      }
      
      Eigen::Vector3d delta = J.inverse() * error;
      
      // Damped Newton step (prevent divergence)
      delta *= pg_params.newton_damping;
      
      // Additional step limiting
      if (delta.norm() > pg_params.max_step_size) {
        delta = delta.normalized() * 1.0;
      }

      params -= delta;
      spiral_params[1] = params(0);
      spiral_params[2] = params(1);
      spiral_params[4] = std::max(0.5, params(2)); // Keep sf positive
      
      // Clamp curvatures to reasonable values
      spiral_params[1] = std::clamp(spiral_params[1], -max_kappa, max_kappa);
      spiral_params[2] = std::clamp(spiral_params[2], -max_kappa, max_kappa);
  }
  
  if (norm > pg_params.tolerance){ 
      std::cerr << "Failed to converge. Final error: " << norm << "\n";
      path.clear();
  }

  return path;
}


Eigen::Vector4d LocalPlannerCore::compute_error(const PathPoint actual, const PathPoint target) {
  Eigen::Vector4d error;
  error << actual.x - target.x,
            actual.y - target.y,
            actual.theta - target.theta,
            actual.kappa - target.kappa;
  return error;
}


Eigen::Matrix3d LocalPlannerCore::compute_jacobian_3dof(
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
      double spiral_coeffs[4];
      double spiral_params[5] = {start.kappa, p_perturbed(0), p_perturbed(1), 
                                  target.kappa, p_perturbed(2)};

      calculate_spiral_coeff(spiral_params, spiral_coeffs);
      generate_spiral(start, steps, p_perturbed(2), spiral_coeffs, temp_path);

      PathPoint final_state = temp_path.back();
      
      Eigen::Vector3d error_perturbed;
      error_perturbed << final_state.x - target.x,
                        final_state.y - target.y,
                        normalise_angle(final_state.theta - target.theta);
      
      jacobian.col(j) = (error_perturbed - error) / delta;
  }
  
  return jacobian;
}

// Eigen::Matrix4d LocalPlannerCore::compute_jacobian(
//   const Eigen::Vector3d& p, 
//   const Eigen::Vector4d& error,
//   const PathPoint& start,
//   const PathPoint& target,
//   int steps) 
// {
//   Eigen::Matrix4d jacobian = Eigen::Matrix4d::Zero();
//   const double delta = 1e-6;
  
//   for (int j = 0; j < 3; ++j) {
//       // Perturb parameter j
//       Eigen::Vector3d p_perturbed = p;
//       p_perturbed(j) += delta;
      
//       // Generate spiral with perturbed parameters
//       std::vector<PathPoint> temp_path;
//       double spiral_coeffs[4];
//       double spiral_params[5] = {start.kappa, p_perturbed(0), p_perturbed(1), target.kappa, p_perturbed(2)};

//       calculate_spiral_coeff(spiral_params, spiral_coeffs);
//       generate_spiral(start, steps, p_perturbed(2), spiral_coeffs, temp_path);

//       PathPoint final_state = temp_path.back();
      
//       // Compute error with perturbed parameters
//       Eigen::Vector4d error_perturbed = compute_error(final_state, target);
      
//       // Finite difference
//       jacobian.col(j) = (error_perturbed - error) / delta;
//   }
  
//   return jacobian;
// }
#include <iostream>
#include <cmath>

#include "local_planning/local_planner_core.hpp"


LocalPlannerCore::LocalPlannerCore() = default;

double LocalPlannerCore::get_euc_dist(double x1, double y1, double x2, double y2){
  return std::hypot(x2 - x1, y2 - y1);
}

void LocalPlannerCore::calculate_spiral_coeff(const double p[5], double (&coeffs)[4]){
  coeffs[0] = p[0];
  coeffs[1] = (-11 * p[0] + 18 * p[1] - 9 * p[2] + 2 * p[3])  / (2 * p[4]);
  coeffs[2] = (9 * (2 * p[0] - 5 * p[1] + 4 * p[2] - p[3]))   / (2 * p[4] * p[4]);
  coeffs[3] = (-9 * (p[0] - 3 * p[1] + 3 * p[2] - p[3]))      / (2 * p[4] * p[4] * p[4]);
}

void LocalPlannerCore::generate_spiral(
  FrenetPoint start, 
  int steps, 
  double sf, 
  double c[4], 
  std::vector<FrenetPoint>& path)
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
      FrenetPoint point = {x,y,theta,kappa};
      path.push_back(point);
  }
}


std::vector<FrenetPoint> LocalPlannerCore::generate_path(
  FrenetPoint start, 
  FrenetPoint target,
  const int max_iterations,
  const int steps,
  const double tolerance)
{
  std::vector<FrenetPoint> path;

  double norm = std::numeric_limits<double>::infinity();
  double spiral_coeffs[4];
  double spiral_params[5];

  spiral_params[0] = start.kappa;
  spiral_params[1] = (target.kappa + start.kappa)/2;
  spiral_params[2] = spiral_params[1];
  spiral_params[3] = target.kappa;
  spiral_params[4] = get_euc_dist(start.x, start.y, target.x, target.y);
    
  for(int i = 0; i < max_iterations; i++){
      path.clear();

      calculate_spiral_coeff(spiral_params, spiral_coeffs);
      generate_spiral(start, steps, spiral_params[4], spiral_coeffs, path);

      FrenetPoint final_state = path.back();

      Eigen::Vector4d error = compute_error(final_state, target);
      norm = error.norm();

      if (norm < tolerance) break;

      Eigen::Vector3d params(spiral_params[1],spiral_params[2], spiral_params[4]);
      Eigen::Matrix4d J = compute_jacobian(params, error, start, target, steps);
      Eigen::Vector3d delta = J.topLeftCorner(3, 3).inverse() * error.head(3);

      params -= delta;
      spiral_params[1] = params(0);
      spiral_params[2] = params(1);
      spiral_params[4] = params(2);
  }
  if (norm > tolerance){ 
      std::cerr << "Newton's method did not converge after 20 iterations. Final error: " << norm << "\n";
      path.clear();
  }

  return path;
}

Eigen::Vector4d LocalPlannerCore::compute_error(const FrenetPoint actual, const FrenetPoint target) {
  Eigen::Vector4d error;
  error << actual.x - target.x,
            actual.y - target.y,
            actual.theta - target.theta,
            actual.kappa - target.kappa;
  return error;
}

Eigen::Matrix4d LocalPlannerCore::compute_jacobian(
  const Eigen::Vector3d& p, 
  const Eigen::Vector4d& error,
  const FrenetPoint& start,
  const FrenetPoint& target,
  int steps) 
{
  Eigen::Matrix4d jacobian = Eigen::Matrix4d::Zero();
  const double delta = 1e-6;
  
  for (int j = 0; j < 3; ++j) {
      // Perturb parameter j
      Eigen::Vector3d p_perturbed = p;
      p_perturbed(j) += delta;
      
      // Generate spiral with perturbed parameters
      std::vector<FrenetPoint> temp_path;
      double spiral_coeffs[4];
      double spiral_params[5] = {start.kappa, p_perturbed(0), p_perturbed(1), target.kappa, p_perturbed(2)};

      calculate_spiral_coeff(spiral_params, spiral_coeffs);
      generate_spiral(start, steps, p_perturbed(2), spiral_coeffs, temp_path);

      FrenetPoint final_state = temp_path.back();
      
      // Compute error with perturbed parameters
      Eigen::Vector4d error_perturbed = compute_error(final_state, target);
      
      // Finite difference
      jacobian.col(j) = (error_perturbed - error) / delta;
  }
  
  return jacobian;
}
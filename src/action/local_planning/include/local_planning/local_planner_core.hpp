#pragma once

#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>


struct FrenetPoint{
  double x;
  double y;
  double theta;
  double kappa;
};

class LocalPlannerCore
{
public:

  LocalPlannerCore();

  // ultily functions
  double get_euc_dist(double x1, double y1, double x2, double y2);

  // path generation
  std::vector<FrenetPoint> generate_path(
    FrenetPoint start, 
    FrenetPoint target,
    const int max_iterations = 20,
    const int steps = 20,
    const double tolerance = 0.25
  );
  
  void calculate_spiral_coeff(const double p[5], double (&coeffs)[4]);
  void generate_spiral(FrenetPoint start, int steps, double sf, double coeffs[4], std::vector<FrenetPoint>& path);

  Eigen::Vector4d compute_error(const FrenetPoint actual, const FrenetPoint target);
  Eigen::Matrix4d compute_jacobian(
    const Eigen::Vector3d& p, 
    const Eigen::Vector4d& error, 
    const FrenetPoint& start, 
    const FrenetPoint& target, 
    int steps
  );
};
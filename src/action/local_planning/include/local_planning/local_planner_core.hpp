#pragma once

#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

// TODO(wato) make this a true frenet point and make converter functions
struct PathPoint{
  double x;
  double y;
  double theta;
  double kappa;
};

struct Path{
  std::vector<PathPoint> path;
  int64_t target_lanelet_id;
  double lateral_dist_from_goal_lane;
  double cost;
};

struct CostFunctionParams{
  double lateral_movement_weight;
  double physical_limits_weight;
  double preferred_lane_cost;
  double unknown_occupancy_cost;
  double max_curvature_change;
};

struct PathGenParams{
  int max_iterations;
  int steps;
  double tolerance;
  double newton_damping;
  double max_step_size;
};

class LocalPlannerCore
{
public:

  LocalPlannerCore();

  // utility functions
  double get_euc_dist(double x1, double y1, double x2, double y2);
  double get_angle_from_pts(double x1, double y1, double x2, double y2);
  double normalise_angle(double angle);

  // path generation
  std::vector<PathPoint> generate_path(
    PathPoint start, 
    PathPoint target,
    PathGenParams pg_params
  );
  
  void calculate_spiral_coeff(const double p[5], double (&coeffs)[4]);
  void generate_spiral(PathPoint start, int steps, double sf, double coeffs[4], std::vector<PathPoint>& path);

  Eigen::Vector4d compute_error(const PathPoint actual, const PathPoint target);

  Eigen::Matrix3d compute_jacobian_3dof(
    const Eigen::Vector3d& p, 
    const Eigen::Vector3d& error,
    const PathPoint& start,
    const PathPoint& target,
    int steps
  );
};
#pragma once

#include <vector>
#include <unordered_map>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

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

// Enums for parameter indexing
enum class SpiralParam {
  K0,      // Initial curvature
  K1,      // Curvature at 1/3 arc length
  K2,      // Curvature at 2/3 arc length
  K3,      // Terminal curvature
  SF       // Total arc length
};

enum class SpiralCoeff {
  C0,      // Constant coefficient
  C1,      // Linear coefficient
  C2,      // Quadratic coefficient
  C3       // Cubic coefficient
};

// Struct to hold spiral coefficient equation constants
struct SpiralCoeffConstants {
  // C1 numerator coefficients
  double c1_k0_coeff;
  double c1_k1_coeff;
  double c1_k2_coeff;
  double c1_k3_coeff;
  double c1_divisor;
  
  // C2 numerator coefficients
  double c2_k0_coeff;
  double c2_k1_coeff;
  double c2_k2_coeff;
  double c2_k3_coeff;
  double c2_divisor;
  
  // C3 numerator coefficients
  double c3_k0_coeff;
  double c3_k1_coeff;
  double c3_k2_coeff;
  double c3_k3_coeff;
  double c3_divisor;
};

class LatticePlanningCore
{
public:
  LatticePlanningCore(const SpiralCoeffConstants& constants, const PathGenParams& params);

  // utility functions
  double get_euc_dist(double x1, double y1, double x2, double y2);
  double get_angle_from_pts(double x1, double y1, double x2, double y2);
  double normalise_angle(double angle);

  // path generation
  std::vector<PathPoint> generate_path(
    PathPoint start, 
    PathPoint target
  );

  // path costing 
  Path get_lowest_cost_path(
    const std::vector<Path> & paths, 
    const std::unordered_map<int64_t, int> & preferred_lanelets, 
    const CostFunctionParams & cf_params
  );

  double path_cost_function(    
    const Path & path,
    bool preferred_lane,
    CostFunctionParams params
  );
  
  void calculate_spiral_coeff(
    const std::unordered_map<SpiralParam, double>& params, 
    std::unordered_map<SpiralCoeff, double>& coeffs
  );
  
  void generate_spiral(
    PathPoint start, 
    int steps, 
    double sf, 
    const std::unordered_map<SpiralCoeff, double>& coeffs, 
    std::vector<PathPoint>& path
  );

  Eigen::Vector3d compute_error_3dof(const PathPoint& actual, const PathPoint& target);

  Eigen::Matrix3d compute_jacobian_3dof(
    const Eigen::Vector3d& p, 
    const Eigen::Vector3d& error,
    const PathPoint& start,
    const PathPoint& target,
    int steps
  );

private:
  SpiralCoeffConstants spiral_constants_;
  PathGenParams pg_params_;
};
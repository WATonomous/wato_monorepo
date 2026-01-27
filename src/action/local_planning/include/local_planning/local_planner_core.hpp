#pragma once

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

  // std::vector<FrenetPoint> generate_angles(std::vector<FrenetPoint> path);
  double get_euc_dist(double x1, double y1, double x2, double y2);
};
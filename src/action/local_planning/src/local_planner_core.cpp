#include "local_planning/local_planner_core.hpp"
#include <cmath>

LocalPlannerCore::LocalPlannerCore() = default;

// std::vector<FrenetPoint> LocalPlannerCore::generate_angles(std::vector<FrenetPoint> path){}

double LocalPlannerCore::get_euc_dist(double x1, double y1, double x2, double y2){
    return std::hypot(x2 - x1, y2 - y1);
}
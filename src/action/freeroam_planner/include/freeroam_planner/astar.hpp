#ifndef FREEROAM_PLANNER__ASTAR_HPP_
#define FREEROAM_PLANNER__ASTAR_HPP_

#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace freeroam_planner
{

struct GridCell
{
  int row;
  int col;
};

/// Run A* on an OccupancyGrid from start_world to goal_world (in the grid's frame).
/// Returns a vector of world-frame (x, y) waypoints from start to goal, or empty if no path.
std::vector<std::pair<double, double>> astar(
  const nav_msgs::msg::OccupancyGrid & grid,
  double start_x, double start_y,
  double goal_x, double goal_y,
  int obstacle_threshold,
  bool allow_diagonal);

}  // namespace freeroam_planner

#endif  // FREEROAM_PLANNER__ASTAR_HPP_

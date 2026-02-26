#include "freeroam_planner/astar.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

namespace freeroam_planner
{

namespace
{

struct AStarNode
{
  int row;
  int col;
  double g;
  double f;
};

struct CompareF
{
  bool operator()(const AStarNode & a, const AStarNode & b) const { return a.f > b.f; }
};

inline int toIndex(int row, int col, int width)
{
  return row * width + col;
}

inline GridCell worldToGrid(
  double wx, double wy,
  double origin_x, double origin_y, double resolution)
{
  return {
    static_cast<int>(std::floor((wy - origin_y) / resolution)),
    static_cast<int>(std::floor((wx - origin_x) / resolution))};
}

inline std::pair<double, double> gridToWorld(
  int row, int col,
  double origin_x, double origin_y, double resolution)
{
  return {
    origin_x + (col + 0.5) * resolution,
    origin_y + (row + 0.5) * resolution};
}

}  // namespace

std::vector<std::pair<double, double>> astar(
  const nav_msgs::msg::OccupancyGrid & grid,
  double start_x, double start_y,
  double goal_x, double goal_y,
  int obstacle_threshold,
  bool allow_diagonal)
{
  const auto & info = grid.info;
  const int width = static_cast<int>(info.width);
  const int height = static_cast<int>(info.height);
  const double res = info.resolution;
  const double ox = info.origin.position.x;
  const double oy = info.origin.position.y;

  GridCell start = worldToGrid(start_x, start_y, ox, oy, res);
  GridCell goal = worldToGrid(goal_x, goal_y, ox, oy, res);

  // Bounds check
  auto inBounds = [&](int r, int c) {
    return r >= 0 && r < height && c >= 0 && c < width;
  };

  if (!inBounds(start.row, start.col) || !inBounds(goal.row, goal.col)) {
    return {};
  }

  // Check goal is not in obstacle
  if (grid.data[toIndex(goal.row, goal.col, width)] >= obstacle_threshold) {
    return {};
  }

  // Directions: 4-connected + optional diagonal
  std::vector<std::pair<int, int>> dirs = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}};
  if (allow_diagonal) {
    dirs.push_back({1, 1});
    dirs.push_back({1, -1});
    dirs.push_back({-1, 1});
    dirs.push_back({-1, -1});
  }

  auto heuristic = [&](int r, int c) {
    return std::hypot(r - goal.row, c - goal.col);
  };

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;
  std::unordered_map<int, double> g_costs;
  std::unordered_map<int, int> came_from;

  int start_idx = toIndex(start.row, start.col, width);
  open.push({start.row, start.col, 0.0, heuristic(start.row, start.col)});
  g_costs[start_idx] = 0.0;

  int goal_idx = toIndex(goal.row, goal.col, width);

  while (!open.empty()) {
    AStarNode current = open.top();
    open.pop();

    int cur_idx = toIndex(current.row, current.col, width);

    if (cur_idx == goal_idx) {
      // Reconstruct path
      std::vector<std::pair<double, double>> path;
      int idx = goal_idx;
      while (idx != start_idx) {
        int r = idx / width;
        int c = idx % width;
        path.push_back(gridToWorld(r, c, ox, oy, res));
        idx = came_from[idx];
      }
      path.push_back(gridToWorld(start.row, start.col, ox, oy, res));
      std::reverse(path.begin(), path.end());
      return path;
    }

    // Skip if we already found a better path to this node
    if (current.g > g_costs[cur_idx]) {
      continue;
    }

    for (auto & [dr, dc] : dirs) {
      int nr = current.row + dr;
      int nc = current.col + dc;
      if (!inBounds(nr, nc)) continue;

      int cell_cost = grid.data[toIndex(nr, nc, width)];
      if (cell_cost >= obstacle_threshold) continue;

      double move_cost = (dr != 0 && dc != 0) ? 1.414 : 1.0;
      // Soft penalty for cells with cost > 0
      double penalty = (cell_cost > 0) ? cell_cost * 0.1 : 0.0;
      double new_g = current.g + move_cost + penalty;

      int n_idx = toIndex(nr, nc, width);
      auto it = g_costs.find(n_idx);
      if (it == g_costs.end() || new_g < it->second) {
        g_costs[n_idx] = new_g;
        came_from[n_idx] = cur_idx;
        open.push({nr, nc, new_g, new_g + heuristic(nr, nc)});
      }
    }
  }

  return {};  // No path found
}

}  // namespace freeroam_planner

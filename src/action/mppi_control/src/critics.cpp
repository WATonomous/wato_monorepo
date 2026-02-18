
#include "critics.hpp"

static double angle_diff(double a, double b) {
    double d = a - b;
    while (d > M_PI)  d -= 2.0 * M_PI;
    while (d < -M_PI) d += 2.0 * M_PI;
    return d;
}

double critic::MppiCritic::evaluate(const std::vector<double>& state,
                                    const std::vector<double>& action,
                                    const std::vector<double>& prev_action) {
    
    if (state.size() < 4 || action.size() < 2 || prev_action.size() < 2) {
        return 1e6;
    }

    State s;
    s.x = state[0];
    s.y = state[1];
    s.yaw = state[2];
    s.v = state[3];

    double a = action[0];
    double delta = action[1];
    double prev_a = prev_action[0];
    double prev_delta = prev_action[1];

    // nearest reference and lateral/tangent
    int ref_idx = find_nearest_ref_index(s);
    double tx = 1.0, ty = 0.0;
    double lateral = lateral_error_and_tangent(ref_idx, s, tx, ty);

    double cost = 0.0;

    // lateral deviation, quadratic
    cost += params_.w_deviation * (lateral * lateral);

    // heading cost, quadratic
    double ref_yaw = std::atan2(ty, tx);
    double d_yaw = angle_diff(s.yaw, ref_yaw);

    cost += params_.w_heading * (d_yaw * d_yaw);

    // forward progress
    double forward_proj = s.v * (std::cos(s.yaw) * tx + std::sin(s.yaw) * ty);
    
    // progress lowers the cost
    cost -= params_.w_progress * forward_proj;

    // smoothing penalties, uses prev_action for delta
    double da = a - prev_a;
    double dd = delta - prev_delta;
    cost += params_.w_jerk * (da * da);
    cost += params_.w_steering_rate * (dd * dd);

    // obstacle cost
    cost += params_.w_obstacle * compute_obstacle_cost(s.x, s.y);

    // safety
    if (!std::isfinite(cost)) cost = 1e6;
    return cost;
}

void critic::MppiCritic::set_trajectory(const std::vector<struct State>& traj){
    desired_trajectory_ = traj; 
}

void critic::MppiCritic::set_occupancy_grid(const std::vector<int8_t>& data, 
                                            unsigned int width, unsigned int height,
                                            double resolution, double origin_x, double origin_y) {
    occupancy_grid_data_ = data;
    grid_width_ = width;
    grid_height_ = height;
    grid_resolution_ = resolution;
    grid_origin_x_ = origin_x;
    grid_origin_y_ = origin_y;
    has_occupancy_grid_ = (width > 0 && height > 0 && resolution > 0.0 && !data.empty());
    
    // compute distance field when grid is updated
    if (has_occupancy_grid_) {
        compute_distance_field();
    }
}

double critic::MppiCritic::terminal_cost(double x, double y, double yaw, double v){
    State s;
    s.x = x; s.y = y; s.yaw = yaw; s.v = v;

    int ref_idx = find_nearest_ref_index(s);
    double tx = 1.0, ty = 0.0;
    double lateral = lateral_error_and_tangent(ref_idx, s, tx, ty);

    double cost = 0.0;
    cost += params_.w_terminal_deviation * (lateral * lateral); // quadratic

    // terminal heading cost
    double ref_yaw = std::atan2(ty, tx);
    double d_yaw = angle_diff(s.yaw, ref_yaw);

    cost += params_.w_terminal_heading * (d_yaw * d_yaw); // quadratic

    // terminal progress lowers cost
    double forward_proj = s.v * (std::cos(s.yaw) * tx + std::sin(s.yaw) * ty);
    cost -= params_.w_terminal_progress * forward_proj;

    // terminal obstacle cost
    cost += params_.w_obstacle * compute_obstacle_cost(x, y);

    if (!std::isfinite(cost)) cost = 1e6;
    return cost;
}

// helpers:

// finds nearest reference index, (temporary)
int critic::MppiCritic::find_nearest_ref_index(const State &s) const {
    if (desired_trajectory_.empty()) return -1;

    double best_d2 = std::numeric_limits<double>::infinity();
    int best_idx = -1;
    for (size_t i = 0; i < desired_trajectory_.size(); ++i) {
        double dx = desired_trajectory_[i].x - s.x;
        double dy = desired_trajectory_[i].y - s.y;
        double d2 = dx*dx + dy*dy;
        if (d2 < best_d2) { best_d2 = d2; best_idx = static_cast<int>(i); }
    }
    return best_idx;
}

// computes lateral error to ref segment idx and unit tangent
// returns signed lateral distance, tangent_x/tangent_y set to unit tangent
double critic::MppiCritic::lateral_error_and_tangent(int idx, const State &s, double &tangent_x, double &tangent_y) const {
    // if no traj or invalid index, return large lateral error and default tangent
    if (desired_trajectory_.empty() || idx < 0) {
        tangent_x = 1.0;
        tangent_y = 0.0;
        return 1e3;
    }

    size_t i = static_cast<size_t>(idx);
    size_t j = (i + 1 < desired_trajectory_.size()) ? i + 1 : (i > 0 ? i - 1 : i);

    double x1 = desired_trajectory_[i].x;
    double y1 = desired_trajectory_[i].y;
    double x2 = desired_trajectory_[j].x;
    double y2 = desired_trajectory_[j].y;

    double vx = x2 - x1;
    double vy = y2 - y1;
    double vlen = std::hypot(vx, vy);

    if (vlen < 1e-6) {
        tangent_x = std::cos(desired_trajectory_[i].yaw);
        tangent_y = std::sin(desired_trajectory_[i].yaw);
        double dx = s.x - x1;
        double dy = s.y - y1;
        return std::hypot(dx, dy);
    }

    tangent_x = vx / vlen;
    tangent_y = vy / vlen;

    // proj of (s - segment_start) on to segment
    double wx = s.x - x1;
    double wy = s.y - y1;
    double proj = (wx * vx + wy * vy) / (vlen * vlen);
    proj = std::clamp(proj, 0.0, 1.0);

    double projx = x1 + proj * vx;
    double projy = y1 + proj * vy;

    double dxp = s.x - projx;
    double dyp = s.y - projy;
    double lat = std::hypot(dxp, dyp);

    // sign of lateral
    double cross = tangent_x * dyp - tangent_y * dxp;
    if (cross < 0) lat = -lat;

    return lat;
}

double critic::MppiCritic::compute_obstacle_cost(double x, double y) const {
    if (!has_occupancy_grid_ || !has_distance_field_) {
        return 0.0;  // no grid or distance field available
    }

    //check vehicle footprint, sample points in a circle around vehicle center
    double max_cost = 0.0;
    int num_samples = 8; 
    
    for (int i = 0; i < num_samples; ++i) {
        double angle = 2.0 * M_PI * i / num_samples;
        double sample_x = x + params_.vehicle_radius * std::cos(angle);
        double sample_y = y + params_.vehicle_radius * std::sin(angle);
        
        double dist = get_distance_to_obstacle(sample_x, sample_y);
        double cost = compute_distance_based_cost(dist);
        max_cost = std::max(max_cost, cost);
    }
    
    // check center point
    double center_dist = get_distance_to_obstacle(x, y);
    double center_cost = compute_distance_based_cost(center_dist);
    max_cost = std::max(max_cost, center_cost);
    
    return max_cost;
}

double critic::MppiCritic::compute_distance_based_cost(double distance) const {
    // distance is in meters, distance to nearest obstacle
    // cost decays from cost_at_obstacle (at distance = 0) to cost_at_inflation (at inflation_radius)
    
    if (distance < 0.0) {
        // inside obstacle or out of bounds
        return params_.cost_at_obstacle;
    }
    
    if (distance >= params_.obstacle_inflation_radius) {
        // outside inflation radius, no cost
        return 0.0;
    }
    
    // linear decay from cost_at_obstacle to cost_at_inflation
    double t = distance / params_.obstacle_inflation_radius;
    double cost = params_.cost_at_obstacle * (1.0 - t) + params_.cost_at_inflation * t;
    
    return cost;
}

double critic::MppiCritic::get_distance_to_obstacle(double x, double y) const {
    if (!has_distance_field_) {
        return -1.0;  // no distance field, treat as obstacle
    }

    // world coordinates to grid indices using floor for mapping
    int grid_x = static_cast<int>(std::floor((x - grid_origin_x_) / grid_resolution_));
    int grid_y = static_cast<int>(std::floor((y - grid_origin_y_) / grid_resolution_));

    // check if coordinates are within grid bounds
    if (grid_x < 0 || grid_x >= static_cast<int>(grid_width_) ||
        grid_y < 0 || grid_y >= static_cast<int>(grid_height_)) {
        // out of bounds, treat as obstacle (negative distance)
        return -1.0;
    }

    // distance from distance field
    size_t index = static_cast<size_t>(grid_y) * grid_width_ + static_cast<size_t>(grid_x);
    if (index >= distance_field_.size()) {
        return -1.0;  // invalid index, treat as obstacle
    }

    return distance_field_[index];
}

void critic::MppiCritic::compute_distance_field() {
    if (!has_occupancy_grid_ || grid_width_ == 0 || grid_height_ == 0) {
        has_distance_field_ = false;
        return;
    }

    size_t num_cells = grid_width_ * grid_height_;
    distance_field_.resize(num_cells);
    
    constexpr double INF = 1e6;  
    constexpr double INF_SQ = INF * INF; 
    
    // first identify obstacle cells and initialize distance field
    // F&H algorithm works with squared distances, 0 for obstacles, large value for free
    for (size_t i = 0; i < num_cells; ++i) {
        int8_t occupancy = (i < occupancy_grid_data_.size()) ? occupancy_grid_data_[i] : -1;
        
        // consider cells with occupancy > 50 as obstacles, or unknown as obstacles
        if (occupancy > 50 || occupancy < 0) {
            distance_field_[i] = 0.0;  // obstacle cell (squared distance = 0)
        } else {
            distance_field_[i] = INF_SQ;  // free cell, initialize to large squared distance
        }
    }
    
    // helper func for 1D EDT: finds the lower envelope of parabolas
    // input f contains squared distances, output is squared distances
    auto edt_1d = [INF, INF_SQ](const std::vector<double>& f, int n, double scale) -> std::vector<double> {
        std::vector<double> d(n);
        std::vector<int> v(n);
        std::vector<double> z(n + 1);
        int k = 0;
        
        v[0] = 0;
        z[0] = -INF;  // z stores intersection x-coordinates, not squared values
        z[1] = INF;
        
        for (int q = 1; q < n; ++q) {
            double s = ((f[q] + scale * scale * q * q) - (f[v[k]] + scale * scale * v[k] * v[k])) / (2.0 * scale * scale * (q - v[k]));
            
            while (s <= z[k]) {
                --k;
                if (k < 0) break;
                s = ((f[q] + scale * scale * q * q) - (f[v[k]] + scale * scale * v[k] * v[k])) / (2.0 * scale * scale * (q - v[k]));
            }
            ++k;
            v[k] = q;
            z[k] = s;
            z[k + 1] = INF;
        }
        
        k = 0;
        for (int q = 0; q < n; ++q) {
            while (k < static_cast<int>(v.size()) - 1 && z[k + 1] < q) ++k;
            int j = v[k];
            d[q] = scale * scale * (q - j) * (q - j) + f[j];
        }
        
        return d;
    };
    
    // first pass: compute EDT for each row (squared distances along rows)
    std::vector<double> row_distances(grid_width_);
    for (unsigned int y = 0; y < grid_height_; ++y) {
        // Extract row
        for (unsigned int x = 0; x < grid_width_; ++x) {
            size_t idx = y * grid_width_ + x;
            row_distances[x] = distance_field_[idx];
        }
        
        // compute 1D EDT for this row (output is squared distances)
        std::vector<double> edt_result = edt_1d(row_distances, grid_width_, grid_resolution_);
        
        // store results back (still squared distances)
        for (unsigned int x = 0; x < grid_width_; ++x) {
            size_t idx = y * grid_width_ + x;
            distance_field_[idx] = edt_result[x];
        }
    }
    
    // second pass: compute EDT for each column using row results
    std::vector<double> col_distances(grid_height_);
    for (unsigned int x = 0; x < grid_width_; ++x) {
        // extract column (contains squared distances from row pass)
        for (unsigned int y = 0; y < grid_height_; ++y) {
            size_t idx = y * grid_width_ + x;
            col_distances[y] = distance_field_[idx];
        }
        
        // compute 1D EDT for this column (output is squared distances in 2D)
        std::vector<double> edt_result = edt_1d(col_distances, grid_height_, grid_resolution_);
        
        // store results back and take square root to get actual Euclidean distance
        for (unsigned int y = 0; y < grid_height_; ++y) {
            size_t idx = y * grid_width_ + x;
            distance_field_[idx] = std::sqrt(edt_result[y]);
        }
    }
    
    has_distance_field_ = true;
}

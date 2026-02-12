
#include "critics.hpp"

static double angle_diff(double a, double b) {
    double d = a - b;
    while (d > M_PI)  d -= 2.0 * M_PI;
    while (d < -M_PI) d += 2.0 * M_PI;
    return d;
}

double critic::MppiCritic::evaluate(const std::vector<double>& state,
                                    const std::vector<double>& action,
                                    const std::vector<double>& prev_action, 
                                    const double dt) {
    
    if (state.size() < 4 || action.size() < 2 || prev_action.size() < 2) return 1e6;
    if (!(dt > 0.0) || !std::isfinite(dt)) return 1e6;

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
    double ref_speed = (ref_idx >= 0) ? desired_trajectory_[ref_idx].v : 0.0;
    double cost = 0.0;

    // lateral deviation, quadratic
    cost += dt * params_.w_deviation * (lateral * lateral);

    // heading cost, quadratic
    double ref_yaw = std::atan2(ty, tx);
    double d_yaw = angle_diff(s.yaw, ref_yaw);

    cost += dt* params_.w_heading * (d_yaw * d_yaw);

    // forward progress
    double forward_proj = dt * s.v * (std::cos(s.yaw) * tx + std::sin(s.yaw) * ty);
    
    // progress lowers the cost
    cost -= params_.w_progress * forward_proj;

    // smoothing penalties, uses prev_action for delta
    double jerk = (a - prev_a)/dt;
    double steer_rate = (delta - prev_delta)/dt;
    cost += params_.w_jerk * (jerk * jerk);
    cost += params_.w_steering_rate * (steer_rate * steer_rate);

    //effort
    cost += dt * params_.w_accel * (a * a);
    cost += dt * params_.w_steer_angle * (delta * delta);

    //speed tracking
    double speed_error = s.v - ref_speed;
    cost += dt * params_.w_velocity * (speed_error * speed_error);
    
    if (!std::isfinite(cost)) cost = 1e6;
    return cost;
}

void critic::MppiCritic::set_trajectory(const std::vector<struct State>& traj){
    desired_trajectory_ = traj; 
}

double critic::MppiCritic::terminal_cost(double x, double y, double yaw, double v){
    State s;
    s.x = x; s.y = y; s.yaw = yaw; s.v = v;

    int ref_idx = find_nearest_ref_index(s);
    double tx = 1.0, ty = 0.0;
    double lateral = lateral_error_and_tangent(ref_idx, s, tx, ty);
    double ref_speed = (ref_idx >= 0) ? desired_trajectory_[ref_idx].v : 0.0;

    double cost = 0.0;
    cost += params_.w_terminal_deviation * (lateral * lateral); // quadratic

    // terminal velocity cost
    double speed_error = s.v - ref_speed;
    cost += params_.w_terminal_velocity * (speed_error * speed_error);

    // terminal heading cost
    double ref_yaw = std::atan2(ty, tx);
    double d_yaw = angle_diff(s.yaw, ref_yaw);

    cost += params_.w_terminal_heading * (d_yaw * d_yaw); // quadratic

    // terminal progress lowers cost
    double forward_proj = s.v * (std::cos(s.yaw) * tx + std::sin(s.yaw) * ty);
    cost -= params_.w_terminal_progress * forward_proj;

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

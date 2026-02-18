#pragma once
#include <vector>
#include <cstddef>
#include <cmath>
#include <algorithm>
#include <limits>
#include "helpers.hpp"

namespace critic{

class MppiCritic{

// implement progress tracking for traj
// 

public:
    MppiCritic() = default;

    double evaluate(const std::vector<double>& state,
                            const std::vector<double>& action,
                            const std::vector<double>& prev_action,
                            const double dt);
    void set_trajectory(const std::vector<struct State>& traj);
    double terminal_cost(double x, double y, double yaw, double v);

    struct Params{
        // -- Progress Cost -- 

        //velocity tracking cost
        double w_velocity = 20.0;

        // regular step cost 
        double w_progress = 0.0;

        // terminal step cost
        double w_terminal_progress = 0.0;

        // step deviation cost
        double w_deviation = 10.0;

        // terminal deviation cost
        double w_terminal_deviation = 5.0;
        double w_terminal_velocity = 20.0;

        // heading error weights:
        double w_heading = 1.0;
        double w_terminal_heading = 1.0;

        // -- Rate Costs --
        double w_jerk = 0.1;
        double w_steering_rate = 0.5;

        //effort costs
        double w_accel = 0.05;
        double w_steer_angle = 0.1;
    };
private:
    Params params_;

    std::vector<struct State> desired_trajectory_;

    // helpers
    int find_nearest_ref_index(const State &s) const;
    double lateral_error_and_tangent(int index, const State &s, double &tangent_x, double &tangent_y) const;
    
};


}
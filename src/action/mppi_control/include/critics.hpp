#pragma once
#include <vector>
#include <cstddef>
#include "helpers.hpp"

namespace critic{

class MppiCritic{

// implement progress tracking for traj
// 

public:
    MppiCritic() = default;

    double evaluate(const std::vector<double>& state,
                            const std::vector<double>& action,
                            const std::vector<double>& prev_action);
    void set_trajectory(const std::vector<struct State>& traj);

    struct Params{
        // -- Progress Cost -- 
        // regular step cost 
        double w_progress = 1.0;

        // terminal step cost
        double w_terminal_progress = 10.0;

        // step deviation cost
        double w_deviation = 1.0;

        // terminal deviation cost
        double w_terminal_deviation = 10.0;

        // -- Rate Costs --
        // smoothen acceleration changes
        double w_jerk = 1.0;

        // smoothen steering rate changes
        double w_steering_rate = 1.0;
    };
private:
    Params params_;

    std::vector<struct State> desired_trajectory_;
    
};


}
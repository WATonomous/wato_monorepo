
#include "critics.hpp"

double critic::MppiCritic::evaluate(const std::vector<double>& state,
                                    const std::vector<double>& action,
                                    const std::vector<double>& prev_action) {
    // temp implementation
    return 0.0;
}

void critic::MppiCritic::set_trajectory(const std::vector<struct State>& traj){
    desired_trajectory_ = traj; 
}

double critic::MppiCritic::terminal_cost(double x, double y, double yaw, double v){
    // temp implementation
    return 0.0;
}
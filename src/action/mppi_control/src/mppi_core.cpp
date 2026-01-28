#include "mppi_core.hpp"

MppiCore::MppiCore(int num_samples, double time_horizon, int num_time_step,
                   double L, double a_noise_std, double delta_noise_std)
  : num_samples_(num_samples),
    time_horizon_(time_horizon),
    dt_(time_horizon / num_time_step),
    num_time_step_(num_time_step),
    a_noise_std_(a_noise_std),
    delta_noise_std_(delta_noise_std),
    L_(L),
    control_sequences_(num_samples, num_time_step)
{
}


State MppiCore::step_bicycle(const State& s, double a, double delta_dot, double dt, double L) {
    State ns = s;

    //clamp inputs here: a, delta_dot

    a = std::clamp(a, -1.0, 1.0);
    delta_dot = std::clamp(delta_dot, -1.0, 1.0);

    ns.delta = s.delta + delta_dot * dt;
    ns.delta = std::clamp(ns.delta, -1.0, 1.0);

    ns.x   = s.x   + s.v * std::cos(s.yaw) * dt;
    ns.y   = s.y   + s.v * std::sin(s.yaw) * dt;
    ns.yaw = s.yaw + (s.v / L) * std::tan(s.delta) * dt;
    ns.v   = s.v   + a * dt;

    // clamp states here: ns.v, ns.delta
    ns.v = std::clamp(ns.v, -1.0, 1.0);
    return ns;
}

Control_Output MppiCore::computeControl(){    
        add_noise_to_control_sequences();
        eval_trajectories_scores();
        compute_weights();
        aggregate_controls();
        return Control_Output{0.5, 0.0};
    };

void MppiCore::add_noise_to_control_sequences(){
        // add noise to control sequences
        for(int k=0; k<num_samples_; k++){
            for(int t=0; t<num_time_step_; t++){
                // simple gaussian noise

                double a_noise = gaussian_noise(a_noise_std_);
                double delta_noise = gaussian_noise(delta_noise_std_);
                control_sequences_.A(k, t) += a_noise;
                control_sequences_.D(k, t) += delta_noise;
            }
        }
    };
double MppiCore::compute_costs(const State& old_state, const State& new_state, double a, double delta_dot){
    return 1.0;
};

std::vector<double> MppiCore::eval_trajectories_scores(){
        std::vector<double> trajectory_costs(num_samples_);
        // simulate trajectories based on control sequences
        for (int k=0; k<num_samples_; k++){
            State sim_state = current_state_;
            for(int t=0; t<num_time_step_; t++){
                double a = control_sequences_.A(k, t);
                double delta_dot = control_sequences_.D(k, t);
                State old_state = sim_state;
                State sim_state = step_bicycle(sim_state, a, delta_dot, dt_, L_);
                // store sim_state if needed
                trajectory_costs[k] += compute_costs(old_state, sim_state, a, delta_dot); //add prev control commands also

            }
        }
        return trajectory_costs;
        
};
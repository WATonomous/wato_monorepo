#include "mppi_core.hpp"


MppiCore::MppiCore(int num_samples, double time_horizon, int num_time_step,
                   double L, double a_noise_std, double delta_noise_std,double accel_max, double steer_angle_max, double lambda)
  : num_samples_(num_samples),
    time_horizon_(time_horizon),
    dt_(time_horizon / num_time_step),
    num_time_step_(num_time_step),
    a_noise_std_(a_noise_std),
    delta_noise_std_(delta_noise_std),
    L_(L),
    noise_samples_(num_samples, num_time_step),
    optimal_control_sequence_(1, num_time_step),
    lambda_(lambda),
    accel_max_(accel_max),
    steer_angle_max_(steer_angle_max)
    
{
    // initialize control sequences to zero
    for (int k = 0; k < num_samples_; k++) {
        for (int t = 0; t < num_time_step_; t++) {
            noise_samples_.A(k, t) = 0.0;
            noise_samples_.D(k, t) = 0.0;
        }
    }
    // initialize optimal control sequence to zero
    for (int t = 0; t < num_time_step_; t++) {
        optimal_control_sequence_.A(0, t) = 0.0;
        optimal_control_sequence_.D(0, t) = 0.0;
    }
    critic_ = critic::MppiCritic();
    
}


State MppiCore::step_bicycle(const State& s, double a, double delta, double dt, double L) {
    State ns = s;

    a = std::clamp(a, -1.0*accel_max_, accel_max_);
    delta = std::clamp(delta, -1.0*steer_angle_max_,steer_angle_max_);

    ns.x   = s.x   + s.v * std::cos(s.yaw) * dt;
    ns.y   = s.y   + s.v * std::sin(s.yaw) * dt;
    ns.yaw = s.yaw + (s.v / L) * std::tan(delta) * dt;
    ns.v   = s.v   + a * dt;


    return ns;
}

Control_Output MppiCore::computeControl(){
        warm_start_control_sequences();
        add_noise_to_control_sequences();
        trajectory_costs_ = eval_trajectories_scores();
        compute_weights();
        weighted_average_controls();
        return Control_Output{optimal_control_sequence_.A(0, 0), optimal_control_sequence_.D(0, 0)};
    };
void MppiCore::warm_start_control_sequences() {
    // Shift the nominal (optimal) control sequence forward
    for (int t = 0; t < num_time_step_ - 1; t++) {
        optimal_control_sequence_.A(0, t) =
            optimal_control_sequence_.A(0, t + 1);

        optimal_control_sequence_.D(0, t) =
            optimal_control_sequence_.D(0, t + 1);
    }

    // last time step: repeat the last control
    optimal_control_sequence_.A(0, num_time_step_ - 1) = optimal_control_sequence_.A(0, num_time_step_ - 2);
    optimal_control_sequence_.D(0, num_time_step_ - 1) = optimal_control_sequence_.D(0, num_time_step_ - 2);

}

void MppiCore::add_noise_to_control_sequences(){
        // add noise to control sequences
        for(int k=0; k<num_samples_; k++){
            for(int t=0; t<num_time_step_; t++){
                // simple gaussian noise, consider clamps
                noise_samples_.A(k, t) = gaussian_noise(a_noise_std_);
                noise_samples_.D(k, t) = gaussian_noise(delta_noise_std_);
            }
        }
    };
double MppiCore::compute_costs(const State& old_state, const State& new_state, double u_a, double u_delta, double prev_u_a, double prev_u_delta){
        //use critic to compute costs
        std::vector<double> state_vec = {new_state.x, new_state.y, new_state.yaw, new_state.v};
        std::vector<double> action_vec = {u_a, u_delta};
        std::vector<double> prev_action_vec = {prev_u_a, prev_u_delta};
        double cost = critic_.evaluate(state_vec, action_vec, prev_action_vec);
    return cost;
};

std::vector<double> MppiCore::eval_trajectories_scores(){
        std::vector<double> trajectory_costs(num_samples_, 0.0);
        // simulate trajectories based on control sequences
        for (int k=0; k<num_samples_; k++){
            State sim_state = current_state_;
            double prev_u_a = optimal_control_sequence_.A(0, 0);
            double prev_u_delta = optimal_control_sequence_.D(0, 0);

            for (int t = 0; t < num_time_step_; t++) {
                const double u_a_nom = optimal_control_sequence_.A(0, t);
                const double u_d_nom = optimal_control_sequence_.D(0, t);

                const double eps_a = noise_samples_.A(k, t);
                const double eps_d = noise_samples_.D(k, t);

                double u_a = std::clamp(u_a_nom + eps_a, -accel_max_, accel_max_);
                double u_d = std::clamp(u_d_nom + eps_d, -steer_angle_max_, steer_angle_max_);

                State old_state = sim_state;
                sim_state = step_bicycle(sim_state, u_a, u_d, dt_, L_);

                trajectory_costs[k] += compute_costs(old_state, sim_state, u_a, u_d, prev_u_a, prev_u_delta);

                const double inv_var_a = 1.0 / (a_noise_std_ * a_noise_std_);
                const double inv_var_d = 1.0 / (delta_noise_std_ * delta_noise_std_);

                trajectory_costs[k] += lambda_ * (u_a_nom * eps_a * inv_var_a + u_d_nom * eps_d * inv_var_d);
                trajectory_costs[k] += 0.5 * lambda_ * (eps_a * eps_a * inv_var_a + eps_d * eps_d * inv_var_d);

                prev_u_a = u_a;
                prev_u_delta = u_d;
            }
            trajectory_costs[k] += critic_.terminal_cost(sim_state.x, sim_state.y, sim_state.yaw, sim_state.v);
        }
        return trajectory_costs;        
};


//compute weights
void MppiCore::compute_weights() {
    if (trajectory_costs_.size() != static_cast<size_t>(num_samples_)) {
        trajectory_costs_.assign(num_samples_, 0.0);
    }
    trajectory_weights_.assign(num_samples_, 0.0);

    // numerical stability: subtract the minimum cost
    double min_cost = trajectory_costs_[0];
    for (int k = 1; k < num_samples_; k++) {
        if (trajectory_costs_[k] < min_cost) min_cost = trajectory_costs_[k];
    }

    // softmin: w_k = exp(-(J_k - J_min)/lambda)
    double sum_w = 0.0;
    const double inv_lambda = (lambda_ > 1e-12) ? (1.0 / lambda_) : (1.0 / 1e-12);

    for (int k = 0; k < num_samples_; k++) {
        const double scaled = -(trajectory_costs_[k] - min_cost) * inv_lambda;
        const double w = std::exp(scaled);
        trajectory_weights_[k] = w;
        sum_w += w;
    }

    // normalize (avoid divide-by-zero)
    if (sum_w < 1e-12) {
        const double uniform = 1.0 / static_cast<double>(num_samples_);
        for (int k = 0; k < num_samples_; k++) trajectory_weights_[k] = uniform;
        return;
    }

    for (int k = 0; k < num_samples_; k++) {
        trajectory_weights_[k] /= sum_w;
    }
}


void MppiCore::weighted_average_controls() {
    if (trajectory_weights_.size() != static_cast<size_t>(num_samples_)) {
        return;
    }

    for (int t = 0; t < num_time_step_; t++) {
        double delta_a = 0.0;
        double delta_d = 0.0;

        for (int k = 0; k < num_samples_; k++) {
            const double w = trajectory_weights_[k];
            delta_a += w * noise_samples_.A(k, t);
            delta_d += w * noise_samples_.D(k, t);
        }

        // MPPI update: add noise correction
        optimal_control_sequence_.A(0, t) += delta_a;
        optimal_control_sequence_.D(0, t) += delta_d;
    }
}

//update pose
void MppiCore::update_pose(double x, double y, double yaw){
    current_state_.x = x;
    current_state_.y = y;
    current_state_.yaw = yaw;
}
void MppiCore::update_velocity(double v){
    current_state_.v = v;
}
//update trajectory
void MppiCore::update_trajectory(const std::vector<State>& traj){
    //assume traj is valid
    critic_.set_trajectory(traj);
    //desired_trajectory_ = traj; 
}
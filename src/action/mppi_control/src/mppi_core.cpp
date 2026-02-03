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
    control_sequences_(num_samples, num_time_step),
    optimal_control_sequence_(1, num_time_step),
    lambda_(lambda),
    accel_max_(accel_max),
    steer_angle_max_(steer_angle_max)
    
{
    // initialize control sequences to zero
    for (int k = 0; k < num_samples_; k++) {
        for (int t = 0; t < num_time_step_; t++) {
            control_sequences_.A(k, t) = 0.0;
            control_sequences_.D(k, t) = 0.0;
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

    //clamp inputs here: a, delta

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
void MppiCore::warm_start_control_sequences(){
    //shift optimal control sequence to control sequences
    for(int k=0; k<num_samples_; k++){
        for(int t=0; t<num_time_step_-1; t++){
            control_sequences_.A(k, t) = optimal_control_sequence_.A(0, t+1);
            control_sequences_.D(k, t) = optimal_control_sequence_.D(0, t+1);
        }
        //last time step set to zero
        control_sequences_.A(k, num_time_step_-1) = 0.0;
        control_sequences_.D(k, num_time_step_-1) = 0.0;
    }
};

void MppiCore::add_noise_to_control_sequences(){
        // add noise to control sequences
        for(int k=0; k<num_samples_; k++){
            for(int t=0; t<num_time_step_; t++){
                // simple gaussian noise

                double a_noise = gaussian_noise(a_noise_std_);
                double delta_noise = gaussian_noise(delta_noise_std_);
                //change to clamped values

                control_sequences_.A(k, t) = std::clamp(control_sequences_.A(k, t) + a_noise, -1.0*accel_max_, accel_max_);
                control_sequences_.D(k, t) = std::clamp(control_sequences_.D(k, t) + delta_noise, -1.0*steer_angle_max_, steer_angle_max_);
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
            for(int t=0; t<num_time_step_; t++){
                double u_a = control_sequences_.A(k, t);
                double u_delta = control_sequences_.D(k, t);
                double prev_u_a = (t==0) ? 0.0 : control_sequences_.A(k, t-1);
                double prev_u_delta = (t==0) ? 0.0 : control_sequences_.D(k, t-1);

                State old_state = sim_state;
                sim_state = step_bicycle(sim_state, u_a, u_delta, dt_, L_);
                // store sim_state if needed
                trajectory_costs[k] += compute_costs(old_state, sim_state, u_a, u_delta, prev_u_a, prev_u_delta);
            }
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
        //throw error
        return;
    }

    for (int t = 0; t < num_time_step_; t++) {
        double a_bar = 0.0;
        double d_bar = 0.0;

        for (int k = 0; k < num_samples_; k++) {
            const double w = trajectory_weights_[k];
            a_bar += w * control_sequences_.A(k, t);
            d_bar += w * control_sequences_.D(k, t);
        }

        a_bar = std::clamp(a_bar, -1.0*accel_max_, accel_max_);
        d_bar = std::clamp(d_bar, -1.0*steer_angle_max_, steer_angle_max_);

        optimal_control_sequence_.A(0, t) = a_bar;
        optimal_control_sequence_.D(0, t) = d_bar;
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
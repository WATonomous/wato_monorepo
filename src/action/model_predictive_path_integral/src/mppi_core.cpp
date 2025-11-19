//add header file



#include "rclcpp/rclcpp.hpp"
#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <torch/torch.h>
#include <random>
#include <mppi_core.hpp>

//action responses - s]
// follow path - responses: following path or no valid path found, stopping
// 
// 
// 
// - stop - stopping, stopped



MppiCore::MppiCore() {
    first_iteration = true;

    
    n = 50; //number of time steps in horizon
    control_dim = 2; //control dim: steer angle and acceleration

    previous_control_sequence = torch::zeros({control_dim, n});  // reference control sequence
    best_control_sequence = torch::zeros({control_dim, n});  // reference control sequence

    num_trajectories = 100;

    control_sequences = torch::zeros({num_trajectories, control_dim, n}); //trajectories tensor
    
    //state
    state_dim = 4; //x, y, yaw, v (longitudinal velocity)

    state_trajectories = torch::zeros({num_trajectories, state_dim, n});  


    noise_std_steer_ = 0.1; // tune this (units of your controls)
    noise_std_accel_ = 0.5; // tune this (units of your controls)

    dt_ = 0.05;                  // [s] time step
    L_  = 2.8;                   // [m] wheelbase
    max_steering_angle_rad_ = 0.5;  // [rad] max steering angle

    
    
    current_state_ = torch::zeros({4});       // [4] = [x, y, yaw, v], set before generate_control_command()

    q_control_effort_ = 0.1; // weight for control effort in cost function
    q_tracking_error_ = 1.0; // weight for tracking error in cost function
    q_collision_ = 100.0;    // weight for collision cost in cost function

    steer_max_ = 1.0;  // normalized max steering
    steer_min_ = -1.0; // normalized min steering
    accel_max_ = 1.0;  // max acceleration
    accel_min_ = 0.0; // min acceleration

}

void MppiCore::update_velocity(double v) {
    //update current_state
    current_state_.index_put_({3}, v);
}

void MppiCore::update_position(double x, double y, double yaw) {
    current_state_.index_put_({0}, x);
    current_state_.index_put_({1}, y);
    current_state_.index_put_({2}, yaw);
}

double MppiCore::quat_to_yaw(double x, double y, double z, double w) {
    // Convert quaternion to yaw angle (in radians)
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
}


void MppiCore::generate_control_command(){
    //make nominal control sequence equal to last best control sequence if not initial iteration
    if (first_iteration) {
        
        previous_control_sequence = best_control_sequence.clone();
        first_iteration = false;
    }

    add_noise_to_control_sequences();

    simulate_trajectories();

    evaluate_trajectories();



}
void MppiCore::add_noise_to_control_sequences() {
    // Expand nominal controls to match trajectory count
    auto base = nominal_control_sequence
                    .unsqueeze(0)                         // [1, 2, n]
                    .expand({num_trajectories, -1, -1});  // [N, 2, n]

    // ---  noise for steering ---
    auto steer_noise = torch::normal(
        /*mean=*/0.0,
        /*std=*/noise_std_steer_,
        /*size=*/{num_trajectories, 1, n},
        /*options=*/base.options()
    );  // [N, 1, n]

    // ---  noise for acceleration ---
    auto accel_noise = torch::normal(
        /*mean=*/0.0,
        /*std=*/noise_std_accel_,
        /*size=*/{num_trajectories, 1, n},
        /*options=*/base.options()
    );  // [N, 1, n]

    // Stack the independent noises into a single noise tensor
    auto noise = torch::cat({steer_noise, accel_noise}, /*dim=*/1);   // noise shape: [N, 2, n]

    // Final sampled controls
    control_sequences = base + noise;

    //clamp vaslues
}


using torch::indexing::Slice;

void MppiCore::simulate_trajectories() {
    // state_trajectories: [N, 4, n]
    // control_sequences:  [N, 2, n]
    // current_state_:     [4]  (x, y, yaw, v)

    const int64_t N_traj = num_trajectories;
    const int64_t T      = n;

    // Set initial state for all trajectories at k = 0
    // x0_batch: [N, 4]
    auto x0_batch = current_state_
                        .to(control_sequences.device())
                        .to(control_sequences.dtype())
                        .view({1, state_dim})
                        .expand({N_traj, state_dim});

    state_trajectories.index_put_({Slice(), Slice(), 0}, x0_batch);

    // Forward simulate using Euler discretization of the bicycle model
    for (int64_t k = 0; k < T - 1; ++k) {
        // x_k: [N, 4]
        auto x_k = state_trajectories.index({Slice(), Slice(), k});

        // Controls at time step k
        // steer_norm: [N], accel: [N]
        auto steer_norm = control_sequences.index({Slice(), 0, k});
        auto accel      = control_sequences.index({Slice(), 1, k});

        // Convert normalized steering to radians
        auto delta = steer_norm * max_steering_angle_rad_;

        // Extract yaw and speed
        auto yaw = x_k.index({Slice(), 2});  // [N]
        auto v   = x_k.index({Slice(), 3});  // [N]

        auto cos_yaw   = torch::cos(yaw);
        auto sin_yaw   = torch::sin(yaw);
        auto tan_delta = torch::tan(delta);

        // Bicycle model dynamics:
        // x_dot   = v * cos(yaw)
        // y_dot   = v * sin(yaw)
        // yaw_dot = (v / L) * tan(delta)
        // v_dot   = accel
        auto dx   = v * cos_yaw;             // [N]
        auto dy   = v * sin_yaw;             // [N]
        auto dyaw = (v / L_) * tan_delta;    // [N]
        auto dv   = accel;                   // [N]

        // Stack derivatives into [N, 4]
        auto x_dot = torch::stack({dx, dy, dyaw, dv}, /*dim=*/1);

        // Euler integration: x_{k+1} = x_k + dt * f(x_k, u_k)
        auto x_next = x_k + dt_ * x_dot;     // [N, 4]

        // Optionally clamp speed to be non-negative
        // auto v_next = x_next.index({Slice(), 3}).clamp_min(0.0);
        // x_next.index_put_({Slice(), 3}, v_next);

        // Store next state
        state_trajectories.index_put_({Slice(), Slice(), k + 1}, x_next);
    }
}


void MppiCore::evaluate_trajectories() {
    using torch::indexing::Slice;

    const int64_t N_traj = num_trajectories;

    // One scalar cost per trajectory
    auto costs = torch::empty(
        {N_traj},
        control_sequences.options().dtype(torch::kFloat64)
    );

    for (int64_t i = 0; i < N_traj; ++i) {
        // trajectory: [state_dim, n]
        auto trajectory = state_trajectories.index({i, Slice(), Slice()}).contiguous();

        // control sequence: [control_dim, n]
        auto control_sequence = control_sequences.index({i, Slice(), Slice()}).contiguous();

        // These functions are intentionally left unimplemented here.
        // You can change their signatures/returns as needed.
        double J_track = tracking_error(trajectory, reference_path_);
        double J_u     = control_effort(control_sequence);
        double J_col   = collision_cost(trajectory, costmap_);

        double J = q_tracking_error_ * J_track
                 + q_control_effort_ * J_u
                 + q_collision_      * J_col;

        costs.index_put_({i}, J);
    }

    // Find best (minimum-cost) trajectory index
    auto min_res  = costs.min(0);  // returns (values, indices)
    auto best_idx = std::get<1>(min_res).item<int64_t>();

    // Best control sequence: [control_dim, n]
    auto best_u = control_sequences.index({best_idx, Slice(), Slice()}).clone();

    // Receding-horizon update of nominal_control_sequence
    if (n > 1) {
        // Shift best_u by one step to build next iteration's nominal sequence
        // nominal[:, 0..n-2] = best_u[:, 1..n-1]
        nominal_control_sequence.index_put_(
            {Slice(), Slice(0, n - 1)},
            best_u.index({Slice(), Slice(1, n)})
        );

        // Last control stays as the last of the best sequence
        nominal_control_sequence.index_put_(
            {Slice(), n - 1},
            best_u.index({Slice(), n - 1})
        );
    } else {
        nominal_control_sequence = best_u;
    }

    // integrate_controls() will use nominal_control_sequence (or part of it)
    // to actually send commands; leave its implementation to you.
}



//tracking error - returns double
double tracking_error(const torch::Tensor& trajectory, const nav_msgs::msg::Path& reference_path) {
    
    return 0.0;
}

//control effort - returns double
double control_effort(const torch::Tensor& control_sequence) {
    
    return 0.0;
}

//collision cost - returns double
double collision_cost(const torch::Tensor& trajectory, const nav_msgs::msg::OccupancyGrid
& costmap) {
    
    return 0.0;
}


void MppiCore::integrate_controls() {
    

}

#include <vector>
#include <cmath>
#include <algorithm>
#include <vector>
#include <rclcpp/rclcpp.hpp>
//goal - no ROS dependencies
#include <random>

double gaussian_noise(double sigma)
{
    static std::mt19937 gen(std::random_device{}());
    static std::normal_distribution<double> dist(0.0, 1.0);

    return dist(gen) * sigma;
}

class MppiCore
{
struct State { double x, y, yaw, delta, v };

inline State step_bicycle(const State& s, double a, double delta_dot, double dt, double L) {
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


struct ControlSequences {
    int K;   // num_samples
    int T;   // num_time_step
    // size K*T
    std::vector<double> a; 
    std::vector<double> delta;  

    ControlSequences(int K_, int T_)
      : K(K_), T(T_), a((size_t)K_*T_), delta((size_t)K_*T_) {}

    inline size_t idx(int k, int t) const { return (size_t)k*T + t; }

    inline double& A(int k, int t) { return a[idx(k,t)]; }
    inline double& D(int k, int t) { return delta[idx(k,t)]; }
    inline const double& A(int k, int t) const { return a[idx(k,t)]; }
    inline const double& D(int k, int t) const { return delta[idx(k,t)]; }
};

public:
    MppiCore(int num_samples, double time_horizon, int num_time_step, double L, double a_noise_std, double delta_noise_std){
         // make into get from parameter 
        num_samples_ = num_samples;
        time_horizon_ = time_horizon;
        num_time_step_ = num_time_step; 
        dt_ = time_horizon_ / num_time_step_;
        a_noise_std_ = a_noise_std;
        delta_noise_std_ = delta_noise_std;
        L_ = L;


        control_sequences_ = ControlSequences(num_samples_, num_time_step_);


    }

    void computeControl(){    
        add_noise_to_control_sequences();
        eval_trajectories_scores();
        compute_weights();
        aggregate_controls();
    }

    void add_noise_to_control_sequences(){
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
    }

    std::vector<double> eval_trajectories_scores(){
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
        
    }


    double compute_costs(const State& old_state, const State& new_state, double a, double delta_dot){
        state reference_state;
        
    }

    void compute_weights(){}

    void aggregate_controls(){}



    void update_pose(double x, double y, double yaw){
        current_state_.x = x;
        current_state_.y = y;
        current_state_.yaw = yaw;
    }
    void update_velocity(double v){
        current_state_.v = v;
    } 
    //void update_trajectory(const std::vector<State>& traj){    }

private:
    State current_state_;

    int num_samples_;
    double time_horizon_;
    double dt_;
    double num_time_step_;


    double a_noise_std_ ;
    double delta_noise_std_ ;

    // wheelbase length
    double L_;
    ControlSequences control_sequences_;
};

//consider: lifecycle node
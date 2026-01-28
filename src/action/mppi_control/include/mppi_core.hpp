#include <vector>
#include <cmath>
#include <algorithm>
#include <vector>
//goal - no ROS dependencies
#include <random>

inline double gaussian_noise(double sigma)
{
    static std::mt19937 gen(std::random_device{}());
    static std::normal_distribution<double> dist(0.0, 1.0);

    return dist(gen) * sigma;
}

struct State { double x, y, yaw, delta, v;};

struct Control_Output { double a, delta_dot; };

class MppiCore
{


State step_bicycle(const State& s, double a, double delta_dot, double dt, double L);


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
    MppiCore(int num_samples, double time_horizon, int num_time_step, double L, double a_noise_std, double delta_noise_std);

    Control_Output computeControl();

    void add_noise_to_control_sequences();

    std::vector<double> eval_trajectories_scores();


    double compute_costs(const State& old_state, const State& new_state, double a, double delta_dot);

    void compute_weights(){}

    void aggregate_controls(){}


    /*
    void update_pose(double x, double y, double yaw){
        current_state_.x = x;
        current_state_.y = y;
        current_state_.yaw = yaw;
    }
    void update_velocity(double v){
        current_state_.v = v;
    } */
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
//add header file



#include "rclcpp/rclcpp.hpp"
#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <torch/torch.h>



//action responses - s]
// follow path - responses: following path or no valid path found, stopping
// 
// 
// 
// - stop - stopping, stopped



class MppiCore {

    public:
        MppiCore();

        void generate_random_trajectories();
        void evealuate_trajectories() ;
        void integrate_controls() ;

        void update_velocity(double v);
        void update_position(double x, double y, double yaw);

        double quat_to_yaw(double x, double y, double z, double w);
        

    private:
        int n; 
        int control_dim ; //control dim: steer angle and acceleration

        torch::Tensor previous_control_sequence;  // reference control sequence
        torch::Tensor best_control_sequence;  // reference control sequence
        int num_trajectories;

        torch::Tensor control_sequences; //trajectories tensor
        
        //state
        int state_dim; //x, y, yaw, v (longitudinal velocity)

        torch::Tensor state_trajectories;  

        torch::Tensor costs;
        
        double noise_std_steer_;
        double noise_std_accel_;
        bool first_iteration;
        double dt_;                  // [s] time step
        double L_;                   // [m] wheelbase
        double max_steering_angle_rad_;  // [rad] max steering angle
        torch::Tensor current_state_;       // [4] = [x, y, yaw, v], set before generate_control_command()

        double q_control_effort_; // weight for control effort in cost function
        double q_tracking_error_; // weight for tracking error in cost function
        double q_collision_;    // weight for collision cost in cost function
        

        double steer_max_;
        double steer_min_;
        double accel_max_;
        double accel_min_;
        

}  ;
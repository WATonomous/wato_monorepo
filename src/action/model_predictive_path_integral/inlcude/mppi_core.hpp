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



    private:
        int n; 
        int control_dim ; //control dim: steer angle and acceleration

        torch::Tensor nominal_control_sequence;  // reference control sequence

        int num_trajectories;

        torch::Tensor control_sequences; //trajectories tensor
        
        //state
        int state_dim; //x, y, yaw, v (longitudinal velocity)

        torch::Tensor state_trajectories;  
        
        double noise_std_steer_;
        double noise_std_accel_;

        double dt_;                  // [s] time step
        double L_;                   // [m] wheelbase
        double max_steering_angle_rad_;  // [rad] max steering angle
        torch::Tensor current_state_;       // [4] = [x, y, yaw, v], set before generate_control_command()

}  ;
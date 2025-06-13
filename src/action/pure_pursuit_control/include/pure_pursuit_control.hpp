#include <vector>
#include <cmath>
#include <limits>
#include <mutex>

#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_status.hpp>

class PurePursuitController : public rclcpp::Node {
    public:
        PurePursuitController();

    private:

        double lookahead_distance_;
        double control_frequency_;
        double max_steering_angle_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr control_pub_;
        // rclcpp::Client<world_modeling_msgs::srv::BehaviorTreeInfo>::SharedPtr bt_info_client_;
        rclcpp::TimerBase::SharedPtr timer_;

        nav_msgs::msg::Odometry::SharedPtr current_odom_;

        // std::vector<lanelet::routing::LaneletPath> path_;

        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void laneletPathCallback(const lanelet::routing::LaneletPath::SharedPtr msg);
        void controlLoop();
        int findClosestWaypointAhead(const geometry_msgs::msg::Pose& pose);
        bool findTargetWaypoint(int start_idx, const geometry_msgs::msg::Pose& pose, geometry_msgs::msg::PoseStamped& target_wp);
        double computeSteeringAngle(const geometry_msgs::msg::Pose& pose, const geometry_msgs::msg::Pose& target);
};

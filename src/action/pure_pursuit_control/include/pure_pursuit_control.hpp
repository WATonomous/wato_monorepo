#ifndef PURE_PURSUIT_CONTROL_HPP_
#define PURE_PURSUIT_CONTROL_HPP_

#include <vector>
#include <cmath>
#include <limits>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <world_modeling_msgs/srv/behaviour_tree_info.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_status.hpp>

constexpr int k = 2;
using Point2D = std::array<double, k>;

struct KDNode {
    Point2D point;
    KDNode* left;
    int index;
    KDNode* right;
};

// KD Tree helper functions
KDNode* newNode(const Point2D& arr, int index);
KDNode* insert(KDNode* root, const Point2D& arr, int index, unsigned depth);
KDNode* insert(KDNode* root, const Point2D& arr, int index);  // Wrapper
KDNode* closest(const Point2D& target, KDNode* n1, KDNode* n2);
KDNode* nearestNeighbour(KDNode* root, const Point2D& target, int depth = 0);

class PurePursuitController : public rclcpp::Node {
    public:
        PurePursuitController(const std::vector<geometry_msgs::msg::Point>& current_path);

    private:
        double lookahead_distance_;
        double control_frequency_;
        double max_steering_angle_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Client<world_modeling_msgs::srv::BehaviourTreeInfo>::SharedPtr bt_info_client_;
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr cmd_pub_;

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_wp_pub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

        nav_msgs::msg::Odometry::SharedPtr current_odom_;

        std::vector<geometry_msgs::msg::Point> current_path_;

        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void controlLoop();
        int findClosestWaypointAhead(const geometry_msgs::msg::Pose& pose);
        bool findTargetWaypoint(int start_idx, const geometry_msgs::msg::Pose& pose, const std::vector<geometry_msgs::msg::Point>& current_path, geometry_msgs::msg::PoseStamped& target_wp);
        double wrapAngle(double angle);
        double computeSteeringAngle(const geometry_msgs::msg::Pose& pose, const geometry_msgs::msg::Pose& target);

        KDNode* root = nullptr;

        void buildKDTree();
};

#endif  // PURE_PURSUIT_CONTROL_HPP_
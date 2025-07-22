#ifndef PURE_PURSUIT_CONTROLLER_HPP_
#define PURE_PURSUIT_CONTROLLER_HPP_

#include <vector>
#include <array>
#include <cmath>
#include <limits>
#include <mutex>

// ROS & messages
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <world_modeling_msgs/srv/behaviour_tree_info.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_status.hpp>

// **New** base‑class
#include "controller_pkg/base_controller.hpp"

constexpr int k = 2;
using Point2D = std::array<double, k>;

struct KDNode {
    Point2D point;
    KDNode* left{nullptr};
    KDNode* right{nullptr};
    int index{0};
};

// KD‑tree helpers (free functions)
KDNode* newNode(const Point2D& arr, int index);
KDNode* insert(KDNode* root, const Point2D& arr, int index, unsigned depth);
KDNode* insert(KDNode* root, const Point2D& arr, int index);
KDNode* closest(const Point2D& target, KDNode* n1, KDNode* n2);
KDNode* nearestNeighbour(KDNode* root, const Point2D& target, int depth = 0);

namespace controller_pkg {

class PurePursuitController : public BaseController {
public:
    PurePursuitController();
    ~PurePursuitController() override;

private:
    // --- your existing data members ---
    KDNode* root_{nullptr};

    double lookahead_distance_{2.0};
    double max_steering_angle_{1.4};

    // still need your behaviour tree client
    rclcpp::Client<world_modeling_msgs::srv::BehaviourTreeInfo>::SharedPtr bt_info_client_;

    nav_msgs::msg::Odometry::SharedPtr current_odom_;
    std::vector<geometry_msgs::msg::Point> current_path_;

    bool goalReached_{false};
    bool pathSet_{false};
    int current_index_{0};

    // --- overrides of the base‑class hooks ---
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) override;
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) override;
    void controlLoop() override;

    // --- your helper methods ---
    int findClosestWaypointAhead(const geometry_msgs::msg::Pose& pose);
    bool findTargetWaypoint(int start_idx,
                            const geometry_msgs::msg::Pose& pose,
                            const std::vector<geometry_msgs::msg::Point>& current_path,
                            geometry_msgs::msg::PoseStamped& target_wp);
    double computeSteeringAngle(const geometry_msgs::msg::Pose& pose,
                                const geometry_msgs::msg::Pose& target);
    void buildKDTree();
    void deleteKDTree(KDNode* node);
    bool isSamePath(const std::vector<geometry_msgs::msg::Point>& new_path);
};

}  // namespace controller_pkg

#endif  // PURE_PURSUIT_CONTROLLER_HPP_

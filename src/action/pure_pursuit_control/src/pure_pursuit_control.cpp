#include <pure_pursuit_control.hpp>

constexpr int k = 2;
using Point2D = std::array<double, k>;

struct KDNode {
    Point2D point;
    KDNode* left;
    KDNode* right;
};

KDNode* newNode(const Point2D& arr) {
    KDNode* temp = new KDNode;
    temp->point = arr;
    temp->left = temp->right = nullptr;
    return temp;
}

KDNode* insert(KDNode* root, const Point2D& arr, unsigned depth) {
    if (root == nullptr) {
        return newNode(arr);
    }

    unsigned cd = depth % k;

    if (arr[cd] < root->point[cd]) {
        root->left = insert(root->left, arr, depth + 1);
    } else {
        root->right = insert(root->right, arr, depth + 1);
    }

    return root;
}

KDNode* insert(KDNode* root, const Point2D& arr) {
    return insert(root, arr, 0);
}

KDNode* closest(const Point2D& target, KDNode* n1, KDNode* n2) {
    if (n1 == nullptr) return n2;
    if (n2 == nullptr) return n1;

    double dist1 = std::hypot(target[0] - n1->point[0], target[1] - n1->point[1]);
    double dist2 = std::hypot(target[0] - n2->point[0], target[1] - n2->point[1]);

    return (dist1 < dist2) ? n1 : n2;
}

KDNode* nearestNeighbour(KDNode* root, const Point2D& target, int depth = 0) {
    if (!root) return nullptr;

    unsigned cd = depth % k;
    KDNode* nextBranch = nullptr;
    KDNode* otherBranch = nullptr;

    if (target[cd] < root->point[cd]) {
        nextBranch = root->left;
        otherBranch = root->right;
    } else {
        nextBranch = root->right;
        otherBranch = root->left;
    }

    KDNode* temp = nearestNeighbour(nextBranch, target, depth + 1);
    KDNode* best = closest(target, temp, root);

    double radiusSquared = std::hypot(target[0] - root->point[0], target[1] - root->point[1]);
    double dist = target[cd] - root->point[cd];

    if (radiusSquared >= dist * dist) {
        temp = nearestNeighbour(otherBranch, target, depth + 1);
        best = closest(target, temp, best);
    }

    return best;
}

PurePursuitController::PurePursuitController() : Node("pure_pursuit_control") {

    this->declare_parameter<double>("lookahead_distance", 2.0);
    this->declare_parameter<double>("control_frequency", 500);
    this->declare_parameter<double>("max_steering_angle", 0.6);  // radians

    lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
    control_frequency_ = this->get_parameter("control_frequency").as_double();
    max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();

    // Subscriptions
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/carla/ego/odometry", 10, std::bind(&PurePursuitController::odomCallback, this, std::placeholders::_1));

    bt_info_client_ = this->create_client<world_modeling_msgs::srv::BehaviourTreeInfo>("behaviour_tree_info");

    while (!bt_info_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(this->get_logger(), "Waiting for Behaviour Tree Info service...");
    }
    
    // Timer for control loop
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(control_frequency_)),
        std::bind(&PurePursuitController::controlLoop, this));

    // Publisher for steering or velocity commands
    cmd_pub_ = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>("/cmd_lateral", 10);

    RCLCPP_INFO(this->get_logger(), "Pure Pursuit Controller Initialized");
}

std::vector<geometry_msgs::msg::Point> PurePursuitController::callBTInfoService() {
    std::vector<geometry_msgs::msg::Point> points;

    auto request = std::make_shared<world_modeling_msgs::srv::BehaviourTreeInfo::Request>();
    auto future = bt_info_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(shared_from_this(), future) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();

        const auto& route_list = response->route_list;
        if (!route_list.centerline.empty()) {
            for (const auto& point : route_list.centerline) {
                geometry_msgs::msg::Point p;
                p.x = point.x;
                p.y = point.y;
                p.z = point.z;  // Assuming z is part of the point structure
                points.push_back(p);
                std::cout << " Point: (" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
            }
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Service call failed");
    }

    return points;
}

void PurePursuitController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_odom_ = msg;
}

void PurePursuitController::controlLoop() {
    if (!current_odom_) return;

    RCLCPP_INFO(this->get_logger(), "control_loop");

    geometry_msgs::msg::Pose current_pose = current_odom_->pose.pose;

    RCLCPP_INFO(this->get_logger(), "current pose: x=%.2f, y=%.2f, z=%.2f",
                current_pose.position.x, current_pose.position.y, current_pose.position.z);

    std::vector<geometry_msgs::msg::Point> current_path = callBTInfoService();
    if (current_path.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty path from Behaviour Tree Info service.");
        return;
    }

    int closest_idx = findClosestWaypointAhead(current_pose, current_path);

    if (closest_idx == -1) {
        RCLCPP_WARN(this->get_logger(), "No valid waypoint found.");
        return;
    }

    geometry_msgs::msg::PoseStamped target_wp;
    bool success = findTargetWaypoint(closest_idx, current_pose, current_path, target_wp);

    if (!success) {
        RCLCPP_WARN(this->get_logger(), "Failed to find target waypoint.");
        return;
    }

    double steering_angle = computeSteeringAngle(current_pose, target_wp.pose);

    steering_angle = std::clamp(steering_angle, -max_steering_angle_, max_steering_angle_);

    carla_msgs::msg::CarlaEgoVehicleControl cmd;
    cmd.header.stamp = this->now();
    cmd.steer = steering_angle;
    cmd.throttle = 0.5;
    cmd_pub_->publish(cmd);
}

int PurePursuitController::findClosestWaypointAhead(const geometry_msgs::msg::Pose& pose, const std::vector<geometry_msgs::msg::Point>& path) {
    if (path.empty()) return -1;

    KDNode* root = nullptr;
    std::vector<std::array<double, 2>> points;
    for (const auto& p : path) {
        Point2D arr = { p.x, p.y };
        root = ::insert(root, arr);
        points.push_back({arr[0], arr[1]});
    }

    Point2D query = { pose.position.x, pose.position.y };
    KDNode* nearest = nearestNeighbour(root, query);

    for (size_t i = 0; i < points.size(); ++i) {
        if (points[i][0] == nearest->point[0] && points[i][1] == nearest->point[1])
            return static_cast<int>(i);
    }

    return -1;
}

bool PurePursuitController::findTargetWaypoint(int start_idx, const geometry_msgs::msg::Pose& pose, const std::vector<geometry_msgs::msg::Point>& current_path, geometry_msgs::msg::PoseStamped& target_wp) {
    if (current_path.size() < 2 || start_idx < 0 || static_cast<size_t>(start_idx) >= current_path.size()) {
        return false;
    }

    double L = lookahead_distance_;
    double cx = pose.position.x;
    double cy = pose.position.y;

    // Step through path until we find a segment where distance crosses L
    for (size_t i = start_idx; i < current_path.size() - 1; ++i) {
        const auto& p1 = current_path[i];
        const auto& p2 = current_path[i + 1];

        double d1 = std::hypot(p1.x - cx, p1.y - cy);
        double d2 = std::hypot(p2.x - cx, p2.y - cy);

        // If one point is before and one after the lookahead distance, interpolate
        if (d1 < L && d2 >= L) {
            double ratio = (L - d1) / (d2 - d1);  // Linear interpolation ratio

            target_wp.pose.position.x = p1.x + ratio * (p2.x - p1.x);
            target_wp.pose.position.y = p1.y + ratio * (p2.y - p1.y);
            target_wp.pose.position.z = p1.z + ratio * (p2.z - p1.z);

            // Orientation can be left as identity or copied from p1/p2 if needed
            target_wp.pose.orientation.w = 1.0;
            target_wp.pose.orientation.x = 0.0;
            target_wp.pose.orientation.y = 0.0;
            target_wp.pose.orientation.z = 0.0;

            return true;
        }
    }

    // If no segment crosses L, just pick the last point
    const auto& last = current_path.back();
    target_wp.pose.position = last;
    target_wp.pose.orientation.w = 1.0;
    target_wp.pose.orientation.x = 0.0;
    target_wp.pose.orientation.y = 0.0;
    target_wp.pose.orientation.z = 0.0;

    return true;
}

double PurePursuitController::computeSteeringAngle(const geometry_msgs::msg::Pose& pose, const geometry_msgs::msg::Pose& target) {
    // Transform target to vehicle frame

    double siny_cosp = 2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y);
    double cosy_cosp = 1.0 - 2.0 * (pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    double dx = target.position.x - pose.position.x;
    double dy = target.position.y - pose.position.y;

    double x_veh =  std::cos(-yaw) * dx - std::sin(-yaw) * dy;
    double y_veh =  std::sin(-yaw) * dx + std::cos(-yaw) * dy;

    if (x_veh == 0.0) return 0.0;

    double curvature = 2.0 * y_veh / (lookahead_distance_ * lookahead_distance_);
    return std::atan(curvature);  // Assuming a simple bicycle model
}

double PurePursuitController::wrapAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuitController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

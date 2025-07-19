#include <pure_pursuit_control.hpp>

KDNode* newNode(const Point2D& arr, int index) {
    KDNode* temp = new KDNode;
    temp->point = arr;
    temp->left = temp->right = nullptr;
    temp->index = index;
    return temp;
}

KDNode* insert(KDNode* root, const Point2D& arr, int index, unsigned depth) {

    if (root == nullptr) {
        return newNode(arr, index);
    }

    unsigned cd = depth % k;

    if (arr[cd] < root->point[cd]) {
        root->left = insert(root->left, arr, index, depth + 1);
    } else {
        root->right = insert(root->right, arr, index, depth + 1);
    }

    return root;
}

KDNode* insert(KDNode* root, const Point2D& arr, int index) {
    return insert(root, arr, index, 0);
}

KDNode* closest(const Point2D& target, KDNode* n1, KDNode* n2) {
    if (n1 == nullptr) return n2;
    if (n2 == nullptr) return n1;

    double dist1 = std::hypot(target[0] - n1->point[0], target[1] - n1->point[1]);
    double dist2 = std::hypot(target[0] - n2->point[0], target[1] - n2->point[1]);

    const double EPSILON = 3;  // Small threshold for "nearly equal"

    std::cout << "Distances are nearly equal: " << std::abs(dist1 - dist2) << std::endl;

    if (std::abs(dist1 - dist2) < EPSILON) {
        // Prefer lower index when distances are very close
        return (n1->index < n2->index) ? n1 : n2;
    }

    return (dist1 < dist2) ? n1 : n2;
}

KDNode* nearestNeighbour(KDNode* root, const Point2D& target, int depth) {
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
    this->declare_parameter<double>("control_frequency", 100.0);
    this->declare_parameter<double>("max_steering_angle", 1.4);  // radians

    lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
    control_frequency_ = this->get_parameter("control_frequency").as_double();
    max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();

    // Subscriptions
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/carla/ego/odometry", 10, std::bind(&PurePursuitController::odomCallback, this, std::placeholders::_1));

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/extracted_path", 10, std::bind(&PurePursuitController::pathCallback, this, std::placeholders::_1));

    // Timer for control loop
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(control_frequency_)),
        std::bind(&PurePursuitController::controlLoop, this));

    // Publisher for steering or velocity commands
    cmd_pub_ = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>("/carla/ego/vehicle_control_cmd", 10);

    RCLCPP_INFO(this->get_logger(), "Pure Pursuit Controller Initialized");
}

void PurePursuitController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_odom_ = msg;
}

void PurePursuitController::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (!pathSet && !msg->poses.empty()) {
        current_index_ = 0;
        current_path_.clear();
        
        std::vector<geometry_msgs::msg::Point> new_path;
        new_path.reserve(msg->poses.size());

        for (const auto& pose_stamped : msg->poses) {
            new_path.push_back(pose_stamped.pose.position);
        }

        // if (isSamePath(new_path)) {
        //     RCLCPP_INFO(this->get_logger(), "Received the same path, ignoring update.");
        //     return;
        // }

        current_path_ = std::move(new_path);
        buildKDTree();
        pathSet = true;
    }
}

bool PurePursuitController::isSamePath(const std::vector<geometry_msgs::msg::Point>& new_path) {
    if (new_path.size() != current_path_.size()) return false;

    for (size_t i = 0; i < new_path.size(); ++i) {
        const auto& p1 = new_path[i];
        const auto& p2 = current_path_[i];
        if (std::hypot(p1.x - p2.x, p1.y - p2.y) > 1e-3 || std::abs(p1.z - p2.z) > 1e-3) {
            return false;
        }
    }

    return true;
}

void PurePursuitController::controlLoop() {
    if (!current_odom_) return;

    geometry_msgs::msg::Pose current_pose = current_odom_->pose.pose;

    // RCLCPP_INFO(this->get_logger(), "current pose: x=%.2f, y=%.2f, z=%.2f",
                // current_pose.position.x, current_pose.position.y, current_pose.position.z);

    if (current_path_.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty path from Behaviour Tree Info service.");
        return;
    }

    int closest_idx = findClosestWaypointAhead(current_pose);

    if (closest_idx == -1) {
        RCLCPP_WARN(this->get_logger(), "No valid waypoint found.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Closest waypoint index: %d", closest_idx);
    RCLCPP_INFO(this->get_logger(), "Closest waypoint position: x=%.2f, y=%.2f, z=%.2f",
                current_path_[closest_idx].x, current_path_[closest_idx].y, current_path_[closest_idx].z);

    geometry_msgs::msg::PoseStamped target_wp;
    bool success = findTargetWaypoint(closest_idx, current_pose, current_path_, target_wp);

    if (!success) {
        RCLCPP_WARN(this->get_logger(), "Failed to find target waypoint.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Target waypoint position: x=%.2f, y=%.2f, z=%.2f",
                target_wp.pose.position.x, target_wp.pose.position.y, target_wp.pose.position.z);

    double steering_angle = -computeSteeringAngle(current_pose, target_wp.pose);

    RCLCPP_INFO(this->get_logger(), "Steering angle: %.2f radians", steering_angle);

    const auto& goal = current_path_.back();
    double dx = current_pose.position.x - goal.x;
    double dy = current_pose.position.y - goal.y;
    double distance_to_goal = std::hypot(dx, dy);

    carla_msgs::msg::CarlaEgoVehicleControl cmd;
    cmd.header.stamp = this->now();

    if (distance_to_goal < 0.1 || goalReached) {
        cmd.throttle = 0;
        cmd.brake = 1.0;
        cmd.steer = 0;
        RCLCPP_INFO(this->get_logger(), "Reached goal, applying brake.");
        goalReached = true;
    }
    else if (!goalReached) {
        cmd.throttle = 0.3;
        cmd.steer = std::clamp(steering_angle / max_steering_angle_, -1.0, 1.0);
    }

    cmd_pub_->publish(cmd);
}

int PurePursuitController::findClosestWaypointAhead(const geometry_msgs::msg::Pose& pose) {
    if (!root || current_path_.empty()) return -1;

    const double heading_x = std::cos(tf2::getYaw(pose.orientation));
    const double heading_y = std::sin(tf2::getYaw(pose.orientation));

    double min_dist = std::numeric_limits<double>::max();
    int best_index = -1;

    for (size_t i = 0; i < current_path_.size(); ++i) {
        const auto& wp = current_path_[i];

        double dx = wp.x - pose.position.x;
        double dy = wp.y - pose.position.y;

        double dot = dx * heading_x + dy * heading_y;
        if (dot <= 0.0) continue;

        double dist = std::hypot(dx, dy);
        if (dist < min_dist) {
            min_dist = dist;
            best_index = static_cast<int>(i);
        }
    }

    if (best_index != -1 && best_index > current_index_) {
        current_index_ = best_index;
    }

    return best_index;
}

bool PurePursuitController::findTargetWaypoint(int start_idx, const geometry_msgs::msg::Pose& pose, const std::vector<geometry_msgs::msg::Point>& current_path, geometry_msgs::msg::PoseStamped& target_wp) {
    double cx = pose.position.x;
    double cy = pose.position.y;

    geometry_msgs::msg::Point best_point;
    double best_distance = -1.0;

    for (size_t i = start_idx; i < current_path.size() - 1; ++i) {
        const auto& p1 = current_path[i];
        const auto& p2 = current_path[i + 1];

        double d1 = std::hypot(p1.x - cx, p1.y - cy);
        double d2 = std::hypot(p2.x - cx, p2.y - cy);

        if (d1 < lookahead_distance_ && d2 >= lookahead_distance_) {
            double ratio = (lookahead_distance_ - d1) / (d2 - d1);
            target_wp.pose.position.x = p1.x + ratio * (p2.x - p1.x);
            target_wp.pose.position.y = p1.y + ratio * (p2.y - p1.y);
            return true;
        }

        if (d1 < lookahead_distance_ && d1 > best_distance) {
            best_distance = d1;
            best_point = p1;
        }
    }

    if (best_distance > 0.0) {
        target_wp.pose.position = best_point;
        return true;
    }

    // As a last resort, pick a point slightly ahead if available
    if (!current_path.empty()) {
        size_t idx = std::min(static_cast<size_t>(start_idx + 2), current_path.size() - 1);
        target_wp.pose.position = current_path[idx];
        return true;
    }

    return false;  // No target found
}

double PurePursuitController::computeSteeringAngle(const geometry_msgs::msg::Pose& pose, const geometry_msgs::msg::Pose& target) {
    // Transform target to vehicle frame
    double yaw = tf2::getYaw(pose.orientation);

    double dx = target.position.x - pose.position.x;
    double dy = target.position.y - pose.position.y;

    double x_veh =   std::cos(yaw) * dx + std::sin(yaw) * dy;
    double y_veh =  -std::sin(yaw) * dx + std::cos(yaw) * dy;

    if (x_veh == 0.0) return 0.0;

    double curvature = 2.0 * y_veh / (lookahead_distance_ * lookahead_distance_);
    return std::atan(curvature);  // Assuming a simple bicycle model
}

void PurePursuitController::buildKDTree() {
    deleteKDTree(root);
    root = nullptr;
    for (size_t i = 0; i < current_path_.size(); ++i) {
        Point2D pt = {current_path_[i].x, current_path_[i].y};
        root = ::insert(root, pt, static_cast<int>(i));
    }
}

void PurePursuitController::deleteKDTree(KDNode* node) {
    if (!node) return;
    deleteKDTree(node->left);
    deleteKDTree(node->right);
    delete node;
} 

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto controller_node = std::make_shared<PurePursuitController>();
    rclcpp::spin(controller_node);
    rclcpp::shutdown();
    return 0;
}


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

    const double EPSILON = 0.5;  // Small threshold for "nearly equal"

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

    this->declare_parameter<double>("lookahead_distance", 4);
    this->declare_parameter<double>("control_frequency", 20);
    this->declare_parameter<double>("max_steering_angle", 1.2);  // radians

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
    std::vector<geometry_msgs::msg::Point> new_path;
    new_path.reserve(msg->poses.size());

    for (const auto& pose_stamped : msg->poses) {
        new_path.push_back(pose_stamped.pose.position);
    }

    if (isSamePath(new_path)) return;

    current_path_ = std::move(new_path);
    buildKDTree();
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

    // RCLCPP_INFO(this->get_logger(), "Closest waypoint index: %d", closest_idx);

    geometry_msgs::msg::PoseStamped target_wp;
    bool success = findTargetWaypoint(closest_idx, current_pose, current_path_, target_wp);

    if (!success) {
        RCLCPP_WARN(this->get_logger(), "Failed to find target waypoint.");
        return;
    }

    double steering_angle = -computeSteeringAngle(current_pose, target_wp.pose);

    // RCLCPP_INFO(this->get_logger(), "Steering angle: %.2f radians", steering_angle);

    carla_msgs::msg::CarlaEgoVehicleControl cmd;
    cmd.header.stamp = this->now();
    cmd.steer = std::clamp(steering_angle / max_steering_angle_, -1.0, 1.0);
    cmd.throttle = 0.3;
    cmd_pub_->publish(cmd);
}

int PurePursuitController::findClosestWaypointAhead(const geometry_msgs::msg::Pose& pose) {
    if (!root) return -1;

    Point2D query = { pose.position.x, pose.position.y };
    KDNode* nearest = nearestNeighbour(root, query);

    if (nearest) {
        // RCLCPP_INFO(this->get_logger(), "Closest waypoint index: %d (%f, %f)", nearest->index, nearest->point[0], nearest->point[1]);
        return nearest->index;
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
    double yaw = tf2::getYaw(pose.orientation);

    double dx = target.position.x - pose.position.x;
    double dy = target.position.y - pose.position.y;

    double x_veh =   std::cos(yaw) * dx + std::sin(yaw) * dy;
    double y_veh =  -std::sin(yaw) * dx + std::cos(yaw) * dy;

    if (x_veh == 0.0) return 0.0;

    double curvature = 2.0 * y_veh / (lookahead_distance_ * lookahead_distance_);
    return std::atan(curvature);  // Assuming a simple bicycle model
}

double PurePursuitController::wrapAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

void PurePursuitController::buildKDTree() {
    root = nullptr;
    for (size_t i = 0; i < current_path_.size(); ++i) {
        Point2D pt = {current_path_[i].x, current_path_[i].y};
        root = ::insert(root, pt, static_cast<int>(i));
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto controller_node = std::make_shared<PurePursuitController>();
    rclcpp::spin(controller_node);
    rclcpp::shutdown();
    return 0;
}


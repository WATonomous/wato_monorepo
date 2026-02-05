// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "behaviour/behaviour_node.hpp"

#include <chrono>
#include <filesystem>
#include <memory>
#include <string>

namespace behaviour
{

    /**
     * @brief Constructor for BehaviourNode.
     */

    BehaviourNode::BehaviourNode(const rclcpp::NodeOptions &options)
        : Node("behaviour_node", options)
    {
        // Declare parameters
        this->declare_parameter("bt_tree_file", "main_tree.xml");
        this->declare_parameter("rate_hz", 10.0);
        this->declare_parameter("ego_state_rate_hz", 20.0);
        this->declare_parameter("map_frame", "map");
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("enable_console_logging", false);

        // Init TF
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    /**
     * @brief Separate init function to pass node to behaviour tree after construction.
     */

    void BehaviourNode::init()
    {
        map_frame_ = this->get_parameter("map_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        double tick_rate_hz = this->get_parameter("rate_hz").as_double();
        double ego_rate_hz = this->get_parameter("ego_state_rate_hz").as_double();
        bool enable_console_logging = this->get_parameter("enable_console_logging").as_bool();

        // behaviour tree file path
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("behaviour");
        std::string bt_xml_name = this->get_parameter("bt_tree_file").as_string();
        std::filesystem::path tree_path = std::filesystem::path(package_share_directory) / "trees" / bt_xml_name;

        // create the tree
        tree_ = std::make_shared<BehaviourTree>(this->shared_from_this(), tree_path.string(), enable_console_logging);

        // set TF and frames on blackboard to be used by BT nodes
        tree_->updateBlackboard("tf_buffer", tf_buffer_);
        tree_->updateBlackboard("map_frame", map_frame_);
        tree_->updateBlackboard("base_frame", base_frame_);

        // stores for world state publishers
        dynamic_objects_store_ = std::make_shared<behaviour::DynamicObjectStore>();
        area_occupancy_store_ = std::make_shared<behaviour::AreaOccupancyStore>();

        auto tick_period = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / tick_rate_hz));
        auto ego_period = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / ego_rate_hz));

        // timer to tick the behaviour tree
        tick_tree_timer_ = this->create_wall_timer(
            tick_period, std::bind(&BehaviourNode::tickTreeTimerCallback, this));

        // timer to update ego state on blackboard
        tf_timer_ = this->create_wall_timer(
            ego_period, std::bind(&BehaviourNode::tfTimerCallback, this));

        // subscribers
        goal_point_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "goal_point", 10,
            [this](const geometry_msgs::msg::Point::SharedPtr msg)
            {
                tree_->updateBlackboard("goal_point", msg);
                RCLCPP_INFO(this->get_logger(), "New goal received: x=%.2f, y=%.2f", msg->x, msg->y);
            });

        current_lane_context_sub_ = this->create_subscription<lanelet_msgs::msg::CurrentLaneContext>(
            "/world_modeling/lanelet/lane_context", 10,
            [this](const lanelet_msgs::msg::CurrentLaneContext::SharedPtr msg)
            {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Received CurrentLaneContext message");
                tree_->updateBlackboard("current_lane_ctx", msg);
            });

        dynamic_objects_sub_ = this->create_subscription<world_model_msgs::msg::WorldObjectArray>(
            "/world_modeling/world_objects", rclcpp::QoS(10),
            [this](world_model_msgs::msg::WorldObjectArray::ConstSharedPtr msg)
            {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Received WorldObjectArray message with %zu objects", msg->objects.size());
                dynamic_objects_store_->update(msg);
            });

        area_occupancy_sub_ = this->create_subscription<world_model_msgs::msg::AreaOccupancy>(
            "/world_modeling/area_occupancy", rclcpp::QoS(10),
            [this](world_model_msgs::msg::AreaOccupancy::ConstSharedPtr msg)
            {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Received AreaOccupancy message with %zu areas", msg->areas.size());
                area_occupancy_store_->update(msg);
            });

        RCLCPP_INFO(this->get_logger(), "BehaviourNode has been fully initialized.");
    }

    /**
     * @brief Ticks the behavior tree.
     */
    void BehaviourNode::tickTreeTimerCallback()
    {
        try
        {
            // send latest world state snapshots to blackboard
            tree_->updateBlackboard("dynamic_objects.snapshot", dynamic_objects_store_->snapshot());
            tree_->updateBlackboard("area_occupancy.snapshot", area_occupancy_store_->snapshot());

            // tick tree
            tree_->tick();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Behavior Tree tick failed: %s", e.what());
        }
    }

    /**
     * @brief Fetches latest TF transform and updates the BT blackboard with ego state.
     */
    void BehaviourNode::tfTimerCallback()
    {
        try
        {
            const auto tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
            const rclcpp::Time current_time = tf.header.stamp;

            // 1. Extract Position as geometry_msgs::msg::Point
            geometry_msgs::msg::Point::SharedPtr ego_point = std::make_shared<geometry_msgs::msg::Point>();
            ego_point->x = tf.transform.translation.x;
            ego_point->y = tf.transform.translation.y;
            ego_point->z = tf.transform.translation.z;

            // Convert to TF2 types for math
            tf2::Vector3 current_position(ego_point->x, ego_point->y, ego_point->z);
            tf2::Quaternion current_orientation;
            tf2::fromMsg(tf.transform.rotation, current_orientation);

            // 2. Calculate Velocity (Twist)
            geometry_msgs::msg::Twist::SharedPtr ego_velocity = std::make_shared<geometry_msgs::msg::Twist>();

            // Only calculate if we have a valid previous state AND the time sources match
            if (has_last_tf_ && current_time.get_clock_type() == last_time_.get_clock_type())
            {
                const double dt = (current_time - last_time_).seconds();

                if (dt > 0.001)
                {
                    // World-frame linear velocity
                    tf2::Vector3 world_vel = (current_position - last_position_) / dt;

                    // Convert to body-frame (Local) velocity
                    //
                    tf2::Vector3 local_vel = tf2::quatRotate(current_orientation.inverse(), world_vel);
                    ego_velocity->linear.x = local_vel.x();
                    ego_velocity->linear.y = local_vel.y();
                    ego_velocity->linear.z = local_vel.z();

                    // Calculate Yaw rate (Angular Z)
                    tf2::Quaternion dq = last_orientation_.inverse() * current_orientation;
                    dq.normalize();
                    double roll, pitch, yaw;
                    tf2::Matrix3x3(dq).getRPY(roll, pitch, yaw);
                    ego_velocity->angular.z = yaw / dt;
                }
            }

            // 3. Update Blackboard
            tree_->updateBlackboard("ego_point", ego_point);
            tree_->updateBlackboard("ego_velocity", ego_velocity);

            // 4. Update State for next calculation
            last_position_ = current_position;
            last_orientation_ = current_orientation;
            last_time_ = current_time;
            has_last_tf_ = true;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "TF lookup failed: %s", ex.what());
        }
    }

} // namespace behaviour

/**
 * @brief Main entry point for the behaviour node.
 */
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<behaviour::BehaviourNode>();
    node->init();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
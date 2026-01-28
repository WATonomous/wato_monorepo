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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace behaviour
{

/**
 * @brief Constructor for BehaviourNode.
 * Handles parameter declaration, core utility initialization, and BT setup.
 */
BehaviourNode::BehaviourNode(const rclcpp::NodeOptions & options)
: Node("behaviour_node", options)
{
  // 1. Declare and retrieve parameters
  this->declare_parameter("bt_tree_file", "main_tree.xml");
  this->declare_parameter("rate_hz", 10.0);
  this->declare_parameter("ego_state_rate_hz", 20.0);
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("base_frame", "base_link");
  
  map_frame_ = this->get_parameter("map_frame").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();
  double tick_rate_hz = this->get_parameter("rate_hz").as_double();
  double ego_rate_hz = this->get_parameter("ego_state_rate_hz").as_double();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 3. Setup Behaviour Tree path
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("behaviour");
  std::string bt_xml_name = this->get_parameter("bt_tree_file").as_string();
  std::filesystem::path tree_path = std::filesystem::path(package_share_directory) / "trees" / bt_xml_name;

  // 4. Initialize the Behaviour Tree
  // We pass 'this->shared_from_this()' so the BT can access node capabilities
  tree_ = std::make_shared<BehaviourTree>(
    this->shared_from_this(),
    tree_path.string(),
    std::make_shared<DynamicObjectStore>());

  // 5. Initialize Timers
  auto tick_period = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / tick_rate_hz));
  tick_tree_timer_ = this->create_wall_timer(
    tick_period, std::bind(&BehaviourNode::tickTreeTimerCallback, this));

  auto ego_period = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / ego_rate_hz));
  ego_state_timer_ = this->create_wall_timer(
    ego_period, std::bind(&BehaviourNode::egoStateTimerCallback, this));

  RCLCPP_INFO(this->get_logger(), "BehaviourNode has been initialized.");
}

/**
 * @brief Ticks the behavior tree.
 */
void BehaviourNode::tickTreeTimerCallback()
{
  try {
    tree_->tick();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Behavior Tree tick failed: %s", e.what());
  }
}

/**
 * @brief Fetches latest TF transform and updates the BT blackboard with ego state.
 */
void BehaviourNode::egoStateTimerCallback()
{
  try {
    // Lookup latest transform
    const auto tf =
      tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);

    const rclcpp::Time current_time = tf.header.stamp;

    // ego_point
    geometry_msgs::msg::PointStamped ego_point;
    ego_point.header = tf.header;
    ego_point.point.x = tf.transform.translation.x;
    ego_point.point.y = tf.transform.translation.y;
    ego_point.point.z = tf.transform.translation.z;

    tf2::Vector3 current_pos(
      ego_point.point.x,
      ego_point.point.y,
      ego_point.point.z);

    tf2::Quaternion current_rot;
    tf2::fromMsg(tf.transform.rotation, current_rot);

    // --- Ego velocity ---
    geometry_msgs::msg::TwistStamped ego_velocity;
    ego_velocity.header = tf.header;

    if (has_last_tf_) {
      const double dt = (current_time - last_time_).seconds();

      if (dt > 1e-3) {
        // linear velocity
        tf2::Vector3 world_vel = (current_pos - last_pos_) / dt;

        // Convert to body frame
        tf2::Vector3 local_vel =
          tf2::quatRotate(current_rot.inverse(), world_vel);

        ego_velocity.twist.linear.x = local_vel.x();
        ego_velocity.twist.linear.y = local_vel.y();
        ego_velocity.twist.linear.z = local_vel.z();

        // angular velocity
        tf2::Quaternion dq = last_rot_.inverse() * current_rot;
        dq.normalize();

        double roll, pitch, yaw;
        tf2::Matrix3x3(dq).getRPY(roll, pitch, yaw);

        ego_velocity.twist.angular.z = yaw / dt;
      }
    }

    // update blackboard
    tree_->updateBlackboard("ego_point", ego_point);
    tree_->updateBlackboard("ego_velocity", ego_velocity);

    last_pos_ = current_pos;
    last_rot_ = current_rot;
    last_time_ = current_time;
    has_last_tf_ = true;

  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Ego TF lookup failed: %s", ex.what());
  }
}

}  // namespace behaviour

/**
 * @brief Main entry point for the behaviour node.
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<behaviour::BehaviourNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
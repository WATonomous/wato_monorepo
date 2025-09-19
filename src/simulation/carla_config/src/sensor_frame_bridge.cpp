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

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using std::placeholders::_1;

// Bridge node to convert the frame of certain sensor messages from ego to base_link
// Mainly for localization, so IMU and GNSS
class SensorFrameBridge : public rclcpp::Node
{
public:
  SensorFrameBridge()
  : Node("sensor_frame_bridge")
  {
    // Declare and get parameters for input/output topics and target frame
    this->declare_parameter("output_imu_topic", "/imu");
    this->declare_parameter("output_gnss_topic", "/gnss");
    this->declare_parameter("target_frame", "base_link");

    std::string output_imu_topic = this->get_parameter("output_imu_topic").as_string();
    std::string output_gnss_topic = this->get_parameter("output_gnss_topic").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();

    // Publishers
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(output_imu_topic, 10);
    gnss_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(output_gnss_topic, 10);

    // Subscriptions
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/carla/ego/imu", 10, std::bind(&SensorFrameBridge::imu_callback, this, _1));
    gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/carla/ego/gnss", 10, std::bind(&SensorFrameBridge::gnss_callback, this, _1));

    // Create tf objects, in order to duplicate the transforms
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&SensorFrameBridge::broadcast_transform, this));
  }
  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_pub_;
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  // TF objects
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string target_frame_;

  // Callback to update frame_id and republish Imu
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    auto out_msg = *msg;
    out_msg.header.frame_id = target_frame_ + "/imu";
    imu_pub_->publish(out_msg);
  }

  // Callback to update frame_id and republish NavSatFix
  void gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    auto out_msg = *msg;
    out_msg.header.frame_id = target_frame_ + "/gnss";
    gnss_pub_->publish(out_msg);
  }

  // Broadcast the transform from base_link to imu and gnss
  void broadcast_transform()
  {
    std::vector<std::string> sensor_frames = {"ego/imu", "ego/gnss"};
    for (const auto & sensor_frame : sensor_frames) {
      try {
        geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
          "ego", sensor_frame, tf2::TimePointZero);
        // Republish as base_link -> base_link/sensor
        geometry_msgs::msg::TransformStamped transform_stamped_out;
        transform_stamped_out.header.stamp = transform_stamped.header.stamp;
        transform_stamped_out.header.frame_id = target_frame_;
        transform_stamped_out.child_frame_id = target_frame_ + "/" + sensor_frame.substr(4); // remove "ego/"
        transform_stamped_out.transform = transform_stamped.transform;
        // Broadcast the transform
        tf_broadcaster_->sendTransform(transform_stamped_out);
      } catch (tf2::TransformException & ex) {}
    }
  }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SensorFrameBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

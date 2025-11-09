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

#include "prediction/object_adapter.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace
{
double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 m(tf_q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}
}  // namespace

ObjectAdapter::ObjectAdapter()
: Node("object_adapter")
{
  // Subscriber
  subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
    "hd_map_viz", 10,
    std::bind(&ObjectAdapter::marker_array_callback, this, std::placeholders::_1));

  // Publisher
  publisher_ = this->create_publisher<autoware_perception_msgs::msg::TrackedObjects>(
    "/prediction/tracked_objects", 10);
}

void ObjectAdapter::marker_array_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
  autoware_perception_msgs::msg::TrackedObjects tracked_objects;
  if (!msg->markers.empty()) {
    tracked_objects.header = msg->markers[0].header;
  } else {
    tracked_objects.header.stamp = this->now();
    tracked_objects.header.frame_id = "map";
  }

  const rclcpp::Time current_time = tracked_objects.header.stamp;
  if (is_first_) {
    is_first_ = false;
    prev_time_ = current_time;
    for (const auto & marker : msg->markers) {
      autoware_perception_msgs::msg::TrackedObject tracked_object;
      tracked_object.kinematics.pose_with_covariance.pose = marker.pose;
      tracked_object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
      tracked_object.shape.dimensions.x = marker.scale.x;
      tracked_object.shape.dimensions.y = marker.scale.y;
      tracked_object.shape.dimensions.z = marker.scale.z;
      prev_objects_[marker.id] = tracked_object;
    }
    return;
  }

  const double dt = (current_time - prev_time_).seconds();
  if (dt < 1e-3) {
    return;
  }

  std::unordered_map<int, autoware_perception_msgs::msg::TrackedObject> new_objects;

  for (const auto & marker : msg->markers) {
    autoware_perception_msgs::msg::TrackedObject tracked_object;
    tracked_object.kinematics.pose_with_covariance.pose = marker.pose;
    tracked_object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    tracked_object.shape.dimensions.x = marker.scale.x;
    tracked_object.shape.dimensions.y = marker.scale.y;
    tracked_object.shape.dimensions.z = marker.scale.z;

    if (prev_objects_.count(marker.id)) {
      const auto & prev_object = prev_objects_.at(marker.id);
      const auto & prev_pose = prev_object.kinematics.pose_with_covariance.pose;
      const auto & current_pose = tracked_object.kinematics.pose_with_covariance.pose;

      // Velocities
      tracked_object.kinematics.twist_with_covariance.twist.linear.x =
        (current_pose.position.x - prev_pose.position.x) / dt;
      tracked_object.kinematics.twist_with_covariance.twist.linear.y =
        (current_pose.position.y - prev_pose.position.y) / dt;
      tracked_object.kinematics.twist_with_covariance.twist.linear.z =
        (current_pose.position.z - prev_pose.position.z) / dt;

      const double prev_yaw = get_yaw_from_quaternion(prev_pose.orientation);
      const double current_yaw = get_yaw_from_quaternion(current_pose.orientation);
      tracked_object.kinematics.twist_with_covariance.twist.angular.z =
        (current_yaw - prev_yaw) / dt;

      // Acceleration
      const auto & prev_twist = prev_object.kinematics.twist_with_covariance.twist;
      const auto & current_twist = tracked_object.kinematics.twist_with_covariance.twist;
      tracked_object.kinematics.acceleration_with_covariance.accel.linear.x =
        (current_twist.linear.x - prev_twist.linear.x) / dt;
      tracked_object.kinematics.acceleration_with_covariance.accel.linear.y =
        (current_twist.linear.y - prev_twist.linear.y) / dt;
      tracked_object.kinematics.acceleration_with_covariance.accel.linear.z =
        (current_twist.linear.z - prev_twist.linear.z) / dt;
    }

    tracked_objects.objects.push_back(tracked_object);
    new_objects[marker.id] = tracked_object;
  }

  prev_objects_ = new_objects;
  prev_time_ = current_time;

  publisher_->publish(tracked_objects);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectAdapter>());
  rclcpp::shutdown();
  return 0;
}
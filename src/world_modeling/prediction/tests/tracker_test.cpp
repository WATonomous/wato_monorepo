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

#include <catch2/catch.hpp>
#include <wato_test/wato_test.hpp>

#include <chrono>
#include <memory>

#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "prediction/tracker_core.hpp"

using namespace std::chrono_literals;

namespace
{
/**
 * @brief Helper function to create a quaternion from yaw angle
 */
geometry_msgs::msg::Quaternion create_quaternion_from_yaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.w = std::cos(yaw / 2.0);
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  return q;
}

/**
 * @brief Helper function to create a marker at a specific position
 */
visualization_msgs::msg::Marker create_marker(
  int id, double x, double y, double z, double yaw, rclcpp::Time stamp)
{
  visualization_msgs::msg::Marker marker;
  marker.id = id;
  marker.header.stamp = stamp;
  marker.header.frame_id = "map";
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation = create_quaternion_from_yaw(yaw);
  marker.scale.x = 4.0;  // length
  marker.scale.y = 2.0;  // width
  marker.scale.z = 1.5;  // height
  return marker;
}
}  // namespace

TEST_CASE("TrackerCore processes empty marker array", "[tracker]")
{
  wato::world_modeling::prediction::TrackerCore adapter;
  
  auto marker_array = std::make_shared<visualization_msgs::msg::MarkerArray>();
  autoware_perception_msgs::msg::TrackedObjects tracked_objects;

  adapter.processMarkerArray(marker_array, tracked_objects);

  SECTION("Empty array produces no tracked objects")
  {
    REQUIRE(tracked_objects.objects.size() == 0);
    REQUIRE(tracked_objects.header.frame_id == "map");
  }
}

TEST_CASE("TrackerCore processes single marker on first call", "[tracker]")
{
  wato::world_modeling::prediction::TrackerCore adapter;
  
  auto marker_array = std::make_shared<visualization_msgs::msg::MarkerArray>();
  rclcpp::Time stamp(1, 0, RCL_ROS_TIME);
  
  marker_array->markers.push_back(create_marker(1, 10.0, 20.0, 0.0, 0.5, stamp));

  autoware_perception_msgs::msg::TrackedObjects tracked_objects;
  adapter.processMarkerArray(marker_array, tracked_objects);

  SECTION("First call initializes state but produces no output")
  {
    // On first call, the adapter stores state but doesn't output objects
    REQUIRE(tracked_objects.objects.size() == 0);
    REQUIRE(tracked_objects.header.stamp == stamp);
    REQUIRE(tracked_objects.header.frame_id == "map");
  }
}

TEST_CASE("TrackerCore computes velocity between two frames", "[tracker]")
{
  wato::world_modeling::prediction::TrackerCore adapter;
  
  rclcpp::Time t0(0, 0, RCL_ROS_TIME);
  rclcpp::Time t1(0, 100000000, RCL_ROS_TIME);  // 0.1 seconds later (100ms = 100,000,000ns)
  double dt = 0.1;

  SECTION("Object moving in positive x direction")
  {
    // First frame at (0, 0)
    auto marker_array_0 = std::make_shared<visualization_msgs::msg::MarkerArray>();
    marker_array_0->markers.push_back(create_marker(1, 0.0, 0.0, 0.0, 0.0, t0));
    
    autoware_perception_msgs::msg::TrackedObjects tracked_objects_0;
    adapter.processMarkerArray(marker_array_0, tracked_objects_0);
    
    // Second frame at (1, 0) - moved 1m in x direction
    auto marker_array_1 = std::make_shared<visualization_msgs::msg::MarkerArray>();
    marker_array_1->markers.push_back(create_marker(1, 1.0, 0.0, 0.0, 0.0, t1));
    
    autoware_perception_msgs::msg::TrackedObjects tracked_objects_1;
    adapter.processMarkerArray(marker_array_1, tracked_objects_1);

    REQUIRE(tracked_objects_1.objects.size() == 1);
    
    const auto & obj = tracked_objects_1.objects[0];
    
    // Check velocity: dx/dt = 1.0 / 0.1 = 10.0 m/s
    REQUIRE(obj.kinematics.twist_with_covariance.twist.linear.x == Approx(10.0));
    REQUIRE(obj.kinematics.twist_with_covariance.twist.linear.y == Approx(0.0));
    REQUIRE(obj.kinematics.twist_with_covariance.twist.linear.z == Approx(0.0));
  }

  SECTION("Object moving in positive y direction")
  {
    adapter = wato::world_modeling::prediction::TrackerCore();
    
    // First frame at (0, 0)
    auto marker_array_0 = std::make_shared<visualization_msgs::msg::MarkerArray>();
    marker_array_0->markers.push_back(create_marker(1, 0.0, 0.0, 0.0, 0.0, t0));
    
    autoware_perception_msgs::msg::TrackedObjects tracked_objects_0;
    adapter.processMarkerArray(marker_array_0, tracked_objects_0);
    
    // Second frame at (0, 2) - moved 2m in y direction
    auto marker_array_1 = std::make_shared<visualization_msgs::msg::MarkerArray>();
    marker_array_1->markers.push_back(create_marker(1, 0.0, 2.0, 0.0, 0.0, t1));
    
    autoware_perception_msgs::msg::TrackedObjects tracked_objects_1;
    adapter.processMarkerArray(marker_array_1, tracked_objects_1);

    REQUIRE(tracked_objects_1.objects.size() == 1);
    
    const auto & obj = tracked_objects_1.objects[0];
    
    // Check velocity: dy/dt = 2.0 / 0.1 = 20.0 m/s
    REQUIRE(obj.kinematics.twist_with_covariance.twist.linear.x == Approx(0.0));
    REQUIRE(obj.kinematics.twist_with_covariance.twist.linear.y == Approx(20.0));
    REQUIRE(obj.kinematics.twist_with_covariance.twist.linear.z == Approx(0.0));
  }
}

TEST_CASE("TrackerCore computes angular velocity", "[tracker]")
{
  wato::world_modeling::prediction::TrackerCore adapter;
  
  rclcpp::Time t0(0, 0, RCL_ROS_TIME);
  rclcpp::Time t1(0, 100000000, RCL_ROS_TIME);  // 0.1 seconds later (100ms = 100,000,000ns)
  double dt = 0.1;

  SECTION("Object rotating counterclockwise")
  {
    // First frame at yaw = 0
    auto marker_array_0 = std::make_shared<visualization_msgs::msg::MarkerArray>();
    marker_array_0->markers.push_back(create_marker(1, 0.0, 0.0, 0.0, 0.0, t0));
    
    autoware_perception_msgs::msg::TrackedObjects tracked_objects_0;
    adapter.processMarkerArray(marker_array_0, tracked_objects_0);
    
    // Second frame at yaw = 0.1 rad
    auto marker_array_1 = std::make_shared<visualization_msgs::msg::MarkerArray>();
    marker_array_1->markers.push_back(create_marker(1, 0.0, 0.0, 0.0, 0.1, t1));
    
    autoware_perception_msgs::msg::TrackedObjects tracked_objects_1;
    adapter.processMarkerArray(marker_array_1, tracked_objects_1);

    REQUIRE(tracked_objects_1.objects.size() == 1);
    
    const auto & obj = tracked_objects_1.objects[0];
    
    // Check angular velocity: dyaw/dt = 0.1 / 0.1 = 1.0 rad/s
    REQUIRE(obj.kinematics.twist_with_covariance.twist.angular.z == Approx(1.0));
  }
}

TEST_CASE("TrackerCore computes acceleration", "[tracker]")
{
  wato::world_modeling::prediction::TrackerCore adapter;
  
  rclcpp::Time t0(0, 0, RCL_ROS_TIME);
  rclcpp::Time t1(0, 100000000, RCL_ROS_TIME);  // 0.1 seconds later (100ms = 100,000,000ns)
  rclcpp::Time t2(0, 200000000, RCL_ROS_TIME);  // 0.2 seconds later (200ms = 200,000,000ns)
  double dt = 0.1;

  SECTION("Object accelerating in x direction")
  {
    // Frame 0: at x=0
    auto marker_array_0 = std::make_shared<visualization_msgs::msg::MarkerArray>();
    marker_array_0->markers.push_back(create_marker(1, 0.0, 0.0, 0.0, 0.0, t0));
    
    autoware_perception_msgs::msg::TrackedObjects tracked_objects_0;
    adapter.processMarkerArray(marker_array_0, tracked_objects_0);
    
    // Frame 1: at x=1 (velocity = 10 m/s)
    auto marker_array_1 = std::make_shared<visualization_msgs::msg::MarkerArray>();
    marker_array_1->markers.push_back(create_marker(1, 1.0, 0.0, 0.0, 0.0, t1));
    
    autoware_perception_msgs::msg::TrackedObjects tracked_objects_1;
    adapter.processMarkerArray(marker_array_1, tracked_objects_1);
    
    // Frame 2: at x=3 (velocity = 20 m/s)
    auto marker_array_2 = std::make_shared<visualization_msgs::msg::MarkerArray>();
    marker_array_2->markers.push_back(create_marker(1, 3.0, 0.0, 0.0, 0.0, t2));
    
    autoware_perception_msgs::msg::TrackedObjects tracked_objects_2;
    adapter.processMarkerArray(marker_array_2, tracked_objects_2);

    REQUIRE(tracked_objects_2.objects.size() == 1);
    
    const auto & obj = tracked_objects_2.objects[0];
    
    // Velocity at t1: 10 m/s, Velocity at t2: 20 m/s
    // Acceleration: (20 - 10) / 0.1 = 100 m/s^2
    REQUIRE(obj.kinematics.acceleration_with_covariance.accel.linear.x == Approx(100.0));
    REQUIRE(obj.kinematics.acceleration_with_covariance.accel.linear.y == Approx(0.0));
  }
}

TEST_CASE("TrackerCore handles multiple objects", "[tracker]")
{
  wato::world_modeling::prediction::TrackerCore adapter;
  
  rclcpp::Time t0(0, 0, RCL_ROS_TIME);
  rclcpp::Time t1(0, 100000000, RCL_ROS_TIME);  // 0.1 seconds later (100ms = 100,000,000ns)

  SECTION("Tracks multiple objects independently")
  {
    // First frame: two objects
    auto marker_array_0 = std::make_shared<visualization_msgs::msg::MarkerArray>();
    marker_array_0->markers.push_back(create_marker(1, 0.0, 0.0, 0.0, 0.0, t0));
    marker_array_0->markers.push_back(create_marker(2, 10.0, 0.0, 0.0, 0.0, t0));
    
    autoware_perception_msgs::msg::TrackedObjects tracked_objects_0;
    adapter.processMarkerArray(marker_array_0, tracked_objects_0);
    
    // Second frame: both objects moved
    auto marker_array_1 = std::make_shared<visualization_msgs::msg::MarkerArray>();
    marker_array_1->markers.push_back(create_marker(1, 1.0, 0.0, 0.0, 0.0, t1));  // moved 1m
    marker_array_1->markers.push_back(create_marker(2, 10.0, 2.0, 0.0, 0.0, t1));  // moved 2m in y
    
    autoware_perception_msgs::msg::TrackedObjects tracked_objects_1;
    adapter.processMarkerArray(marker_array_1, tracked_objects_1);

    REQUIRE(tracked_objects_1.objects.size() == 2);
    
    // Find objects by checking their velocities (order might differ)
    bool found_x_mover = false;
    bool found_y_mover = false;
    
    for (const auto & obj : tracked_objects_1.objects) {
      if (std::abs(obj.kinematics.twist_with_covariance.twist.linear.x - 10.0) < 0.01) {
        found_x_mover = true;
        REQUIRE(obj.kinematics.twist_with_covariance.twist.linear.y == Approx(0.0));
      }
      if (std::abs(obj.kinematics.twist_with_covariance.twist.linear.y - 20.0) < 0.01) {
        found_y_mover = true;
        REQUIRE(obj.kinematics.twist_with_covariance.twist.linear.x == Approx(0.0));
      }
    }
    
    REQUIRE(found_x_mover);
    REQUIRE(found_y_mover);
  }
}

TEST_CASE("TrackerCore handles new objects appearing", "[tracker]")
{
  wato::world_modeling::prediction::TrackerCore adapter;
  
  rclcpp::Time t0(0, 0, RCL_ROS_TIME);
  rclcpp::Time t1(0, 100000000, RCL_ROS_TIME);  // 0.1 seconds later (100ms = 100,000,000ns)

  SECTION("New objects have no velocity data")
  {
    // First frame: one object
    auto marker_array_0 = std::make_shared<visualization_msgs::msg::MarkerArray>();
    marker_array_0->markers.push_back(create_marker(1, 0.0, 0.0, 0.0, 0.0, t0));
    
    autoware_perception_msgs::msg::TrackedObjects tracked_objects_0;
    adapter.processMarkerArray(marker_array_0, tracked_objects_0);
    
    // Second frame: original object moved + new object appears
    auto marker_array_1 = std::make_shared<visualization_msgs::msg::MarkerArray>();
    marker_array_1->markers.push_back(create_marker(1, 1.0, 0.0, 0.0, 0.0, t1));
    marker_array_1->markers.push_back(create_marker(2, 20.0, 0.0, 0.0, 0.0, t1));  // new
    
    autoware_perception_msgs::msg::TrackedObjects tracked_objects_1;
    adapter.processMarkerArray(marker_array_1, tracked_objects_1);

    REQUIRE(tracked_objects_1.objects.size() == 2);
    
    // The new object (id=2) should have zero velocity since it has no history
    bool found_tracked = false;
    bool found_new = false;
    
    for (const auto & obj : tracked_objects_1.objects) {
      double vx = obj.kinematics.twist_with_covariance.twist.linear.x;
      if (std::abs(vx - 10.0) < 0.01) {
        found_tracked = true;  // This is the tracked object
      }
      if (std::abs(vx) < 0.01) {
        found_new = true;  // This is the new object with no velocity
      }
    }
    
    REQUIRE(found_tracked);
    REQUIRE(found_new);
  }
}

TEST_CASE("TrackerCore preserves bounding box dimensions", "[tracker]")
{
  wato::world_modeling::prediction::TrackerCore adapter;
  
  rclcpp::Time t0(0, 0, RCL_ROS_TIME);
  rclcpp::Time t1(0, 100000000, RCL_ROS_TIME);  // 0.1 seconds later (100ms = 100,000,000ns)

  SECTION("Bounding box dimensions are preserved")
  {
    // First frame
    auto marker_array_0 = std::make_shared<visualization_msgs::msg::MarkerArray>();
    marker_array_0->markers.push_back(create_marker(1, 0.0, 0.0, 0.0, 0.0, t0));
    
    autoware_perception_msgs::msg::TrackedObjects tracked_objects_0;
    adapter.processMarkerArray(marker_array_0, tracked_objects_0);
    
    // Second frame
    auto marker_array_1 = std::make_shared<visualization_msgs::msg::MarkerArray>();
    marker_array_1->markers.push_back(create_marker(1, 1.0, 0.0, 0.0, 0.0, t1));
    
    autoware_perception_msgs::msg::TrackedObjects tracked_objects_1;
    adapter.processMarkerArray(marker_array_1, tracked_objects_1);

    REQUIRE(tracked_objects_1.objects.size() == 1);
    
    const auto & obj = tracked_objects_1.objects[0];
    
    REQUIRE(obj.shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX);
    REQUIRE(obj.shape.dimensions.x == Approx(4.0));
    REQUIRE(obj.shape.dimensions.y == Approx(2.0));
    REQUIRE(obj.shape.dimensions.z == Approx(1.5));
  }
}

TEST_CASE("TrackerCore skips updates with small time delta", "[tracker]")
{
  wato::world_modeling::prediction::TrackerCore adapter;
  
  rclcpp::Time t0(0, 0, RCL_ROS_TIME);
  rclcpp::Time t1(0, 100, RCL_ROS_TIME);  // Only 100 nanoseconds later (< 1ms)

  SECTION("Updates with dt < 1ms are skipped")
  {
    // First frame
    auto marker_array_0 = std::make_shared<visualization_msgs::msg::MarkerArray>();
    marker_array_0->markers.push_back(create_marker(1, 0.0, 0.0, 0.0, 0.0, t0));
    
    autoware_perception_msgs::msg::TrackedObjects tracked_objects_0;
    adapter.processMarkerArray(marker_array_0, tracked_objects_0);
    
    // Second frame very close in time
    auto marker_array_1 = std::make_shared<visualization_msgs::msg::MarkerArray>();
    marker_array_1->markers.push_back(create_marker(1, 1.0, 0.0, 0.0, 0.0, t1));
    
    autoware_perception_msgs::msg::TrackedObjects tracked_objects_1;
    adapter.processMarkerArray(marker_array_1, tracked_objects_1);

    // Should produce no objects due to small dt
    REQUIRE(tracked_objects_1.objects.size() == 0);
  }
}

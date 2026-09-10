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

#include <tf2_ros/static_transform_broadcaster.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <thread>

#include <catch2/catch_all.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <wato_test/wato_test.hpp>

#include "trajectory_planner/trajectory_planner_node.hpp"

namespace wato
{

TEST_CASE_METHOD(
  wato::test::TestExecutorFixture,
  "trajectory deformation survives the inverse costmap transform",
  "[trajectory_planner_node][tf]")
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("traj_pub_topic", "/elastic_tf_test/trajectory");
  options.append_parameter_override("marker_pub_topic", "/elastic_tf_test/markers");
  options.append_parameter_override("path_sub_topic", "/elastic_tf_test/path");
  options.append_parameter_override("costmap_sub_topic", "/elastic_tf_test/costmap");
  options.append_parameter_override("lane_context_sub_topic", "/elastic_tf_test/lane_context");
  options.append_parameter_override("odom_sub_topic", "/elastic_tf_test/odom");
  options.append_parameter_override("bt_sub_topic", "/elastic_tf_test/behaviour");
  options.append_parameter_override("interpolation_resolution", 0.25);
  options.append_parameter_override("footprint_x_min", 0.0);
  options.append_parameter_override("footprint_x_max", 0.0);
  options.append_parameter_override("footprint_y_min", -0.5);
  options.append_parameter_override("footprint_y_max", 0.5);
  options.append_parameter_override("eb_clearance_margin", 0.2);
  options.append_parameter_override("eb_transition_distance", 2.0);

  auto planner = std::make_shared<trajectory_planner::TrajectoryPlannerNode>(options);
  auto tf_node = std::make_shared<rclcpp::Node>("elastic_tf_test_broadcaster");
  auto tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(tf_node);
  auto path_pub = std::make_shared<wato::test::PublisherTestNode<nav_msgs::msg::Path>>(
    "/elastic_tf_test/path", "elastic_tf_test_path_publisher");
  auto costmap_pub = std::make_shared<wato::test::PublisherTestNode<nav_msgs::msg::OccupancyGrid>>(
    "/elastic_tf_test/costmap", "elastic_tf_test_costmap_publisher");
  auto odom_pub = std::make_shared<wato::test::PublisherTestNode<nav_msgs::msg::Odometry>>(
    "/elastic_tf_test/odom", "elastic_tf_test_odom_publisher");
  auto behaviour_pub = std::make_shared<wato::test::PublisherTestNode<behaviour_msgs::msg::ExecuteBehaviour>>(
    "/elastic_tf_test/behaviour", "elastic_tf_test_behaviour_publisher");
  auto trajectory_sub = std::make_shared<wato::test::SubscriberTestNode<wato_trajectory_msgs::msg::Trajectory>>(
    "/elastic_tf_test/trajectory", "elastic_tf_test_trajectory_subscriber");

  add_node(planner);
  add_node(tf_node);
  add_node(path_pub);
  add_node(costmap_pub);
  add_node(odom_pub);
  add_node(behaviour_pub);
  add_node(trajectory_sub);

  geometry_msgs::msg::TransformStamped map_from_costmap;
  map_from_costmap.header.stamp = tf_node->now();
  map_from_costmap.header.frame_id = "map";
  map_from_costmap.child_frame_id = "base_footprint";
  map_from_costmap.transform.translation.x = 4.0;
  map_from_costmap.transform.translation.y = -2.0;
  map_from_costmap.transform.rotation.z = std::sin(M_PI / 4.0);
  map_from_costmap.transform.rotation.w = std::cos(M_PI / 4.0);
  tf_broadcaster->sendTransform(map_from_costmap);

  REQUIRE(planner->configure().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  REQUIRE(planner->activate().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  start_spinning();

  REQUIRE(path_pub->wait_for_subscribers(1, std::chrono::seconds(2)));
  REQUIRE(costmap_pub->wait_for_subscribers(1, std::chrono::seconds(2)));
  REQUIRE(odom_pub->wait_for_subscribers(1, std::chrono::seconds(2)));
  REQUIRE(behaviour_pub->wait_for_subscribers(1, std::chrono::seconds(2)));
  REQUIRE(trajectory_sub->wait_for_publishers(1, std::chrono::seconds(2)));

  // A straight costmap-frame path (x=0..20, y=0) transformed by the static TF above becomes
  // a vertical map-frame path at x=4, y=-2..18.
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (double costmap_x : {0.0, 10.0, 20.0}) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = 4.0;
    pose.pose.position.y = -2.0 + costmap_x;
    pose.pose.orientation.z = std::sin(M_PI / 4.0);
    pose.pose.orientation.w = std::cos(M_PI / 4.0);
    path.poses.push_back(pose);
  }

  nav_msgs::msg::OccupancyGrid costmap;
  costmap.header.frame_id = "base_footprint";
  costmap.info.resolution = 0.5;
  costmap.info.width = 50;
  costmap.info.height = 20;
  costmap.info.origin.position.x = -1.0;
  costmap.info.origin.position.y = -3.0;
  costmap.data.assign(costmap.info.width * costmap.info.height, 0);
  const int obstacle_col = static_cast<int>(std::floor((10.0 - costmap.info.origin.position.x) / 0.5));
  const int obstacle_row = static_cast<int>(std::floor((0.5 - costmap.info.origin.position.y) / 0.5));
  costmap.data[static_cast<size_t>(obstacle_row) * costmap.info.width + obstacle_col] = 100;

  nav_msgs::msg::Odometry odom;
  behaviour_msgs::msg::ExecuteBehaviour behaviour;
  behaviour.behaviour = "cruise";
  path_pub->publish(path);
  odom_pub->publish(odom);
  behaviour_pub->publish(behaviour);

  auto trajectory_future = trajectory_sub->expect_next_message();
  for (int attempt = 0;
       attempt < 10 && trajectory_future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready;
       ++attempt)
  {
    costmap_pub->publish(costmap);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  REQUIRE(trajectory_future.wait_for(std::chrono::seconds(1)) == std::future_status::ready);
  const auto trajectory = trajectory_future.get();
  REQUIRE(trajectory.header.frame_id == "map");
  REQUIRE(trajectory.points.size() == 81);

  const auto nearest_obstacle =
    std::min_element(trajectory.points.begin(), trajectory.points.end(), [](const auto & lhs, const auto & rhs) {
      return std::fabs(lhs.pose.position.y - 8.0) < std::fabs(rhs.pose.position.y - 8.0);
    });
  REQUIRE(nearest_obstacle != trajectory.points.end());
  // Avoidance moves right in base_footprint (negative y), which the 90-degree transform maps to
  // positive map x. This would remain exactly x=4 if the node restored the original path poses.
  REQUIRE(nearest_obstacle->pose.position.x > 4.1);
}

}  // namespace wato

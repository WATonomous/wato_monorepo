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

#include "prediction/tracker_node.hpp"

Tracker::Tracker()
: Node("tracker")
{
  // Subscriber
  subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
    "hd_map_viz", 10,
    std::bind(&Tracker::marker_array_callback, this, std::placeholders::_1));

  // Publisher
  publisher_ = this->create_publisher<autoware_perception_msgs::msg::TrackedObjects>(
    "/prediction/tracked_objects", 10);
}

void Tracker::marker_array_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
  auto tracked_objects = autoware_perception_msgs::msg::TrackedObjects();
  
  // Use core logic to process marker array
  tracker_core_.processMarkerArray(msg, tracked_objects);

  // If the header wasn't set (empty markers), set timestamp now
  if (tracked_objects.header.stamp.sec == 0 && tracked_objects.header.stamp.nanosec == 0) {
    tracked_objects.header.stamp = this->now();
  }

  // Publish tracked objects
  publisher_->publish(tracked_objects);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Tracker>());
  rclcpp::shutdown();
  return 0;
}

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

#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

class TrackingPlaceholderNode : public rclcpp::Node {
public:
  TrackingPlaceholderNode() : Node("tracking_placeholder_node") {
    RCLCPP_INFO(this->get_logger(), "Tracking Placeholder Node started");

    // Publisher for tracked 3D detections
    detections_publisher_ =
        this->create_publisher<vision_msgs::msg::Detection3DArray>(
            "/perception/detections_3D_tracked", 10);

    // Timer to periodically publish placeholder detections
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&TrackingPlaceholderNode::timer_callback, this));
  }

private:
  void timer_callback() {
    auto msg = vision_msgs::msg::Detection3DArray();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "base_link";

    // Static test detections
    msg.detections = create_test_detections();

    detections_publisher_->publish(msg);
    RCLCPP_DEBUG(this->get_logger(), "Published %zu tracked detections",
                 msg.detections.size());
  }

  std::vector<vision_msgs::msg::Detection3D> create_test_detections() {
    std::vector<vision_msgs::msg::Detection3D> detections;

    // Detection 1: Vehicle ahead
    {
      vision_msgs::msg::Detection3D det;
      det.header.stamp = this->get_clock()->now();
      det.header.frame_id = "base_link";

      // Center position
      det.bbox.center.position.x = 10.0;
      det.bbox.center.position.y = 0.0;
      det.bbox.center.position.z = 0.75;
      det.bbox.center.orientation.w = 1.0;

      // Dimensions
      det.bbox.size.x = 4.5; // length
      det.bbox.size.y = 1.8; // width
      det.bbox.size.z = 1.5; // height

      detections.push_back(det);
    }

    // Detection 2: Vehicle to the left
    {
      vision_msgs::msg::Detection3D det;
      det.header.stamp = this->get_clock()->now();
      det.header.frame_id = "base_link";

      det.bbox.center.position.x = 15.0;
      det.bbox.center.position.y = 3.5;
      det.bbox.center.position.z = 0.75;
      det.bbox.center.orientation.w = 1.0;

      det.bbox.size.x = 4.5;
      det.bbox.size.y = 1.8;
      det.bbox.size.z = 1.5;

      detections.push_back(det);
    }

    // Detection 3: Pedestrian on the right
    {
      vision_msgs::msg::Detection3D det;
      det.header.stamp = this->get_clock()->now();
      det.header.frame_id = "base_link";

      det.bbox.center.position.x = 8.0;
      det.bbox.center.position.y = -2.0;
      det.bbox.center.position.z = 0.9;
      det.bbox.center.orientation.w = 1.0;

      // Pedestrian dimensions
      det.bbox.size.x = 0.5;
      det.bbox.size.y = 0.5;
      det.bbox.size.z = 1.8;

      detections.push_back(det);
    }

    return detections;
  }

  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr
      detections_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrackingPlaceholderNode>());
  rclcpp::shutdown();
  return 0;
}

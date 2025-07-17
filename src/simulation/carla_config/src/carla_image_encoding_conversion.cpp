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

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
using std::placeholders::_1;

class Image_Encoding_Conversion : public rclcpp::Node
{
public:
  Image_Encoding_Conversion()
  : Node("image_encoding_conversion_node")
  {
    // Wait for service
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "set_initial_pose");
    while (!parameters_client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    // Check if role name specified
    std::string role_name;
    std::string param_name = "role_name";
    do {
      role_name = parameters_client->get_parameters({"role_name"})[0].get_value<std::string>();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } while (role_name == "");

    // Declare Parameters
    this->declare_parameter("camera_out_center_topic", "/carla/" + role_name + "/rgb_center/image_bgr");
    this->declare_parameter("camera_out_right_topic", "/carla/" + role_name + "/rgb_right/image_bgr");
    this->declare_parameter("camera_out_left_topic", "/carla/" + role_name + "/rgb_left/image_bgr");

    // Get Parameters
    std::string camera_out_center_topic = this->get_parameter("camera_out_center_topic").as_string();
    std::string camera_out_right_topic = this->get_parameter("camera_out_right_topic").as_string();
    std::string camera_out_left_topic = this->get_parameter("camera_out_left_topic").as_string();

    this->imagePubCenter = this->create_publisher<sensor_msgs::msg::Image>(camera_out_center_topic, 10);
    imageSubCenter = this->create_subscription<sensor_msgs::msg::Image>(
      "/carla/" + role_name + "/rgb_center/image",
      10,
      std::bind(&Image_Encoding_Conversion::update_image_center, this, _1));
    this->imagePubRight = this->create_publisher<sensor_msgs::msg::Image>(camera_out_right_topic, 10);
    imageSubRight = this->create_subscription<sensor_msgs::msg::Image>(
      "/carla/" + role_name + "/rgb_right/image",
      10,
      std::bind(&Image_Encoding_Conversion::update_image_right, this, _1));
    this->imagePubLeft = this->create_publisher<sensor_msgs::msg::Image>(camera_out_left_topic, 10);
    imageSubLeft = this->create_subscription<sensor_msgs::msg::Image>(
      "/carla/" + role_name + "/rgb_left/image",
      10,
      std::bind(&Image_Encoding_Conversion::update_image_left, this, _1));
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePubCenter;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubCenter;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePubRight;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubRight;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePubLeft;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubLeft;

  void update_image_center(sensor_msgs::msg::Image::SharedPtr image)
  {
    sensor_msgs::msg::Image updatedImage;
    update_image(updatedImage, image);
    imagePubCenter->publish(updatedImage);
  }

  void update_image_right(sensor_msgs::msg::Image::SharedPtr image)
  {
    sensor_msgs::msg::Image updatedImage;
    update_image(updatedImage, image);
    imagePubRight->publish(updatedImage);
  }

  void update_image_left(sensor_msgs::msg::Image::SharedPtr image)
  {
    sensor_msgs::msg::Image updatedImage;
    update_image(updatedImage, image);
    imagePubLeft->publish(updatedImage);
  }

  void update_image(sensor_msgs::msg::Image & updatedImage, sensor_msgs::msg::Image::SharedPtr image)
  {
    updatedImage.header = image->header;
    updatedImage.height = image->height;
    updatedImage.width = image->width;
    updatedImage.encoding = "bgr8";
    updatedImage.is_bigendian = image->is_bigendian;
    updatedImage.step = (image->step * 3) / 4;
    updatedImage.data = std::vector<unsigned char>();

    for (size_t i = 0; i < image->data.size(); i += 4) {
      // Take only BGR values (skip Alpha value)
      updatedImage.data.push_back(image->data[i + 0]);
      updatedImage.data.push_back(image->data[i + 1]);
      updatedImage.data.push_back(image->data[i + 2]);
    }
  }
};

int main(int argc, char ** argv)
{
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Create Node
  auto node = std::make_shared<Image_Encoding_Conversion>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}

#include <chrono>
#include <memory>

#include "camera_compression_node.hpp"

CameraCompressionNode::CameraCompressionNode(std::vector<std::string> image_topics, int publish_freq_ms)
: Node("camera_compression"), image_topics_(image_topics) {
  
  for(int i = 0; i < this->image_topics_.size(); i++){
    std::function<void(const sensor_msgs::msg::Image::SharedPtr msg)> bound_callback_func = std::bind(&CameraCompressionNode::image_sub_callback, this, i, std::placeholders::_1);
    auto sub = this->create_subscription<sensor_msgs::msg::Image>(image_topics[i], 20, bound_callback_func);

    this->image_subscribers_.push_back(sub);

    this->carla_images_.push_back(sensor_msgs::msg::Image());
  }

  read_carla_image_timer_ = this->create_wall_timer(std::chrono::milliseconds(publish_freq_ms), std::bind(&CameraCompressionNode::timer_callback, this));

}

void CameraCompressionNode::image_sub_callback(int i, const sensor_msgs::msg::Image::SharedPtr msg){
  this->carla_images_[i] = *msg;
}

void CameraCompressionNode::timer_callback(){
  for(int i = 0; i < this->image_topics_.size(); i++){
    image_publishers_[i].publish(this->carla_images_[i]);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::vector<std::string> image_topics = {"carla/ego_vehicle/rgb_view/image"};
  int publish_freq_ms = 50;
  auto node = std::make_shared<CameraCompressionNode>(image_topics, publish_freq_ms);

  image_transport::ImageTransport it(node);

  for(auto topic : image_topics){
    node->image_publishers_.push_back(it.advertise(topic + "/reduced", 1));
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

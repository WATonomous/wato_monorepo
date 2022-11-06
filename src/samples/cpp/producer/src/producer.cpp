#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "producer.hpp"

namespace samples
{

Producer::Producer(int x, int y, int z)
: pos_x_(x), pos_y_(y), pos_z_(z), velocity_(0)
{
}

void Producer::set_velocity(int velocity)
{
  velocity_ = velocity;
}

void Producer::update_coordinates()
{
  pos_x_ += velocity_;
  pos_y_ += velocity_;
  pos_z_ += velocity_;
}

void Producer::serialize_coordinates(sample_msgs::msg::Unfiltered & msg) const
{
  msg.data = "x:" + std::to_string(pos_x_) + ";y:" + std::to_string(pos_y_) +
    ";z:" + std::to_string(pos_z_) + ";";
  msg.valid = true;
}

}  // namespace samples

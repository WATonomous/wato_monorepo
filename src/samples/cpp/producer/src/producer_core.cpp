#include <chrono>
#include <cmath>

#include "producer_core.hpp"

namespace samples
{

ProducerCore::ProducerCore(float x, float y, float z)
: pos_x_(x), pos_y_(y), pos_z_(z), velocity_(0)
{
}

void ProducerCore::update_velocity(int velocity)
{
  velocity_ = velocity;
}

void ProducerCore::update_position(double pos_x, double pos_y, double pos_z)
{
  pos_x_ = pos_x;
  pos_y_ = pos_y;
  pos_z_ = pos_z;
}

void ProducerCore::update_coordinates()
{
  pos_x_ += velocity_ / sqrt(3);
  pos_y_ += velocity_ / sqrt(3);
  pos_z_ += velocity_ / sqrt(3);
}

void ProducerCore::serialize_coordinates(sample_msgs::msg::Unfiltered & msg) const
{
  msg.data = "x:" + std::to_string(pos_x_) + ";y:" + std::to_string(pos_y_) +
    ";z:" + std::to_string(pos_z_) + ";";
  msg.valid = true;
}

}  // namespace samples

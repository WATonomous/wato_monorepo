#include <chrono>
#include <cmath>

#include "occupancy_segmentation_core.hpp"

namespace world_modeling
{

// OccupancySegmentationCore::OccupancySegmentationCore(float x, float y, float z)
// : pos_x_(x), pos_y_(y), pos_z_(z), velocity_(0)
// {
// }

// void OccupancySegmentationCore::update_velocity(int velocity)
// {
//   velocity_ = velocity;
// }

// void OccupancySegmentationCore::update_position(double pos_x, double pos_y, double pos_z)
// {
//   pos_x_ = pos_x;
//   pos_y_ = pos_y;
//   pos_z_ = pos_z;
// }

// void OccupancySegmentationCore::update_coordinates()
// {
//   pos_x_ += velocity_ / sqrt(3);
//   pos_y_ += velocity_ / sqrt(3);
//   pos_z_ += velocity_ / sqrt(3);
// }

// void OccupancySegmentationCore::serialize_coordinates(sample_msgs::msg::Unfiltered & msg) const
// {
//   msg.data = "x:" + std::to_string(pos_x_) + ";y:" + std::to_string(pos_y_) +
//     ";z:" + std::to_string(pos_z_) + ";";
//   msg.valid = true;
// }

}

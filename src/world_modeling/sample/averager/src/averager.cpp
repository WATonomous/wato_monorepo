#include <algorithm>

#include "averager.hpp"

namespace world_modeling
{

Averager::Averager()
{}

void Averager::average_msg(
  const sample_msgs::msg::FilteredArray::SharedPtr msg, 
  sample_msgs::msg::FilteredArrayAverage & avg_msg)
{
  double avg_x = 0.0;
  double avg_y = 0.0;
  double avg_z = 0.0;
  
  int elements = 0;
  for (auto filtered_msg : msg->packets) {
    elements ++;
    avg_x += filtered_msg.pos_x;
    avg_y += filtered_msg.pos_y;
    avg_z += filtered_msg.pos_z;
  }

  avg_x /= elements;
  avg_y /= elements;
  avg_z /= elements;
  
  avg_msg.filtered_array = *msg;
  
  avg_msg.x_avg = avg_x;
  avg_msg.y_avg = avg_y;
  avg_msg.z_avg = avg_z;
}

}  // namespace world_modeling

#include <string>
#include <vector>

#include "average_filter.hpp"

namespace samples
{

AverageFilter::AverageFilter()
{}

// std::vector<sample_msgs::msg::FilteredArray> AverageFilter::buffer_messages() const
// {
//   return buffer_;
// }

// void AverageFilter::clear_buffer()
// {
//   buffer_.clear();
// }

// bool AverageFilter::enqueue_message(const sample_msgs::msg::FilteredArrayAverage & msg)
// {
//   if (buffer_.size() < BUFFER_CAPACITY) {
//     buffer_.push_back(msg);
//   }
//   return buffer_.size() == BUFFER_CAPACITY;
// }

bool AverageFilter::average_coordinate(
  const sample_msgs::msg::FilteredArray::SharedPtr filteredArray,
  sample_msgs::msg::FilteredArrayAverage & filteredArrayAverage)
{
  // memcpy(filteredArray.packets, filteredArrayAverage.array, sizeof(filteredArray.packets));
  filteredArrayAverage.array = filteredArray->packets;

  auto total_x = 0;
  auto total_y = 0;
  auto total_z = 0;
  auto total = 0;

  for (auto & pos : filteredArrayAverage.array) {
      total = total + 1;
      total_x = total_x + pos.pos_x;
      total_y = total_y + pos.pos_y;
      total_z = total_z + pos.pos_z;
  }

  filteredArrayAverage.avg_x = total_x/total;
  filteredArrayAverage.avg_y = total_y/total;
  filteredArrayAverage.avg_z = total_z/total;
  return true;
}

}  // namespace samples

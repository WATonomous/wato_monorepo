#include <string>
#include <vector>

#include "occupancy_segmentation_core.hpp"

namespace samples {

OccupancySegmentationCore::OccupancySegmentationCore() {}

std::vector<sample_msgs::msg::Filtered> OccupancySegmentationCore::buffer_messages() const { return buffer_; }

void OccupancySegmentationCore::clear_buffer() { buffer_.clear(); }

bool OccupancySegmentationCore::validate_message(const sample_msgs::msg::Unfiltered::SharedPtr unfiltered) {
  return unfiltered->valid;
}

bool OccupancySegmentationCore::enqueue_message(const sample_msgs::msg::Filtered& msg) {
  if (buffer_.size() < BUFFER_CAPACITY) {
    buffer_.push_back(msg);
  }
  return buffer_.size() == BUFFER_CAPACITY;
}

bool OccupancySegmentationCore::deserialize_coordinate(
    const sample_msgs::msg::Unfiltered::SharedPtr unfiltered,
    sample_msgs::msg::Filtered& filtered) {
  std::string serialized_position = unfiltered->data;
  auto start_pos = serialized_position.find("x:");
  auto end_pos = serialized_position.find(";");
  // Validate that the substrings were found
  if (start_pos == std::string::npos || end_pos == std::string::npos || end_pos < start_pos) {
    return false;
  }
  // Offset index to start of x_pos
  start_pos += 2;
  // Splice string and convert position to float
  float x = std::stof(serialized_position.substr(start_pos, end_pos - start_pos));

  start_pos = serialized_position.find("y:", end_pos + 1);
  end_pos = serialized_position.find(";", end_pos + 1);
  if (start_pos == std::string::npos || end_pos == std::string::npos || end_pos < start_pos) {
    return false;
  }
  // Offset index to start of y_pos
  start_pos += 2;
  float y = std::stof(serialized_position.substr(start_pos, end_pos - start_pos));

  start_pos = serialized_position.find("z:", end_pos + 1);
  end_pos = serialized_position.find(";", end_pos + 1);
  if (start_pos == std::string::npos || end_pos == std::string::npos || end_pos < start_pos) {
    return false;
  }
  // Offset index to start of z_pos
  start_pos += 2;
  float z = std::stof(serialized_position.substr(start_pos, end_pos - start_pos));

  filtered.pos_x = x;
  filtered.pos_y = y;
  filtered.pos_z = z;
  return true;
}

}  // namespace samples

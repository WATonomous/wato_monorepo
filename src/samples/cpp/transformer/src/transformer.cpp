#include <string>

#include "isEven.h"
#include "transformer.hpp"

Transformer::Transformer()
: Node("transformer")
{
  raw_sub_ = this->create_subscription<transformer::msg::Unfiltered>(
    "unfiltered", ADVERTISING_FREQ,
    std::bind(
      &Transformer::unfiltered_callback, this,
      std::placeholders::_1));
  transform_pub_ =
    this->create_publisher<transformer::msg::FilteredArray>("filtered", ADVERTISING_FREQ);

  // Define the default values for parameters if not defined in params.yaml
  this->declare_parameter("version", rclcpp::ParameterValue(0));
  this->declare_parameter("compression_method", rclcpp::ParameterValue(0));
}

void Transformer::unfiltered_callback(const transformer::msg::Unfiltered::SharedPtr msg)
{
  auto filtered_msg = transformer::msg::Filtered();
  bool valid = filtered_transformer(msg, filtered_msg);
  // Filter out messages that are invalid
  if (valid) {
    publish_buffer(filtered_msg);
  }
}

bool Transformer::filtered_transformer(
  const transformer::msg::Unfiltered::SharedPtr unfiltered,
  transformer::msg::Filtered & filtered)
{
  // Filter messages that are invalid or have an odd timestamp
  if (!unfiltered->valid || !_isEven(unfiltered->timestamp % 2)) {
    return false;
  }

  // Filter messages where serialized position data is corrupt
  if (!deserialize_position(unfiltered, filtered)) {
    return false;
  }
  filtered.timestamp = unfiltered->timestamp;
  filtered.metadata.version = this->get_parameter("version").as_int();
  filtered.metadata.compression_method =
    this->get_parameter("compression_method").as_int();
  return true;
}

void Transformer::publish_buffer(const transformer::msg::Filtered & msg)
{
  buffer_.push_back(msg);
  // Publish processed data when the buffer reaches its capacity
  if (buffer_.size() == BUFFER_CAPACITY) {
    transformer::msg::FilteredArray filtered_msgs;
    // Construct FilteredArray object
    for (auto & packet : buffer_) {
      filtered_msgs.packets.push_back(packet);
    }

    transform_pub_->publish(filtered_msgs);
    buffer_.clear();
  }
}

bool Transformer::deserialize_position(
  const transformer::msg::Unfiltered::SharedPtr unfiltered,
  transformer::msg::Filtered & filtered)
{
  std::string serialized_position = unfiltered->data;
  auto start_pos = serialized_position.find("x:");
  auto end_pos = serialized_position.find(";");
  // Validate that the substrings were found
  if (start_pos == std::string::npos || end_pos == std::string::npos ||
    end_pos < start_pos)
  {
    return false;
  }
  // Offset index to start of x_pos
  start_pos += 2;
  // Splice string and convert position to float
  float x = std::stof(serialized_position.substr(start_pos, end_pos - start_pos));

  start_pos = serialized_position.find("y:", end_pos + 1);
  end_pos = serialized_position.find(";", end_pos + 1);
  if (start_pos == std::string::npos || end_pos == std::string::npos ||
    end_pos < start_pos)
  {
    return false;
  }
  // Offset index to start of y_pos
  start_pos += 2;
  float y = std::stof(serialized_position.substr(start_pos, end_pos - start_pos));

  start_pos = serialized_position.find("z:", end_pos + 1);
  end_pos = serialized_position.find(";", end_pos + 1);
  if (start_pos == std::string::npos || end_pos == std::string::npos ||
    end_pos < start_pos)
  {
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

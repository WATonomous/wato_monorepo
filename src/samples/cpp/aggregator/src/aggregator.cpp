#include <algorithm>

#include "aggregator.hpp"

namespace samples
{

Aggregator::Aggregator(int64_t timestamp)
: raw_msg_count_(0), filtered_msg_count_(0), start_(timestamp),
  latest_raw_time_(-1), latest_filtered_time_(-1)
{
}

double Aggregator::raw_frequency() const
{
  if (latest_raw_time_ <= start_) {
    return 0.0;
  }
  return static_cast<double>(raw_msg_count_) / (latest_raw_time_ - start_);
}

double Aggregator::filtered_frequency() const
{
  if (latest_filtered_time_ <= start_) {
    return 0.0;
  }
  return static_cast<double>(filtered_msg_count_) / (latest_filtered_time_ - start_);
}

void Aggregator::add_raw_msg(
  const sample_msgs::msg::Unfiltered::SharedPtr msg)
{
  latest_raw_time_ = std::max(
    static_cast<int64_t>(msg->timestamp), latest_raw_time_);
  raw_msg_count_++;
}

void Aggregator::add_filtered_msg(
  const sample_msgs::msg::FilteredArray::SharedPtr msg)
{
  for (auto filtered_msg : msg->packets) {
    latest_filtered_time_ = std::max(
      static_cast<int64_t>(filtered_msg.timestamp), latest_filtered_time_);
  }
  filtered_msg_count_++;
}

}  // namespace samples

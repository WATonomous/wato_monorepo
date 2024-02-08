#include <algorithm>

#include "aggregator_core.hpp"

namespace samples
{

AggregatorCore::AggregatorCore(int64_t timestamp)
: raw_msg_count_(0), filtered_msg_count_(0), start_(timestamp),
  latest_raw_time_(-1), latest_filtered_time_(-1)
{}

double AggregatorCore::raw_frequency() const
{
  if (latest_raw_time_ <= start_) {
    return 0.0;
  }
  return static_cast<double>(raw_msg_count_) / (latest_raw_time_ - start_);
}

double AggregatorCore::filtered_frequency() const
{
  if (latest_filtered_time_ <= start_) {
    return 0.0;
  }
  return static_cast<double>(filtered_msg_count_) / (latest_filtered_time_ - start_);
}

void AggregatorCore::add_raw_msg(
  const sample_msgs::msg::Unfiltered::SharedPtr msg)
{
  latest_raw_time_ = std::max(
    static_cast<int64_t>(msg->timestamp), latest_raw_time_);
  raw_msg_count_++;
}

void AggregatorCore::add_filtered_msg(
  const sample_msgs::msg::FilteredArray::SharedPtr msg)
{
  for (auto filtered_msg : msg->packets) {
    latest_filtered_time_ = std::max(
      static_cast<int64_t>(filtered_msg.timestamp), latest_filtered_time_);
  }
  filtered_msg_count_++;
}

}  // namespace samples

// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sample_msgs:msg/FilteredArray.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_MSGS__MSG__DETAIL__FILTERED_ARRAY__BUILDER_HPP_
#define SAMPLE_MSGS__MSG__DETAIL__FILTERED_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sample_msgs/msg/detail/filtered_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sample_msgs
{

namespace msg
{

namespace builder
{

class Init_FilteredArray_packets
{
public:
  Init_FilteredArray_packets()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::sample_msgs::msg::FilteredArray packets(::sample_msgs::msg::FilteredArray::_packets_type arg)
  {
    msg_.packets = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sample_msgs::msg::FilteredArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sample_msgs::msg::FilteredArray>()
{
  return sample_msgs::msg::builder::Init_FilteredArray_packets();
}

}  // namespace sample_msgs

#endif  // SAMPLE_MSGS__MSG__DETAIL__FILTERED_ARRAY__BUILDER_HPP_

// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sample_msgs:msg/Unfiltered.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_MSGS__MSG__DETAIL__UNFILTERED__BUILDER_HPP_
#define SAMPLE_MSGS__MSG__DETAIL__UNFILTERED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sample_msgs/msg/detail/unfiltered__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sample_msgs
{

namespace msg
{

namespace builder
{

class Init_Unfiltered_valid
{
public:
  explicit Init_Unfiltered_valid(::sample_msgs::msg::Unfiltered & msg)
  : msg_(msg)
  {}
  ::sample_msgs::msg::Unfiltered valid(::sample_msgs::msg::Unfiltered::_valid_type arg)
  {
    msg_.valid = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sample_msgs::msg::Unfiltered msg_;
};

class Init_Unfiltered_timestamp
{
public:
  explicit Init_Unfiltered_timestamp(::sample_msgs::msg::Unfiltered & msg)
  : msg_(msg)
  {}
  Init_Unfiltered_valid timestamp(::sample_msgs::msg::Unfiltered::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_Unfiltered_valid(msg_);
  }

private:
  ::sample_msgs::msg::Unfiltered msg_;
};

class Init_Unfiltered_data
{
public:
  Init_Unfiltered_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Unfiltered_timestamp data(::sample_msgs::msg::Unfiltered::_data_type arg)
  {
    msg_.data = std::move(arg);
    return Init_Unfiltered_timestamp(msg_);
  }

private:
  ::sample_msgs::msg::Unfiltered msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sample_msgs::msg::Unfiltered>()
{
  return sample_msgs::msg::builder::Init_Unfiltered_data();
}

}  // namespace sample_msgs

#endif  // SAMPLE_MSGS__MSG__DETAIL__UNFILTERED__BUILDER_HPP_

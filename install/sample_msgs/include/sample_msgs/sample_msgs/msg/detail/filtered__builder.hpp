// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sample_msgs:msg/Filtered.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_MSGS__MSG__DETAIL__FILTERED__BUILDER_HPP_
#define SAMPLE_MSGS__MSG__DETAIL__FILTERED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sample_msgs/msg/detail/filtered__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sample_msgs
{

namespace msg
{

namespace builder
{

class Init_Filtered_timestamp
{
public:
  explicit Init_Filtered_timestamp(::sample_msgs::msg::Filtered & msg)
  : msg_(msg)
  {}
  ::sample_msgs::msg::Filtered timestamp(::sample_msgs::msg::Filtered::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sample_msgs::msg::Filtered msg_;
};

class Init_Filtered_metadata
{
public:
  explicit Init_Filtered_metadata(::sample_msgs::msg::Filtered & msg)
  : msg_(msg)
  {}
  Init_Filtered_timestamp metadata(::sample_msgs::msg::Filtered::_metadata_type arg)
  {
    msg_.metadata = std::move(arg);
    return Init_Filtered_timestamp(msg_);
  }

private:
  ::sample_msgs::msg::Filtered msg_;
};

class Init_Filtered_pos_z
{
public:
  explicit Init_Filtered_pos_z(::sample_msgs::msg::Filtered & msg)
  : msg_(msg)
  {}
  Init_Filtered_metadata pos_z(::sample_msgs::msg::Filtered::_pos_z_type arg)
  {
    msg_.pos_z = std::move(arg);
    return Init_Filtered_metadata(msg_);
  }

private:
  ::sample_msgs::msg::Filtered msg_;
};

class Init_Filtered_pos_y
{
public:
  explicit Init_Filtered_pos_y(::sample_msgs::msg::Filtered & msg)
  : msg_(msg)
  {}
  Init_Filtered_pos_z pos_y(::sample_msgs::msg::Filtered::_pos_y_type arg)
  {
    msg_.pos_y = std::move(arg);
    return Init_Filtered_pos_z(msg_);
  }

private:
  ::sample_msgs::msg::Filtered msg_;
};

class Init_Filtered_pos_x
{
public:
  Init_Filtered_pos_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Filtered_pos_y pos_x(::sample_msgs::msg::Filtered::_pos_x_type arg)
  {
    msg_.pos_x = std::move(arg);
    return Init_Filtered_pos_y(msg_);
  }

private:
  ::sample_msgs::msg::Filtered msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sample_msgs::msg::Filtered>()
{
  return sample_msgs::msg::builder::Init_Filtered_pos_x();
}

}  // namespace sample_msgs

#endif  // SAMPLE_MSGS__MSG__DETAIL__FILTERED__BUILDER_HPP_

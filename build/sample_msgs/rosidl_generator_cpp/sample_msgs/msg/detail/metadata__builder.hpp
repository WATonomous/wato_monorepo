// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sample_msgs:msg/Metadata.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_MSGS__MSG__DETAIL__METADATA__BUILDER_HPP_
#define SAMPLE_MSGS__MSG__DETAIL__METADATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sample_msgs/msg/detail/metadata__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sample_msgs
{

namespace msg
{

namespace builder
{

class Init_Metadata_creation_date
{
public:
  explicit Init_Metadata_creation_date(::sample_msgs::msg::Metadata & msg)
  : msg_(msg)
  {}
  ::sample_msgs::msg::Metadata creation_date(::sample_msgs::msg::Metadata::_creation_date_type arg)
  {
    msg_.creation_date = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sample_msgs::msg::Metadata msg_;
};

class Init_Metadata_compression_method
{
public:
  explicit Init_Metadata_compression_method(::sample_msgs::msg::Metadata & msg)
  : msg_(msg)
  {}
  Init_Metadata_creation_date compression_method(::sample_msgs::msg::Metadata::_compression_method_type arg)
  {
    msg_.compression_method = std::move(arg);
    return Init_Metadata_creation_date(msg_);
  }

private:
  ::sample_msgs::msg::Metadata msg_;
};

class Init_Metadata_version
{
public:
  Init_Metadata_version()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Metadata_compression_method version(::sample_msgs::msg::Metadata::_version_type arg)
  {
    msg_.version = std::move(arg);
    return Init_Metadata_compression_method(msg_);
  }

private:
  ::sample_msgs::msg::Metadata msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sample_msgs::msg::Metadata>()
{
  return sample_msgs::msg::builder::Init_Metadata_version();
}

}  // namespace sample_msgs

#endif  // SAMPLE_MSGS__MSG__DETAIL__METADATA__BUILDER_HPP_

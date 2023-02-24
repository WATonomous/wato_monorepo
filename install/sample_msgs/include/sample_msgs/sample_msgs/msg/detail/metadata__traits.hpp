// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sample_msgs:msg/Metadata.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_MSGS__MSG__DETAIL__METADATA__TRAITS_HPP_
#define SAMPLE_MSGS__MSG__DETAIL__METADATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sample_msgs/msg/detail/metadata__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace sample_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Metadata & msg,
  std::ostream & out)
{
  out << "{";
  // member: version
  {
    out << "version: ";
    rosidl_generator_traits::value_to_yaml(msg.version, out);
    out << ", ";
  }

  // member: compression_method
  {
    out << "compression_method: ";
    rosidl_generator_traits::value_to_yaml(msg.compression_method, out);
    out << ", ";
  }

  // member: creation_date
  {
    out << "creation_date: ";
    rosidl_generator_traits::value_to_yaml(msg.creation_date, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Metadata & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: version
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "version: ";
    rosidl_generator_traits::value_to_yaml(msg.version, out);
    out << "\n";
  }

  // member: compression_method
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "compression_method: ";
    rosidl_generator_traits::value_to_yaml(msg.compression_method, out);
    out << "\n";
  }

  // member: creation_date
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "creation_date: ";
    rosidl_generator_traits::value_to_yaml(msg.creation_date, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Metadata & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace sample_msgs

namespace rosidl_generator_traits
{

[[deprecated("use sample_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const sample_msgs::msg::Metadata & msg,
  std::ostream & out, size_t indentation = 0)
{
  sample_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sample_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const sample_msgs::msg::Metadata & msg)
{
  return sample_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<sample_msgs::msg::Metadata>()
{
  return "sample_msgs::msg::Metadata";
}

template<>
inline const char * name<sample_msgs::msg::Metadata>()
{
  return "sample_msgs/msg/Metadata";
}

template<>
struct has_fixed_size<sample_msgs::msg::Metadata>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<sample_msgs::msg::Metadata>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<sample_msgs::msg::Metadata>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SAMPLE_MSGS__MSG__DETAIL__METADATA__TRAITS_HPP_

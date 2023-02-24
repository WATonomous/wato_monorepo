// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sample_msgs:msg/Unfiltered.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_MSGS__MSG__DETAIL__UNFILTERED__TRAITS_HPP_
#define SAMPLE_MSGS__MSG__DETAIL__UNFILTERED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sample_msgs/msg/detail/unfiltered__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace sample_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Unfiltered & msg,
  std::ostream & out)
{
  out << "{";
  // member: data
  {
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
    out << ", ";
  }

  // member: timestamp
  {
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << ", ";
  }

  // member: valid
  {
    out << "valid: ";
    rosidl_generator_traits::value_to_yaml(msg.valid, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Unfiltered & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
    out << "\n";
  }

  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << "\n";
  }

  // member: valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "valid: ";
    rosidl_generator_traits::value_to_yaml(msg.valid, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Unfiltered & msg, bool use_flow_style = false)
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
  const sample_msgs::msg::Unfiltered & msg,
  std::ostream & out, size_t indentation = 0)
{
  sample_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sample_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const sample_msgs::msg::Unfiltered & msg)
{
  return sample_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<sample_msgs::msg::Unfiltered>()
{
  return "sample_msgs::msg::Unfiltered";
}

template<>
inline const char * name<sample_msgs::msg::Unfiltered>()
{
  return "sample_msgs/msg/Unfiltered";
}

template<>
struct has_fixed_size<sample_msgs::msg::Unfiltered>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<sample_msgs::msg::Unfiltered>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<sample_msgs::msg::Unfiltered>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SAMPLE_MSGS__MSG__DETAIL__UNFILTERED__TRAITS_HPP_

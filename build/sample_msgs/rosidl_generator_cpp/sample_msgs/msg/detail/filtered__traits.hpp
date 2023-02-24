// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sample_msgs:msg/Filtered.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_MSGS__MSG__DETAIL__FILTERED__TRAITS_HPP_
#define SAMPLE_MSGS__MSG__DETAIL__FILTERED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sample_msgs/msg/detail/filtered__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'metadata'
#include "sample_msgs/msg/detail/metadata__traits.hpp"

namespace sample_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Filtered & msg,
  std::ostream & out)
{
  out << "{";
  // member: pos_x
  {
    out << "pos_x: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_x, out);
    out << ", ";
  }

  // member: pos_y
  {
    out << "pos_y: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_y, out);
    out << ", ";
  }

  // member: pos_z
  {
    out << "pos_z: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_z, out);
    out << ", ";
  }

  // member: metadata
  {
    out << "metadata: ";
    to_flow_style_yaml(msg.metadata, out);
    out << ", ";
  }

  // member: timestamp
  {
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Filtered & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pos_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pos_x: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_x, out);
    out << "\n";
  }

  // member: pos_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pos_y: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_y, out);
    out << "\n";
  }

  // member: pos_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pos_z: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_z, out);
    out << "\n";
  }

  // member: metadata
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "metadata:\n";
    to_block_style_yaml(msg.metadata, out, indentation + 2);
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Filtered & msg, bool use_flow_style = false)
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
  const sample_msgs::msg::Filtered & msg,
  std::ostream & out, size_t indentation = 0)
{
  sample_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sample_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const sample_msgs::msg::Filtered & msg)
{
  return sample_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<sample_msgs::msg::Filtered>()
{
  return "sample_msgs::msg::Filtered";
}

template<>
inline const char * name<sample_msgs::msg::Filtered>()
{
  return "sample_msgs/msg/Filtered";
}

template<>
struct has_fixed_size<sample_msgs::msg::Filtered>
  : std::integral_constant<bool, has_fixed_size<sample_msgs::msg::Metadata>::value> {};

template<>
struct has_bounded_size<sample_msgs::msg::Filtered>
  : std::integral_constant<bool, has_bounded_size<sample_msgs::msg::Metadata>::value> {};

template<>
struct is_message<sample_msgs::msg::Filtered>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SAMPLE_MSGS__MSG__DETAIL__FILTERED__TRAITS_HPP_

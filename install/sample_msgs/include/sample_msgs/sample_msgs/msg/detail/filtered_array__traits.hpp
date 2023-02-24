// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sample_msgs:msg/FilteredArray.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_MSGS__MSG__DETAIL__FILTERED_ARRAY__TRAITS_HPP_
#define SAMPLE_MSGS__MSG__DETAIL__FILTERED_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sample_msgs/msg/detail/filtered_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'packets'
#include "sample_msgs/msg/detail/filtered__traits.hpp"

namespace sample_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const FilteredArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: packets
  {
    if (msg.packets.size() == 0) {
      out << "packets: []";
    } else {
      out << "packets: [";
      size_t pending_items = msg.packets.size();
      for (auto item : msg.packets) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FilteredArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: packets
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.packets.size() == 0) {
      out << "packets: []\n";
    } else {
      out << "packets:\n";
      for (auto item : msg.packets) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FilteredArray & msg, bool use_flow_style = false)
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
  const sample_msgs::msg::FilteredArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  sample_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sample_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const sample_msgs::msg::FilteredArray & msg)
{
  return sample_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<sample_msgs::msg::FilteredArray>()
{
  return "sample_msgs::msg::FilteredArray";
}

template<>
inline const char * name<sample_msgs::msg::FilteredArray>()
{
  return "sample_msgs/msg/FilteredArray";
}

template<>
struct has_fixed_size<sample_msgs::msg::FilteredArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<sample_msgs::msg::FilteredArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<sample_msgs::msg::FilteredArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SAMPLE_MSGS__MSG__DETAIL__FILTERED_ARRAY__TRAITS_HPP_

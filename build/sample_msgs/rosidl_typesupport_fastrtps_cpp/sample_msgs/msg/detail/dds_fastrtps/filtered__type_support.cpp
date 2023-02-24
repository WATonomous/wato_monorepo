// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from sample_msgs:msg/Filtered.idl
// generated code does not contain a copyright notice
#include "sample_msgs/msg/detail/filtered__rosidl_typesupport_fastrtps_cpp.hpp"
#include "sample_msgs/msg/detail/filtered__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace sample_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const sample_msgs::msg::Metadata &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  sample_msgs::msg::Metadata &);
size_t get_serialized_size(
  const sample_msgs::msg::Metadata &,
  size_t current_alignment);
size_t
max_serialized_size_Metadata(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace sample_msgs


namespace sample_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sample_msgs
cdr_serialize(
  const sample_msgs::msg::Filtered & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: pos_x
  cdr << ros_message.pos_x;
  // Member: pos_y
  cdr << ros_message.pos_y;
  // Member: pos_z
  cdr << ros_message.pos_z;
  // Member: metadata
  sample_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.metadata,
    cdr);
  // Member: timestamp
  cdr << ros_message.timestamp;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sample_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  sample_msgs::msg::Filtered & ros_message)
{
  // Member: pos_x
  cdr >> ros_message.pos_x;

  // Member: pos_y
  cdr >> ros_message.pos_y;

  // Member: pos_z
  cdr >> ros_message.pos_z;

  // Member: metadata
  sample_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.metadata);

  // Member: timestamp
  cdr >> ros_message.timestamp;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sample_msgs
get_serialized_size(
  const sample_msgs::msg::Filtered & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: pos_x
  {
    size_t item_size = sizeof(ros_message.pos_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: pos_y
  {
    size_t item_size = sizeof(ros_message.pos_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: pos_z
  {
    size_t item_size = sizeof(ros_message.pos_z);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: metadata

  current_alignment +=
    sample_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.metadata, current_alignment);
  // Member: timestamp
  {
    size_t item_size = sizeof(ros_message.timestamp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sample_msgs
max_serialized_size_Filtered(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: pos_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: pos_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: pos_z
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: metadata
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      current_alignment +=
        sample_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Metadata(
        inner_full_bounded, inner_is_plain, current_alignment);
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: timestamp
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  return current_alignment - initial_alignment;
}

static bool _Filtered__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const sample_msgs::msg::Filtered *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Filtered__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<sample_msgs::msg::Filtered *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Filtered__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const sample_msgs::msg::Filtered *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Filtered__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_Filtered(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _Filtered__callbacks = {
  "sample_msgs::msg",
  "Filtered",
  _Filtered__cdr_serialize,
  _Filtered__cdr_deserialize,
  _Filtered__get_serialized_size,
  _Filtered__max_serialized_size
};

static rosidl_message_type_support_t _Filtered__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Filtered__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace sample_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_sample_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<sample_msgs::msg::Filtered>()
{
  return &sample_msgs::msg::typesupport_fastrtps_cpp::_Filtered__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, sample_msgs, msg, Filtered)() {
  return &sample_msgs::msg::typesupport_fastrtps_cpp::_Filtered__handle;
}

#ifdef __cplusplus
}
#endif

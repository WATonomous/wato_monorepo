// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from sample_msgs:msg/Metadata.idl
// generated code does not contain a copyright notice
#include "sample_msgs/msg/detail/metadata__rosidl_typesupport_fastrtps_cpp.hpp"
#include "sample_msgs/msg/detail/metadata__struct.hpp"

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

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sample_msgs
cdr_serialize(
  const sample_msgs::msg::Metadata & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: version
  cdr << ros_message.version;
  // Member: compression_method
  cdr << ros_message.compression_method;
  // Member: creation_date
  cdr << ros_message.creation_date;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sample_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  sample_msgs::msg::Metadata & ros_message)
{
  // Member: version
  cdr >> ros_message.version;

  // Member: compression_method
  cdr >> ros_message.compression_method;

  // Member: creation_date
  cdr >> ros_message.creation_date;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sample_msgs
get_serialized_size(
  const sample_msgs::msg::Metadata & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: version
  {
    size_t item_size = sizeof(ros_message.version);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: compression_method
  {
    size_t item_size = sizeof(ros_message.compression_method);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: creation_date
  {
    size_t item_size = sizeof(ros_message.creation_date);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sample_msgs
max_serialized_size_Metadata(
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


  // Member: version
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: compression_method
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: creation_date
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  return current_alignment - initial_alignment;
}

static bool _Metadata__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const sample_msgs::msg::Metadata *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Metadata__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<sample_msgs::msg::Metadata *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Metadata__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const sample_msgs::msg::Metadata *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Metadata__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_Metadata(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _Metadata__callbacks = {
  "sample_msgs::msg",
  "Metadata",
  _Metadata__cdr_serialize,
  _Metadata__cdr_deserialize,
  _Metadata__get_serialized_size,
  _Metadata__max_serialized_size
};

static rosidl_message_type_support_t _Metadata__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Metadata__callbacks,
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
get_message_type_support_handle<sample_msgs::msg::Metadata>()
{
  return &sample_msgs::msg::typesupport_fastrtps_cpp::_Metadata__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, sample_msgs, msg, Metadata)() {
  return &sample_msgs::msg::typesupport_fastrtps_cpp::_Metadata__handle;
}

#ifdef __cplusplus
}
#endif

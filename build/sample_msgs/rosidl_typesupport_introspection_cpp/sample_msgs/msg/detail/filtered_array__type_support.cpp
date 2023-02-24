// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from sample_msgs:msg/FilteredArray.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "sample_msgs/msg/detail/filtered_array__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace sample_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void FilteredArray_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) sample_msgs::msg::FilteredArray(_init);
}

void FilteredArray_fini_function(void * message_memory)
{
  auto typed_message = static_cast<sample_msgs::msg::FilteredArray *>(message_memory);
  typed_message->~FilteredArray();
}

size_t size_function__FilteredArray__packets(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<sample_msgs::msg::Filtered> *>(untyped_member);
  return member->size();
}

const void * get_const_function__FilteredArray__packets(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<sample_msgs::msg::Filtered> *>(untyped_member);
  return &member[index];
}

void * get_function__FilteredArray__packets(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<sample_msgs::msg::Filtered> *>(untyped_member);
  return &member[index];
}

void fetch_function__FilteredArray__packets(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const sample_msgs::msg::Filtered *>(
    get_const_function__FilteredArray__packets(untyped_member, index));
  auto & value = *reinterpret_cast<sample_msgs::msg::Filtered *>(untyped_value);
  value = item;
}

void assign_function__FilteredArray__packets(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<sample_msgs::msg::Filtered *>(
    get_function__FilteredArray__packets(untyped_member, index));
  const auto & value = *reinterpret_cast<const sample_msgs::msg::Filtered *>(untyped_value);
  item = value;
}

void resize_function__FilteredArray__packets(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<sample_msgs::msg::Filtered> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember FilteredArray_message_member_array[1] = {
  {
    "packets",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<sample_msgs::msg::Filtered>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sample_msgs::msg::FilteredArray, packets),  // bytes offset in struct
    nullptr,  // default value
    size_function__FilteredArray__packets,  // size() function pointer
    get_const_function__FilteredArray__packets,  // get_const(index) function pointer
    get_function__FilteredArray__packets,  // get(index) function pointer
    fetch_function__FilteredArray__packets,  // fetch(index, &value) function pointer
    assign_function__FilteredArray__packets,  // assign(index, value) function pointer
    resize_function__FilteredArray__packets  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers FilteredArray_message_members = {
  "sample_msgs::msg",  // message namespace
  "FilteredArray",  // message name
  1,  // number of fields
  sizeof(sample_msgs::msg::FilteredArray),
  FilteredArray_message_member_array,  // message members
  FilteredArray_init_function,  // function to initialize message memory (memory has to be allocated)
  FilteredArray_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t FilteredArray_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &FilteredArray_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace sample_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<sample_msgs::msg::FilteredArray>()
{
  return &::sample_msgs::msg::rosidl_typesupport_introspection_cpp::FilteredArray_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, sample_msgs, msg, FilteredArray)() {
  return &::sample_msgs::msg::rosidl_typesupport_introspection_cpp::FilteredArray_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

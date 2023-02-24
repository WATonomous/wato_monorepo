// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from sample_msgs:msg/FilteredArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "sample_msgs/msg/detail/filtered_array__rosidl_typesupport_introspection_c.h"
#include "sample_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "sample_msgs/msg/detail/filtered_array__functions.h"
#include "sample_msgs/msg/detail/filtered_array__struct.h"


// Include directives for member types
// Member `packets`
#include "sample_msgs/msg/filtered.h"
// Member `packets`
#include "sample_msgs/msg/detail/filtered__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void sample_msgs__msg__FilteredArray__rosidl_typesupport_introspection_c__FilteredArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  sample_msgs__msg__FilteredArray__init(message_memory);
}

void sample_msgs__msg__FilteredArray__rosidl_typesupport_introspection_c__FilteredArray_fini_function(void * message_memory)
{
  sample_msgs__msg__FilteredArray__fini(message_memory);
}

size_t sample_msgs__msg__FilteredArray__rosidl_typesupport_introspection_c__size_function__FilteredArray__packets(
  const void * untyped_member)
{
  const sample_msgs__msg__Filtered__Sequence * member =
    (const sample_msgs__msg__Filtered__Sequence *)(untyped_member);
  return member->size;
}

const void * sample_msgs__msg__FilteredArray__rosidl_typesupport_introspection_c__get_const_function__FilteredArray__packets(
  const void * untyped_member, size_t index)
{
  const sample_msgs__msg__Filtered__Sequence * member =
    (const sample_msgs__msg__Filtered__Sequence *)(untyped_member);
  return &member->data[index];
}

void * sample_msgs__msg__FilteredArray__rosidl_typesupport_introspection_c__get_function__FilteredArray__packets(
  void * untyped_member, size_t index)
{
  sample_msgs__msg__Filtered__Sequence * member =
    (sample_msgs__msg__Filtered__Sequence *)(untyped_member);
  return &member->data[index];
}

void sample_msgs__msg__FilteredArray__rosidl_typesupport_introspection_c__fetch_function__FilteredArray__packets(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const sample_msgs__msg__Filtered * item =
    ((const sample_msgs__msg__Filtered *)
    sample_msgs__msg__FilteredArray__rosidl_typesupport_introspection_c__get_const_function__FilteredArray__packets(untyped_member, index));
  sample_msgs__msg__Filtered * value =
    (sample_msgs__msg__Filtered *)(untyped_value);
  *value = *item;
}

void sample_msgs__msg__FilteredArray__rosidl_typesupport_introspection_c__assign_function__FilteredArray__packets(
  void * untyped_member, size_t index, const void * untyped_value)
{
  sample_msgs__msg__Filtered * item =
    ((sample_msgs__msg__Filtered *)
    sample_msgs__msg__FilteredArray__rosidl_typesupport_introspection_c__get_function__FilteredArray__packets(untyped_member, index));
  const sample_msgs__msg__Filtered * value =
    (const sample_msgs__msg__Filtered *)(untyped_value);
  *item = *value;
}

bool sample_msgs__msg__FilteredArray__rosidl_typesupport_introspection_c__resize_function__FilteredArray__packets(
  void * untyped_member, size_t size)
{
  sample_msgs__msg__Filtered__Sequence * member =
    (sample_msgs__msg__Filtered__Sequence *)(untyped_member);
  sample_msgs__msg__Filtered__Sequence__fini(member);
  return sample_msgs__msg__Filtered__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember sample_msgs__msg__FilteredArray__rosidl_typesupport_introspection_c__FilteredArray_message_member_array[1] = {
  {
    "packets",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sample_msgs__msg__FilteredArray, packets),  // bytes offset in struct
    NULL,  // default value
    sample_msgs__msg__FilteredArray__rosidl_typesupport_introspection_c__size_function__FilteredArray__packets,  // size() function pointer
    sample_msgs__msg__FilteredArray__rosidl_typesupport_introspection_c__get_const_function__FilteredArray__packets,  // get_const(index) function pointer
    sample_msgs__msg__FilteredArray__rosidl_typesupport_introspection_c__get_function__FilteredArray__packets,  // get(index) function pointer
    sample_msgs__msg__FilteredArray__rosidl_typesupport_introspection_c__fetch_function__FilteredArray__packets,  // fetch(index, &value) function pointer
    sample_msgs__msg__FilteredArray__rosidl_typesupport_introspection_c__assign_function__FilteredArray__packets,  // assign(index, value) function pointer
    sample_msgs__msg__FilteredArray__rosidl_typesupport_introspection_c__resize_function__FilteredArray__packets  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers sample_msgs__msg__FilteredArray__rosidl_typesupport_introspection_c__FilteredArray_message_members = {
  "sample_msgs__msg",  // message namespace
  "FilteredArray",  // message name
  1,  // number of fields
  sizeof(sample_msgs__msg__FilteredArray),
  sample_msgs__msg__FilteredArray__rosidl_typesupport_introspection_c__FilteredArray_message_member_array,  // message members
  sample_msgs__msg__FilteredArray__rosidl_typesupport_introspection_c__FilteredArray_init_function,  // function to initialize message memory (memory has to be allocated)
  sample_msgs__msg__FilteredArray__rosidl_typesupport_introspection_c__FilteredArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t sample_msgs__msg__FilteredArray__rosidl_typesupport_introspection_c__FilteredArray_message_type_support_handle = {
  0,
  &sample_msgs__msg__FilteredArray__rosidl_typesupport_introspection_c__FilteredArray_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_sample_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sample_msgs, msg, FilteredArray)() {
  sample_msgs__msg__FilteredArray__rosidl_typesupport_introspection_c__FilteredArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sample_msgs, msg, Filtered)();
  if (!sample_msgs__msg__FilteredArray__rosidl_typesupport_introspection_c__FilteredArray_message_type_support_handle.typesupport_identifier) {
    sample_msgs__msg__FilteredArray__rosidl_typesupport_introspection_c__FilteredArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &sample_msgs__msg__FilteredArray__rosidl_typesupport_introspection_c__FilteredArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

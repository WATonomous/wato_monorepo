// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from sample_msgs:msg/Unfiltered.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "sample_msgs/msg/detail/unfiltered__rosidl_typesupport_introspection_c.h"
#include "sample_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "sample_msgs/msg/detail/unfiltered__functions.h"
#include "sample_msgs/msg/detail/unfiltered__struct.h"


// Include directives for member types
// Member `data`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void sample_msgs__msg__Unfiltered__rosidl_typesupport_introspection_c__Unfiltered_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  sample_msgs__msg__Unfiltered__init(message_memory);
}

void sample_msgs__msg__Unfiltered__rosidl_typesupport_introspection_c__Unfiltered_fini_function(void * message_memory)
{
  sample_msgs__msg__Unfiltered__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember sample_msgs__msg__Unfiltered__rosidl_typesupport_introspection_c__Unfiltered_message_member_array[3] = {
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sample_msgs__msg__Unfiltered, data),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "timestamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sample_msgs__msg__Unfiltered, timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "valid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sample_msgs__msg__Unfiltered, valid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers sample_msgs__msg__Unfiltered__rosidl_typesupport_introspection_c__Unfiltered_message_members = {
  "sample_msgs__msg",  // message namespace
  "Unfiltered",  // message name
  3,  // number of fields
  sizeof(sample_msgs__msg__Unfiltered),
  sample_msgs__msg__Unfiltered__rosidl_typesupport_introspection_c__Unfiltered_message_member_array,  // message members
  sample_msgs__msg__Unfiltered__rosidl_typesupport_introspection_c__Unfiltered_init_function,  // function to initialize message memory (memory has to be allocated)
  sample_msgs__msg__Unfiltered__rosidl_typesupport_introspection_c__Unfiltered_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t sample_msgs__msg__Unfiltered__rosidl_typesupport_introspection_c__Unfiltered_message_type_support_handle = {
  0,
  &sample_msgs__msg__Unfiltered__rosidl_typesupport_introspection_c__Unfiltered_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_sample_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sample_msgs, msg, Unfiltered)() {
  if (!sample_msgs__msg__Unfiltered__rosidl_typesupport_introspection_c__Unfiltered_message_type_support_handle.typesupport_identifier) {
    sample_msgs__msg__Unfiltered__rosidl_typesupport_introspection_c__Unfiltered_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &sample_msgs__msg__Unfiltered__rosidl_typesupport_introspection_c__Unfiltered_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from sample_msgs:msg/Filtered.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "sample_msgs/msg/detail/filtered__rosidl_typesupport_introspection_c.h"
#include "sample_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "sample_msgs/msg/detail/filtered__functions.h"
#include "sample_msgs/msg/detail/filtered__struct.h"


// Include directives for member types
// Member `metadata`
#include "sample_msgs/msg/metadata.h"
// Member `metadata`
#include "sample_msgs/msg/detail/metadata__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void sample_msgs__msg__Filtered__rosidl_typesupport_introspection_c__Filtered_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  sample_msgs__msg__Filtered__init(message_memory);
}

void sample_msgs__msg__Filtered__rosidl_typesupport_introspection_c__Filtered_fini_function(void * message_memory)
{
  sample_msgs__msg__Filtered__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember sample_msgs__msg__Filtered__rosidl_typesupport_introspection_c__Filtered_message_member_array[5] = {
  {
    "pos_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sample_msgs__msg__Filtered, pos_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pos_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sample_msgs__msg__Filtered, pos_y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pos_z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sample_msgs__msg__Filtered, pos_z),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "metadata",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sample_msgs__msg__Filtered, metadata),  // bytes offset in struct
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
    offsetof(sample_msgs__msg__Filtered, timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers sample_msgs__msg__Filtered__rosidl_typesupport_introspection_c__Filtered_message_members = {
  "sample_msgs__msg",  // message namespace
  "Filtered",  // message name
  5,  // number of fields
  sizeof(sample_msgs__msg__Filtered),
  sample_msgs__msg__Filtered__rosidl_typesupport_introspection_c__Filtered_message_member_array,  // message members
  sample_msgs__msg__Filtered__rosidl_typesupport_introspection_c__Filtered_init_function,  // function to initialize message memory (memory has to be allocated)
  sample_msgs__msg__Filtered__rosidl_typesupport_introspection_c__Filtered_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t sample_msgs__msg__Filtered__rosidl_typesupport_introspection_c__Filtered_message_type_support_handle = {
  0,
  &sample_msgs__msg__Filtered__rosidl_typesupport_introspection_c__Filtered_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_sample_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sample_msgs, msg, Filtered)() {
  sample_msgs__msg__Filtered__rosidl_typesupport_introspection_c__Filtered_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sample_msgs, msg, Metadata)();
  if (!sample_msgs__msg__Filtered__rosidl_typesupport_introspection_c__Filtered_message_type_support_handle.typesupport_identifier) {
    sample_msgs__msg__Filtered__rosidl_typesupport_introspection_c__Filtered_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &sample_msgs__msg__Filtered__rosidl_typesupport_introspection_c__Filtered_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

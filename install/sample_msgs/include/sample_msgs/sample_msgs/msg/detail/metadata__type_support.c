// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from sample_msgs:msg/Metadata.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "sample_msgs/msg/detail/metadata__rosidl_typesupport_introspection_c.h"
#include "sample_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "sample_msgs/msg/detail/metadata__functions.h"
#include "sample_msgs/msg/detail/metadata__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void sample_msgs__msg__Metadata__rosidl_typesupport_introspection_c__Metadata_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  sample_msgs__msg__Metadata__init(message_memory);
}

void sample_msgs__msg__Metadata__rosidl_typesupport_introspection_c__Metadata_fini_function(void * message_memory)
{
  sample_msgs__msg__Metadata__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember sample_msgs__msg__Metadata__rosidl_typesupport_introspection_c__Metadata_message_member_array[3] = {
  {
    "version",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sample_msgs__msg__Metadata, version),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "compression_method",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sample_msgs__msg__Metadata, compression_method),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "creation_date",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sample_msgs__msg__Metadata, creation_date),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers sample_msgs__msg__Metadata__rosidl_typesupport_introspection_c__Metadata_message_members = {
  "sample_msgs__msg",  // message namespace
  "Metadata",  // message name
  3,  // number of fields
  sizeof(sample_msgs__msg__Metadata),
  sample_msgs__msg__Metadata__rosidl_typesupport_introspection_c__Metadata_message_member_array,  // message members
  sample_msgs__msg__Metadata__rosidl_typesupport_introspection_c__Metadata_init_function,  // function to initialize message memory (memory has to be allocated)
  sample_msgs__msg__Metadata__rosidl_typesupport_introspection_c__Metadata_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t sample_msgs__msg__Metadata__rosidl_typesupport_introspection_c__Metadata_message_type_support_handle = {
  0,
  &sample_msgs__msg__Metadata__rosidl_typesupport_introspection_c__Metadata_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_sample_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sample_msgs, msg, Metadata)() {
  if (!sample_msgs__msg__Metadata__rosidl_typesupport_introspection_c__Metadata_message_type_support_handle.typesupport_identifier) {
    sample_msgs__msg__Metadata__rosidl_typesupport_introspection_c__Metadata_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &sample_msgs__msg__Metadata__rosidl_typesupport_introspection_c__Metadata_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

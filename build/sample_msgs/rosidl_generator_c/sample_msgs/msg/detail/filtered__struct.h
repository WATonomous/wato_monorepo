// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sample_msgs:msg/Filtered.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_MSGS__MSG__DETAIL__FILTERED__STRUCT_H_
#define SAMPLE_MSGS__MSG__DETAIL__FILTERED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'metadata'
#include "sample_msgs/msg/detail/metadata__struct.h"

/// Struct defined in msg/Filtered in the package sample_msgs.
typedef struct sample_msgs__msg__Filtered
{
  float pos_x;
  float pos_y;
  float pos_z;
  sample_msgs__msg__Metadata metadata;
  int64_t timestamp;
} sample_msgs__msg__Filtered;

// Struct for a sequence of sample_msgs__msg__Filtered.
typedef struct sample_msgs__msg__Filtered__Sequence
{
  sample_msgs__msg__Filtered * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sample_msgs__msg__Filtered__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SAMPLE_MSGS__MSG__DETAIL__FILTERED__STRUCT_H_

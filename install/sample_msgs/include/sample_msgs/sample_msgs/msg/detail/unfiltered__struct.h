// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sample_msgs:msg/Unfiltered.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_MSGS__MSG__DETAIL__UNFILTERED__STRUCT_H_
#define SAMPLE_MSGS__MSG__DETAIL__UNFILTERED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'data'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Unfiltered in the package sample_msgs.
typedef struct sample_msgs__msg__Unfiltered
{
  rosidl_runtime_c__String data;
  int64_t timestamp;
  bool valid;
} sample_msgs__msg__Unfiltered;

// Struct for a sequence of sample_msgs__msg__Unfiltered.
typedef struct sample_msgs__msg__Unfiltered__Sequence
{
  sample_msgs__msg__Unfiltered * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sample_msgs__msg__Unfiltered__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SAMPLE_MSGS__MSG__DETAIL__UNFILTERED__STRUCT_H_

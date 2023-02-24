// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sample_msgs:msg/FilteredArray.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_MSGS__MSG__DETAIL__FILTERED_ARRAY__STRUCT_H_
#define SAMPLE_MSGS__MSG__DETAIL__FILTERED_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'packets'
#include "sample_msgs/msg/detail/filtered__struct.h"

/// Struct defined in msg/FilteredArray in the package sample_msgs.
typedef struct sample_msgs__msg__FilteredArray
{
  sample_msgs__msg__Filtered__Sequence packets;
} sample_msgs__msg__FilteredArray;

// Struct for a sequence of sample_msgs__msg__FilteredArray.
typedef struct sample_msgs__msg__FilteredArray__Sequence
{
  sample_msgs__msg__FilteredArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sample_msgs__msg__FilteredArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SAMPLE_MSGS__MSG__DETAIL__FILTERED_ARRAY__STRUCT_H_

// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sample_msgs:msg/Metadata.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_MSGS__MSG__DETAIL__METADATA__STRUCT_H_
#define SAMPLE_MSGS__MSG__DETAIL__METADATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'DEFAULT'.
enum
{
  sample_msgs__msg__Metadata__DEFAULT = 0
};

/// Constant 'DICTIONARY'.
enum
{
  sample_msgs__msg__Metadata__DICTIONARY = 1
};

/// Constant 'RUN_LENGTH'.
enum
{
  sample_msgs__msg__Metadata__RUN_LENGTH = 2
};

/// Struct defined in msg/Metadata in the package sample_msgs.
typedef struct sample_msgs__msg__Metadata
{
  int8_t version;
  int8_t compression_method;
  int16_t creation_date;
} sample_msgs__msg__Metadata;

// Struct for a sequence of sample_msgs__msg__Metadata.
typedef struct sample_msgs__msg__Metadata__Sequence
{
  sample_msgs__msg__Metadata * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sample_msgs__msg__Metadata__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SAMPLE_MSGS__MSG__DETAIL__METADATA__STRUCT_H_

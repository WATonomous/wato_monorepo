// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from sample_msgs:msg/Metadata.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_MSGS__MSG__DETAIL__METADATA__FUNCTIONS_H_
#define SAMPLE_MSGS__MSG__DETAIL__METADATA__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "sample_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "sample_msgs/msg/detail/metadata__struct.h"

/// Initialize msg/Metadata message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * sample_msgs__msg__Metadata
 * )) before or use
 * sample_msgs__msg__Metadata__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
bool
sample_msgs__msg__Metadata__init(sample_msgs__msg__Metadata * msg);

/// Finalize msg/Metadata message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
void
sample_msgs__msg__Metadata__fini(sample_msgs__msg__Metadata * msg);

/// Create msg/Metadata message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * sample_msgs__msg__Metadata__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
sample_msgs__msg__Metadata *
sample_msgs__msg__Metadata__create();

/// Destroy msg/Metadata message.
/**
 * It calls
 * sample_msgs__msg__Metadata__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
void
sample_msgs__msg__Metadata__destroy(sample_msgs__msg__Metadata * msg);

/// Check for msg/Metadata message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
bool
sample_msgs__msg__Metadata__are_equal(const sample_msgs__msg__Metadata * lhs, const sample_msgs__msg__Metadata * rhs);

/// Copy a msg/Metadata message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
bool
sample_msgs__msg__Metadata__copy(
  const sample_msgs__msg__Metadata * input,
  sample_msgs__msg__Metadata * output);

/// Initialize array of msg/Metadata messages.
/**
 * It allocates the memory for the number of elements and calls
 * sample_msgs__msg__Metadata__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
bool
sample_msgs__msg__Metadata__Sequence__init(sample_msgs__msg__Metadata__Sequence * array, size_t size);

/// Finalize array of msg/Metadata messages.
/**
 * It calls
 * sample_msgs__msg__Metadata__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
void
sample_msgs__msg__Metadata__Sequence__fini(sample_msgs__msg__Metadata__Sequence * array);

/// Create array of msg/Metadata messages.
/**
 * It allocates the memory for the array and calls
 * sample_msgs__msg__Metadata__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
sample_msgs__msg__Metadata__Sequence *
sample_msgs__msg__Metadata__Sequence__create(size_t size);

/// Destroy array of msg/Metadata messages.
/**
 * It calls
 * sample_msgs__msg__Metadata__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
void
sample_msgs__msg__Metadata__Sequence__destroy(sample_msgs__msg__Metadata__Sequence * array);

/// Check for msg/Metadata message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
bool
sample_msgs__msg__Metadata__Sequence__are_equal(const sample_msgs__msg__Metadata__Sequence * lhs, const sample_msgs__msg__Metadata__Sequence * rhs);

/// Copy an array of msg/Metadata messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
bool
sample_msgs__msg__Metadata__Sequence__copy(
  const sample_msgs__msg__Metadata__Sequence * input,
  sample_msgs__msg__Metadata__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SAMPLE_MSGS__MSG__DETAIL__METADATA__FUNCTIONS_H_

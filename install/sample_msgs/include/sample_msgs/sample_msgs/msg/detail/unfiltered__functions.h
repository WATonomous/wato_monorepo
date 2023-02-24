// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from sample_msgs:msg/Unfiltered.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_MSGS__MSG__DETAIL__UNFILTERED__FUNCTIONS_H_
#define SAMPLE_MSGS__MSG__DETAIL__UNFILTERED__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "sample_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "sample_msgs/msg/detail/unfiltered__struct.h"

/// Initialize msg/Unfiltered message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * sample_msgs__msg__Unfiltered
 * )) before or use
 * sample_msgs__msg__Unfiltered__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
bool
sample_msgs__msg__Unfiltered__init(sample_msgs__msg__Unfiltered * msg);

/// Finalize msg/Unfiltered message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
void
sample_msgs__msg__Unfiltered__fini(sample_msgs__msg__Unfiltered * msg);

/// Create msg/Unfiltered message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * sample_msgs__msg__Unfiltered__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
sample_msgs__msg__Unfiltered *
sample_msgs__msg__Unfiltered__create();

/// Destroy msg/Unfiltered message.
/**
 * It calls
 * sample_msgs__msg__Unfiltered__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
void
sample_msgs__msg__Unfiltered__destroy(sample_msgs__msg__Unfiltered * msg);

/// Check for msg/Unfiltered message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
bool
sample_msgs__msg__Unfiltered__are_equal(const sample_msgs__msg__Unfiltered * lhs, const sample_msgs__msg__Unfiltered * rhs);

/// Copy a msg/Unfiltered message.
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
sample_msgs__msg__Unfiltered__copy(
  const sample_msgs__msg__Unfiltered * input,
  sample_msgs__msg__Unfiltered * output);

/// Initialize array of msg/Unfiltered messages.
/**
 * It allocates the memory for the number of elements and calls
 * sample_msgs__msg__Unfiltered__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
bool
sample_msgs__msg__Unfiltered__Sequence__init(sample_msgs__msg__Unfiltered__Sequence * array, size_t size);

/// Finalize array of msg/Unfiltered messages.
/**
 * It calls
 * sample_msgs__msg__Unfiltered__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
void
sample_msgs__msg__Unfiltered__Sequence__fini(sample_msgs__msg__Unfiltered__Sequence * array);

/// Create array of msg/Unfiltered messages.
/**
 * It allocates the memory for the array and calls
 * sample_msgs__msg__Unfiltered__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
sample_msgs__msg__Unfiltered__Sequence *
sample_msgs__msg__Unfiltered__Sequence__create(size_t size);

/// Destroy array of msg/Unfiltered messages.
/**
 * It calls
 * sample_msgs__msg__Unfiltered__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
void
sample_msgs__msg__Unfiltered__Sequence__destroy(sample_msgs__msg__Unfiltered__Sequence * array);

/// Check for msg/Unfiltered message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
bool
sample_msgs__msg__Unfiltered__Sequence__are_equal(const sample_msgs__msg__Unfiltered__Sequence * lhs, const sample_msgs__msg__Unfiltered__Sequence * rhs);

/// Copy an array of msg/Unfiltered messages.
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
sample_msgs__msg__Unfiltered__Sequence__copy(
  const sample_msgs__msg__Unfiltered__Sequence * input,
  sample_msgs__msg__Unfiltered__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SAMPLE_MSGS__MSG__DETAIL__UNFILTERED__FUNCTIONS_H_

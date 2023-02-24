// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from sample_msgs:msg/Filtered.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_MSGS__MSG__DETAIL__FILTERED__FUNCTIONS_H_
#define SAMPLE_MSGS__MSG__DETAIL__FILTERED__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "sample_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "sample_msgs/msg/detail/filtered__struct.h"

/// Initialize msg/Filtered message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * sample_msgs__msg__Filtered
 * )) before or use
 * sample_msgs__msg__Filtered__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
bool
sample_msgs__msg__Filtered__init(sample_msgs__msg__Filtered * msg);

/// Finalize msg/Filtered message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
void
sample_msgs__msg__Filtered__fini(sample_msgs__msg__Filtered * msg);

/// Create msg/Filtered message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * sample_msgs__msg__Filtered__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
sample_msgs__msg__Filtered *
sample_msgs__msg__Filtered__create();

/// Destroy msg/Filtered message.
/**
 * It calls
 * sample_msgs__msg__Filtered__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
void
sample_msgs__msg__Filtered__destroy(sample_msgs__msg__Filtered * msg);

/// Check for msg/Filtered message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
bool
sample_msgs__msg__Filtered__are_equal(const sample_msgs__msg__Filtered * lhs, const sample_msgs__msg__Filtered * rhs);

/// Copy a msg/Filtered message.
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
sample_msgs__msg__Filtered__copy(
  const sample_msgs__msg__Filtered * input,
  sample_msgs__msg__Filtered * output);

/// Initialize array of msg/Filtered messages.
/**
 * It allocates the memory for the number of elements and calls
 * sample_msgs__msg__Filtered__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
bool
sample_msgs__msg__Filtered__Sequence__init(sample_msgs__msg__Filtered__Sequence * array, size_t size);

/// Finalize array of msg/Filtered messages.
/**
 * It calls
 * sample_msgs__msg__Filtered__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
void
sample_msgs__msg__Filtered__Sequence__fini(sample_msgs__msg__Filtered__Sequence * array);

/// Create array of msg/Filtered messages.
/**
 * It allocates the memory for the array and calls
 * sample_msgs__msg__Filtered__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
sample_msgs__msg__Filtered__Sequence *
sample_msgs__msg__Filtered__Sequence__create(size_t size);

/// Destroy array of msg/Filtered messages.
/**
 * It calls
 * sample_msgs__msg__Filtered__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
void
sample_msgs__msg__Filtered__Sequence__destroy(sample_msgs__msg__Filtered__Sequence * array);

/// Check for msg/Filtered message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
bool
sample_msgs__msg__Filtered__Sequence__are_equal(const sample_msgs__msg__Filtered__Sequence * lhs, const sample_msgs__msg__Filtered__Sequence * rhs);

/// Copy an array of msg/Filtered messages.
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
sample_msgs__msg__Filtered__Sequence__copy(
  const sample_msgs__msg__Filtered__Sequence * input,
  sample_msgs__msg__Filtered__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SAMPLE_MSGS__MSG__DETAIL__FILTERED__FUNCTIONS_H_

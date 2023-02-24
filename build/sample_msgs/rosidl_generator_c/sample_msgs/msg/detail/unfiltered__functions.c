// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from sample_msgs:msg/Unfiltered.idl
// generated code does not contain a copyright notice
#include "sample_msgs/msg/detail/unfiltered__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `data`
#include "rosidl_runtime_c/string_functions.h"

bool
sample_msgs__msg__Unfiltered__init(sample_msgs__msg__Unfiltered * msg)
{
  if (!msg) {
    return false;
  }
  // data
  if (!rosidl_runtime_c__String__init(&msg->data)) {
    sample_msgs__msg__Unfiltered__fini(msg);
    return false;
  }
  // timestamp
  // valid
  return true;
}

void
sample_msgs__msg__Unfiltered__fini(sample_msgs__msg__Unfiltered * msg)
{
  if (!msg) {
    return;
  }
  // data
  rosidl_runtime_c__String__fini(&msg->data);
  // timestamp
  // valid
}

bool
sample_msgs__msg__Unfiltered__are_equal(const sample_msgs__msg__Unfiltered * lhs, const sample_msgs__msg__Unfiltered * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // data
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->data), &(rhs->data)))
  {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // valid
  if (lhs->valid != rhs->valid) {
    return false;
  }
  return true;
}

bool
sample_msgs__msg__Unfiltered__copy(
  const sample_msgs__msg__Unfiltered * input,
  sample_msgs__msg__Unfiltered * output)
{
  if (!input || !output) {
    return false;
  }
  // data
  if (!rosidl_runtime_c__String__copy(
      &(input->data), &(output->data)))
  {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // valid
  output->valid = input->valid;
  return true;
}

sample_msgs__msg__Unfiltered *
sample_msgs__msg__Unfiltered__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sample_msgs__msg__Unfiltered * msg = (sample_msgs__msg__Unfiltered *)allocator.allocate(sizeof(sample_msgs__msg__Unfiltered), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(sample_msgs__msg__Unfiltered));
  bool success = sample_msgs__msg__Unfiltered__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
sample_msgs__msg__Unfiltered__destroy(sample_msgs__msg__Unfiltered * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    sample_msgs__msg__Unfiltered__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
sample_msgs__msg__Unfiltered__Sequence__init(sample_msgs__msg__Unfiltered__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sample_msgs__msg__Unfiltered * data = NULL;

  if (size) {
    data = (sample_msgs__msg__Unfiltered *)allocator.zero_allocate(size, sizeof(sample_msgs__msg__Unfiltered), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = sample_msgs__msg__Unfiltered__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        sample_msgs__msg__Unfiltered__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
sample_msgs__msg__Unfiltered__Sequence__fini(sample_msgs__msg__Unfiltered__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      sample_msgs__msg__Unfiltered__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

sample_msgs__msg__Unfiltered__Sequence *
sample_msgs__msg__Unfiltered__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sample_msgs__msg__Unfiltered__Sequence * array = (sample_msgs__msg__Unfiltered__Sequence *)allocator.allocate(sizeof(sample_msgs__msg__Unfiltered__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = sample_msgs__msg__Unfiltered__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
sample_msgs__msg__Unfiltered__Sequence__destroy(sample_msgs__msg__Unfiltered__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    sample_msgs__msg__Unfiltered__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
sample_msgs__msg__Unfiltered__Sequence__are_equal(const sample_msgs__msg__Unfiltered__Sequence * lhs, const sample_msgs__msg__Unfiltered__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!sample_msgs__msg__Unfiltered__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
sample_msgs__msg__Unfiltered__Sequence__copy(
  const sample_msgs__msg__Unfiltered__Sequence * input,
  sample_msgs__msg__Unfiltered__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(sample_msgs__msg__Unfiltered);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    sample_msgs__msg__Unfiltered * data =
      (sample_msgs__msg__Unfiltered *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!sample_msgs__msg__Unfiltered__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          sample_msgs__msg__Unfiltered__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!sample_msgs__msg__Unfiltered__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from sample_msgs:msg/Filtered.idl
// generated code does not contain a copyright notice
#include "sample_msgs/msg/detail/filtered__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `metadata`
#include "sample_msgs/msg/detail/metadata__functions.h"

bool
sample_msgs__msg__Filtered__init(sample_msgs__msg__Filtered * msg)
{
  if (!msg) {
    return false;
  }
  // pos_x
  // pos_y
  // pos_z
  // metadata
  if (!sample_msgs__msg__Metadata__init(&msg->metadata)) {
    sample_msgs__msg__Filtered__fini(msg);
    return false;
  }
  // timestamp
  return true;
}

void
sample_msgs__msg__Filtered__fini(sample_msgs__msg__Filtered * msg)
{
  if (!msg) {
    return;
  }
  // pos_x
  // pos_y
  // pos_z
  // metadata
  sample_msgs__msg__Metadata__fini(&msg->metadata);
  // timestamp
}

bool
sample_msgs__msg__Filtered__are_equal(const sample_msgs__msg__Filtered * lhs, const sample_msgs__msg__Filtered * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pos_x
  if (lhs->pos_x != rhs->pos_x) {
    return false;
  }
  // pos_y
  if (lhs->pos_y != rhs->pos_y) {
    return false;
  }
  // pos_z
  if (lhs->pos_z != rhs->pos_z) {
    return false;
  }
  // metadata
  if (!sample_msgs__msg__Metadata__are_equal(
      &(lhs->metadata), &(rhs->metadata)))
  {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  return true;
}

bool
sample_msgs__msg__Filtered__copy(
  const sample_msgs__msg__Filtered * input,
  sample_msgs__msg__Filtered * output)
{
  if (!input || !output) {
    return false;
  }
  // pos_x
  output->pos_x = input->pos_x;
  // pos_y
  output->pos_y = input->pos_y;
  // pos_z
  output->pos_z = input->pos_z;
  // metadata
  if (!sample_msgs__msg__Metadata__copy(
      &(input->metadata), &(output->metadata)))
  {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  return true;
}

sample_msgs__msg__Filtered *
sample_msgs__msg__Filtered__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sample_msgs__msg__Filtered * msg = (sample_msgs__msg__Filtered *)allocator.allocate(sizeof(sample_msgs__msg__Filtered), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(sample_msgs__msg__Filtered));
  bool success = sample_msgs__msg__Filtered__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
sample_msgs__msg__Filtered__destroy(sample_msgs__msg__Filtered * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    sample_msgs__msg__Filtered__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
sample_msgs__msg__Filtered__Sequence__init(sample_msgs__msg__Filtered__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sample_msgs__msg__Filtered * data = NULL;

  if (size) {
    data = (sample_msgs__msg__Filtered *)allocator.zero_allocate(size, sizeof(sample_msgs__msg__Filtered), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = sample_msgs__msg__Filtered__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        sample_msgs__msg__Filtered__fini(&data[i - 1]);
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
sample_msgs__msg__Filtered__Sequence__fini(sample_msgs__msg__Filtered__Sequence * array)
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
      sample_msgs__msg__Filtered__fini(&array->data[i]);
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

sample_msgs__msg__Filtered__Sequence *
sample_msgs__msg__Filtered__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sample_msgs__msg__Filtered__Sequence * array = (sample_msgs__msg__Filtered__Sequence *)allocator.allocate(sizeof(sample_msgs__msg__Filtered__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = sample_msgs__msg__Filtered__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
sample_msgs__msg__Filtered__Sequence__destroy(sample_msgs__msg__Filtered__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    sample_msgs__msg__Filtered__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
sample_msgs__msg__Filtered__Sequence__are_equal(const sample_msgs__msg__Filtered__Sequence * lhs, const sample_msgs__msg__Filtered__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!sample_msgs__msg__Filtered__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
sample_msgs__msg__Filtered__Sequence__copy(
  const sample_msgs__msg__Filtered__Sequence * input,
  sample_msgs__msg__Filtered__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(sample_msgs__msg__Filtered);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    sample_msgs__msg__Filtered * data =
      (sample_msgs__msg__Filtered *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!sample_msgs__msg__Filtered__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          sample_msgs__msg__Filtered__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!sample_msgs__msg__Filtered__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

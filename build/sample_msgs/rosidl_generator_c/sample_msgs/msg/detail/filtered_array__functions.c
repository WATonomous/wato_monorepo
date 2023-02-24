// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from sample_msgs:msg/FilteredArray.idl
// generated code does not contain a copyright notice
#include "sample_msgs/msg/detail/filtered_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `packets`
#include "sample_msgs/msg/detail/filtered__functions.h"

bool
sample_msgs__msg__FilteredArray__init(sample_msgs__msg__FilteredArray * msg)
{
  if (!msg) {
    return false;
  }
  // packets
  if (!sample_msgs__msg__Filtered__Sequence__init(&msg->packets, 0)) {
    sample_msgs__msg__FilteredArray__fini(msg);
    return false;
  }
  return true;
}

void
sample_msgs__msg__FilteredArray__fini(sample_msgs__msg__FilteredArray * msg)
{
  if (!msg) {
    return;
  }
  // packets
  sample_msgs__msg__Filtered__Sequence__fini(&msg->packets);
}

bool
sample_msgs__msg__FilteredArray__are_equal(const sample_msgs__msg__FilteredArray * lhs, const sample_msgs__msg__FilteredArray * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // packets
  if (!sample_msgs__msg__Filtered__Sequence__are_equal(
      &(lhs->packets), &(rhs->packets)))
  {
    return false;
  }
  return true;
}

bool
sample_msgs__msg__FilteredArray__copy(
  const sample_msgs__msg__FilteredArray * input,
  sample_msgs__msg__FilteredArray * output)
{
  if (!input || !output) {
    return false;
  }
  // packets
  if (!sample_msgs__msg__Filtered__Sequence__copy(
      &(input->packets), &(output->packets)))
  {
    return false;
  }
  return true;
}

sample_msgs__msg__FilteredArray *
sample_msgs__msg__FilteredArray__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sample_msgs__msg__FilteredArray * msg = (sample_msgs__msg__FilteredArray *)allocator.allocate(sizeof(sample_msgs__msg__FilteredArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(sample_msgs__msg__FilteredArray));
  bool success = sample_msgs__msg__FilteredArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
sample_msgs__msg__FilteredArray__destroy(sample_msgs__msg__FilteredArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    sample_msgs__msg__FilteredArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
sample_msgs__msg__FilteredArray__Sequence__init(sample_msgs__msg__FilteredArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sample_msgs__msg__FilteredArray * data = NULL;

  if (size) {
    data = (sample_msgs__msg__FilteredArray *)allocator.zero_allocate(size, sizeof(sample_msgs__msg__FilteredArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = sample_msgs__msg__FilteredArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        sample_msgs__msg__FilteredArray__fini(&data[i - 1]);
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
sample_msgs__msg__FilteredArray__Sequence__fini(sample_msgs__msg__FilteredArray__Sequence * array)
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
      sample_msgs__msg__FilteredArray__fini(&array->data[i]);
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

sample_msgs__msg__FilteredArray__Sequence *
sample_msgs__msg__FilteredArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sample_msgs__msg__FilteredArray__Sequence * array = (sample_msgs__msg__FilteredArray__Sequence *)allocator.allocate(sizeof(sample_msgs__msg__FilteredArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = sample_msgs__msg__FilteredArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
sample_msgs__msg__FilteredArray__Sequence__destroy(sample_msgs__msg__FilteredArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    sample_msgs__msg__FilteredArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
sample_msgs__msg__FilteredArray__Sequence__are_equal(const sample_msgs__msg__FilteredArray__Sequence * lhs, const sample_msgs__msg__FilteredArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!sample_msgs__msg__FilteredArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
sample_msgs__msg__FilteredArray__Sequence__copy(
  const sample_msgs__msg__FilteredArray__Sequence * input,
  sample_msgs__msg__FilteredArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(sample_msgs__msg__FilteredArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    sample_msgs__msg__FilteredArray * data =
      (sample_msgs__msg__FilteredArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!sample_msgs__msg__FilteredArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          sample_msgs__msg__FilteredArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!sample_msgs__msg__FilteredArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

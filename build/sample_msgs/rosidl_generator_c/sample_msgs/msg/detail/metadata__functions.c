// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from sample_msgs:msg/Metadata.idl
// generated code does not contain a copyright notice
#include "sample_msgs/msg/detail/metadata__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
sample_msgs__msg__Metadata__init(sample_msgs__msg__Metadata * msg)
{
  if (!msg) {
    return false;
  }
  // version
  // compression_method
  // creation_date
  return true;
}

void
sample_msgs__msg__Metadata__fini(sample_msgs__msg__Metadata * msg)
{
  if (!msg) {
    return;
  }
  // version
  // compression_method
  // creation_date
}

bool
sample_msgs__msg__Metadata__are_equal(const sample_msgs__msg__Metadata * lhs, const sample_msgs__msg__Metadata * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // version
  if (lhs->version != rhs->version) {
    return false;
  }
  // compression_method
  if (lhs->compression_method != rhs->compression_method) {
    return false;
  }
  // creation_date
  if (lhs->creation_date != rhs->creation_date) {
    return false;
  }
  return true;
}

bool
sample_msgs__msg__Metadata__copy(
  const sample_msgs__msg__Metadata * input,
  sample_msgs__msg__Metadata * output)
{
  if (!input || !output) {
    return false;
  }
  // version
  output->version = input->version;
  // compression_method
  output->compression_method = input->compression_method;
  // creation_date
  output->creation_date = input->creation_date;
  return true;
}

sample_msgs__msg__Metadata *
sample_msgs__msg__Metadata__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sample_msgs__msg__Metadata * msg = (sample_msgs__msg__Metadata *)allocator.allocate(sizeof(sample_msgs__msg__Metadata), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(sample_msgs__msg__Metadata));
  bool success = sample_msgs__msg__Metadata__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
sample_msgs__msg__Metadata__destroy(sample_msgs__msg__Metadata * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    sample_msgs__msg__Metadata__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
sample_msgs__msg__Metadata__Sequence__init(sample_msgs__msg__Metadata__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sample_msgs__msg__Metadata * data = NULL;

  if (size) {
    data = (sample_msgs__msg__Metadata *)allocator.zero_allocate(size, sizeof(sample_msgs__msg__Metadata), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = sample_msgs__msg__Metadata__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        sample_msgs__msg__Metadata__fini(&data[i - 1]);
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
sample_msgs__msg__Metadata__Sequence__fini(sample_msgs__msg__Metadata__Sequence * array)
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
      sample_msgs__msg__Metadata__fini(&array->data[i]);
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

sample_msgs__msg__Metadata__Sequence *
sample_msgs__msg__Metadata__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sample_msgs__msg__Metadata__Sequence * array = (sample_msgs__msg__Metadata__Sequence *)allocator.allocate(sizeof(sample_msgs__msg__Metadata__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = sample_msgs__msg__Metadata__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
sample_msgs__msg__Metadata__Sequence__destroy(sample_msgs__msg__Metadata__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    sample_msgs__msg__Metadata__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
sample_msgs__msg__Metadata__Sequence__are_equal(const sample_msgs__msg__Metadata__Sequence * lhs, const sample_msgs__msg__Metadata__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!sample_msgs__msg__Metadata__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
sample_msgs__msg__Metadata__Sequence__copy(
  const sample_msgs__msg__Metadata__Sequence * input,
  sample_msgs__msg__Metadata__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(sample_msgs__msg__Metadata);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    sample_msgs__msg__Metadata * data =
      (sample_msgs__msg__Metadata *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!sample_msgs__msg__Metadata__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          sample_msgs__msg__Metadata__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!sample_msgs__msg__Metadata__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

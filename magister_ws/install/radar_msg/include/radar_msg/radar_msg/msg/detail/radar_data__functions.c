// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from radar_msg:msg/RadarData.idl
// generated code does not contain a copyright notice
#include "radar_msg/msg/detail/radar_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `dtype`
#include "rosidl_runtime_c/string_functions.h"
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
radar_msg__msg__RadarData__init(radar_msg__msg__RadarData * msg)
{
  if (!msg) {
    return false;
  }
  // rows
  // cols
  // dtype
  if (!rosidl_runtime_c__String__init(&msg->dtype)) {
    radar_msg__msg__RadarData__fini(msg);
    return false;
  }
  // data
  if (!rosidl_runtime_c__double__Sequence__init(&msg->data, 0)) {
    radar_msg__msg__RadarData__fini(msg);
    return false;
  }
  return true;
}

void
radar_msg__msg__RadarData__fini(radar_msg__msg__RadarData * msg)
{
  if (!msg) {
    return;
  }
  // rows
  // cols
  // dtype
  rosidl_runtime_c__String__fini(&msg->dtype);
  // data
  rosidl_runtime_c__double__Sequence__fini(&msg->data);
}

bool
radar_msg__msg__RadarData__are_equal(const radar_msg__msg__RadarData * lhs, const radar_msg__msg__RadarData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // rows
  if (lhs->rows != rhs->rows) {
    return false;
  }
  // cols
  if (lhs->cols != rhs->cols) {
    return false;
  }
  // dtype
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->dtype), &(rhs->dtype)))
  {
    return false;
  }
  // data
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->data), &(rhs->data)))
  {
    return false;
  }
  return true;
}

bool
radar_msg__msg__RadarData__copy(
  const radar_msg__msg__RadarData * input,
  radar_msg__msg__RadarData * output)
{
  if (!input || !output) {
    return false;
  }
  // rows
  output->rows = input->rows;
  // cols
  output->cols = input->cols;
  // dtype
  if (!rosidl_runtime_c__String__copy(
      &(input->dtype), &(output->dtype)))
  {
    return false;
  }
  // data
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->data), &(output->data)))
  {
    return false;
  }
  return true;
}

radar_msg__msg__RadarData *
radar_msg__msg__RadarData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_msg__msg__RadarData * msg = (radar_msg__msg__RadarData *)allocator.allocate(sizeof(radar_msg__msg__RadarData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(radar_msg__msg__RadarData));
  bool success = radar_msg__msg__RadarData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
radar_msg__msg__RadarData__destroy(radar_msg__msg__RadarData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    radar_msg__msg__RadarData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
radar_msg__msg__RadarData__Sequence__init(radar_msg__msg__RadarData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_msg__msg__RadarData * data = NULL;

  if (size) {
    data = (radar_msg__msg__RadarData *)allocator.zero_allocate(size, sizeof(radar_msg__msg__RadarData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = radar_msg__msg__RadarData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        radar_msg__msg__RadarData__fini(&data[i - 1]);
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
radar_msg__msg__RadarData__Sequence__fini(radar_msg__msg__RadarData__Sequence * array)
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
      radar_msg__msg__RadarData__fini(&array->data[i]);
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

radar_msg__msg__RadarData__Sequence *
radar_msg__msg__RadarData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_msg__msg__RadarData__Sequence * array = (radar_msg__msg__RadarData__Sequence *)allocator.allocate(sizeof(radar_msg__msg__RadarData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = radar_msg__msg__RadarData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
radar_msg__msg__RadarData__Sequence__destroy(radar_msg__msg__RadarData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    radar_msg__msg__RadarData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
radar_msg__msg__RadarData__Sequence__are_equal(const radar_msg__msg__RadarData__Sequence * lhs, const radar_msg__msg__RadarData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!radar_msg__msg__RadarData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
radar_msg__msg__RadarData__Sequence__copy(
  const radar_msg__msg__RadarData__Sequence * input,
  radar_msg__msg__RadarData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(radar_msg__msg__RadarData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    radar_msg__msg__RadarData * data =
      (radar_msg__msg__RadarData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!radar_msg__msg__RadarData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          radar_msg__msg__RadarData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!radar_msg__msg__RadarData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

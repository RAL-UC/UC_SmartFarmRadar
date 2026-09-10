// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from radar_msg:action/RadarBeamform.idl
// generated code does not contain a copyright notice
#include "radar_msg/action/detail/radar_beamform__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
radar_msg__action__RadarBeamform_Goal__init(radar_msg__action__RadarBeamform_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
radar_msg__action__RadarBeamform_Goal__fini(radar_msg__action__RadarBeamform_Goal * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
radar_msg__action__RadarBeamform_Goal__are_equal(const radar_msg__action__RadarBeamform_Goal * lhs, const radar_msg__action__RadarBeamform_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
radar_msg__action__RadarBeamform_Goal__copy(
  const radar_msg__action__RadarBeamform_Goal * input,
  radar_msg__action__RadarBeamform_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

radar_msg__action__RadarBeamform_Goal *
radar_msg__action__RadarBeamform_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_msg__action__RadarBeamform_Goal * msg = (radar_msg__action__RadarBeamform_Goal *)allocator.allocate(sizeof(radar_msg__action__RadarBeamform_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(radar_msg__action__RadarBeamform_Goal));
  bool success = radar_msg__action__RadarBeamform_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
radar_msg__action__RadarBeamform_Goal__destroy(radar_msg__action__RadarBeamform_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    radar_msg__action__RadarBeamform_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
radar_msg__action__RadarBeamform_Goal__Sequence__init(radar_msg__action__RadarBeamform_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_msg__action__RadarBeamform_Goal * data = NULL;

  if (size) {
    data = (radar_msg__action__RadarBeamform_Goal *)allocator.zero_allocate(size, sizeof(radar_msg__action__RadarBeamform_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = radar_msg__action__RadarBeamform_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        radar_msg__action__RadarBeamform_Goal__fini(&data[i - 1]);
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
radar_msg__action__RadarBeamform_Goal__Sequence__fini(radar_msg__action__RadarBeamform_Goal__Sequence * array)
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
      radar_msg__action__RadarBeamform_Goal__fini(&array->data[i]);
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

radar_msg__action__RadarBeamform_Goal__Sequence *
radar_msg__action__RadarBeamform_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_msg__action__RadarBeamform_Goal__Sequence * array = (radar_msg__action__RadarBeamform_Goal__Sequence *)allocator.allocate(sizeof(radar_msg__action__RadarBeamform_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = radar_msg__action__RadarBeamform_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
radar_msg__action__RadarBeamform_Goal__Sequence__destroy(radar_msg__action__RadarBeamform_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    radar_msg__action__RadarBeamform_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
radar_msg__action__RadarBeamform_Goal__Sequence__are_equal(const radar_msg__action__RadarBeamform_Goal__Sequence * lhs, const radar_msg__action__RadarBeamform_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!radar_msg__action__RadarBeamform_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
radar_msg__action__RadarBeamform_Goal__Sequence__copy(
  const radar_msg__action__RadarBeamform_Goal__Sequence * input,
  radar_msg__action__RadarBeamform_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(radar_msg__action__RadarBeamform_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    radar_msg__action__RadarBeamform_Goal * data =
      (radar_msg__action__RadarBeamform_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!radar_msg__action__RadarBeamform_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          radar_msg__action__RadarBeamform_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!radar_msg__action__RadarBeamform_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"
// Member `radar_data`
#include "radar_msg/msg/detail/radar_data__functions.h"

bool
radar_msg__action__RadarBeamform_Result__init(radar_msg__action__RadarBeamform_Result * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    radar_msg__action__RadarBeamform_Result__fini(msg);
    return false;
  }
  // radar_data
  if (!radar_msg__msg__RadarData__init(&msg->radar_data)) {
    radar_msg__action__RadarBeamform_Result__fini(msg);
    return false;
  }
  return true;
}

void
radar_msg__action__RadarBeamform_Result__fini(radar_msg__action__RadarBeamform_Result * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
  // radar_data
  radar_msg__msg__RadarData__fini(&msg->radar_data);
}

bool
radar_msg__action__RadarBeamform_Result__are_equal(const radar_msg__action__RadarBeamform_Result * lhs, const radar_msg__action__RadarBeamform_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  // radar_data
  if (!radar_msg__msg__RadarData__are_equal(
      &(lhs->radar_data), &(rhs->radar_data)))
  {
    return false;
  }
  return true;
}

bool
radar_msg__action__RadarBeamform_Result__copy(
  const radar_msg__action__RadarBeamform_Result * input,
  radar_msg__action__RadarBeamform_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  // radar_data
  if (!radar_msg__msg__RadarData__copy(
      &(input->radar_data), &(output->radar_data)))
  {
    return false;
  }
  return true;
}

radar_msg__action__RadarBeamform_Result *
radar_msg__action__RadarBeamform_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_msg__action__RadarBeamform_Result * msg = (radar_msg__action__RadarBeamform_Result *)allocator.allocate(sizeof(radar_msg__action__RadarBeamform_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(radar_msg__action__RadarBeamform_Result));
  bool success = radar_msg__action__RadarBeamform_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
radar_msg__action__RadarBeamform_Result__destroy(radar_msg__action__RadarBeamform_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    radar_msg__action__RadarBeamform_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
radar_msg__action__RadarBeamform_Result__Sequence__init(radar_msg__action__RadarBeamform_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_msg__action__RadarBeamform_Result * data = NULL;

  if (size) {
    data = (radar_msg__action__RadarBeamform_Result *)allocator.zero_allocate(size, sizeof(radar_msg__action__RadarBeamform_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = radar_msg__action__RadarBeamform_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        radar_msg__action__RadarBeamform_Result__fini(&data[i - 1]);
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
radar_msg__action__RadarBeamform_Result__Sequence__fini(radar_msg__action__RadarBeamform_Result__Sequence * array)
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
      radar_msg__action__RadarBeamform_Result__fini(&array->data[i]);
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

radar_msg__action__RadarBeamform_Result__Sequence *
radar_msg__action__RadarBeamform_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_msg__action__RadarBeamform_Result__Sequence * array = (radar_msg__action__RadarBeamform_Result__Sequence *)allocator.allocate(sizeof(radar_msg__action__RadarBeamform_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = radar_msg__action__RadarBeamform_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
radar_msg__action__RadarBeamform_Result__Sequence__destroy(radar_msg__action__RadarBeamform_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    radar_msg__action__RadarBeamform_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
radar_msg__action__RadarBeamform_Result__Sequence__are_equal(const radar_msg__action__RadarBeamform_Result__Sequence * lhs, const radar_msg__action__RadarBeamform_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!radar_msg__action__RadarBeamform_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
radar_msg__action__RadarBeamform_Result__Sequence__copy(
  const radar_msg__action__RadarBeamform_Result__Sequence * input,
  radar_msg__action__RadarBeamform_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(radar_msg__action__RadarBeamform_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    radar_msg__action__RadarBeamform_Result * data =
      (radar_msg__action__RadarBeamform_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!radar_msg__action__RadarBeamform_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          radar_msg__action__RadarBeamform_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!radar_msg__action__RadarBeamform_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `status`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
radar_msg__action__RadarBeamform_Feedback__init(radar_msg__action__RadarBeamform_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // status
  if (!rosidl_runtime_c__String__init(&msg->status)) {
    radar_msg__action__RadarBeamform_Feedback__fini(msg);
    return false;
  }
  return true;
}

void
radar_msg__action__RadarBeamform_Feedback__fini(radar_msg__action__RadarBeamform_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // status
  rosidl_runtime_c__String__fini(&msg->status);
}

bool
radar_msg__action__RadarBeamform_Feedback__are_equal(const radar_msg__action__RadarBeamform_Feedback * lhs, const radar_msg__action__RadarBeamform_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->status), &(rhs->status)))
  {
    return false;
  }
  return true;
}

bool
radar_msg__action__RadarBeamform_Feedback__copy(
  const radar_msg__action__RadarBeamform_Feedback * input,
  radar_msg__action__RadarBeamform_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  if (!rosidl_runtime_c__String__copy(
      &(input->status), &(output->status)))
  {
    return false;
  }
  return true;
}

radar_msg__action__RadarBeamform_Feedback *
radar_msg__action__RadarBeamform_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_msg__action__RadarBeamform_Feedback * msg = (radar_msg__action__RadarBeamform_Feedback *)allocator.allocate(sizeof(radar_msg__action__RadarBeamform_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(radar_msg__action__RadarBeamform_Feedback));
  bool success = radar_msg__action__RadarBeamform_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
radar_msg__action__RadarBeamform_Feedback__destroy(radar_msg__action__RadarBeamform_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    radar_msg__action__RadarBeamform_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
radar_msg__action__RadarBeamform_Feedback__Sequence__init(radar_msg__action__RadarBeamform_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_msg__action__RadarBeamform_Feedback * data = NULL;

  if (size) {
    data = (radar_msg__action__RadarBeamform_Feedback *)allocator.zero_allocate(size, sizeof(radar_msg__action__RadarBeamform_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = radar_msg__action__RadarBeamform_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        radar_msg__action__RadarBeamform_Feedback__fini(&data[i - 1]);
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
radar_msg__action__RadarBeamform_Feedback__Sequence__fini(radar_msg__action__RadarBeamform_Feedback__Sequence * array)
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
      radar_msg__action__RadarBeamform_Feedback__fini(&array->data[i]);
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

radar_msg__action__RadarBeamform_Feedback__Sequence *
radar_msg__action__RadarBeamform_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_msg__action__RadarBeamform_Feedback__Sequence * array = (radar_msg__action__RadarBeamform_Feedback__Sequence *)allocator.allocate(sizeof(radar_msg__action__RadarBeamform_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = radar_msg__action__RadarBeamform_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
radar_msg__action__RadarBeamform_Feedback__Sequence__destroy(radar_msg__action__RadarBeamform_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    radar_msg__action__RadarBeamform_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
radar_msg__action__RadarBeamform_Feedback__Sequence__are_equal(const radar_msg__action__RadarBeamform_Feedback__Sequence * lhs, const radar_msg__action__RadarBeamform_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!radar_msg__action__RadarBeamform_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
radar_msg__action__RadarBeamform_Feedback__Sequence__copy(
  const radar_msg__action__RadarBeamform_Feedback__Sequence * input,
  radar_msg__action__RadarBeamform_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(radar_msg__action__RadarBeamform_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    radar_msg__action__RadarBeamform_Feedback * data =
      (radar_msg__action__RadarBeamform_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!radar_msg__action__RadarBeamform_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          radar_msg__action__RadarBeamform_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!radar_msg__action__RadarBeamform_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "radar_msg/action/detail/radar_beamform__functions.h"

bool
radar_msg__action__RadarBeamform_SendGoal_Request__init(radar_msg__action__RadarBeamform_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    radar_msg__action__RadarBeamform_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!radar_msg__action__RadarBeamform_Goal__init(&msg->goal)) {
    radar_msg__action__RadarBeamform_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
radar_msg__action__RadarBeamform_SendGoal_Request__fini(radar_msg__action__RadarBeamform_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  radar_msg__action__RadarBeamform_Goal__fini(&msg->goal);
}

bool
radar_msg__action__RadarBeamform_SendGoal_Request__are_equal(const radar_msg__action__RadarBeamform_SendGoal_Request * lhs, const radar_msg__action__RadarBeamform_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!radar_msg__action__RadarBeamform_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
radar_msg__action__RadarBeamform_SendGoal_Request__copy(
  const radar_msg__action__RadarBeamform_SendGoal_Request * input,
  radar_msg__action__RadarBeamform_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!radar_msg__action__RadarBeamform_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

radar_msg__action__RadarBeamform_SendGoal_Request *
radar_msg__action__RadarBeamform_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_msg__action__RadarBeamform_SendGoal_Request * msg = (radar_msg__action__RadarBeamform_SendGoal_Request *)allocator.allocate(sizeof(radar_msg__action__RadarBeamform_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(radar_msg__action__RadarBeamform_SendGoal_Request));
  bool success = radar_msg__action__RadarBeamform_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
radar_msg__action__RadarBeamform_SendGoal_Request__destroy(radar_msg__action__RadarBeamform_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    radar_msg__action__RadarBeamform_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
radar_msg__action__RadarBeamform_SendGoal_Request__Sequence__init(radar_msg__action__RadarBeamform_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_msg__action__RadarBeamform_SendGoal_Request * data = NULL;

  if (size) {
    data = (radar_msg__action__RadarBeamform_SendGoal_Request *)allocator.zero_allocate(size, sizeof(radar_msg__action__RadarBeamform_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = radar_msg__action__RadarBeamform_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        radar_msg__action__RadarBeamform_SendGoal_Request__fini(&data[i - 1]);
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
radar_msg__action__RadarBeamform_SendGoal_Request__Sequence__fini(radar_msg__action__RadarBeamform_SendGoal_Request__Sequence * array)
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
      radar_msg__action__RadarBeamform_SendGoal_Request__fini(&array->data[i]);
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

radar_msg__action__RadarBeamform_SendGoal_Request__Sequence *
radar_msg__action__RadarBeamform_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_msg__action__RadarBeamform_SendGoal_Request__Sequence * array = (radar_msg__action__RadarBeamform_SendGoal_Request__Sequence *)allocator.allocate(sizeof(radar_msg__action__RadarBeamform_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = radar_msg__action__RadarBeamform_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
radar_msg__action__RadarBeamform_SendGoal_Request__Sequence__destroy(radar_msg__action__RadarBeamform_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    radar_msg__action__RadarBeamform_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
radar_msg__action__RadarBeamform_SendGoal_Request__Sequence__are_equal(const radar_msg__action__RadarBeamform_SendGoal_Request__Sequence * lhs, const radar_msg__action__RadarBeamform_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!radar_msg__action__RadarBeamform_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
radar_msg__action__RadarBeamform_SendGoal_Request__Sequence__copy(
  const radar_msg__action__RadarBeamform_SendGoal_Request__Sequence * input,
  radar_msg__action__RadarBeamform_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(radar_msg__action__RadarBeamform_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    radar_msg__action__RadarBeamform_SendGoal_Request * data =
      (radar_msg__action__RadarBeamform_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!radar_msg__action__RadarBeamform_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          radar_msg__action__RadarBeamform_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!radar_msg__action__RadarBeamform_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
radar_msg__action__RadarBeamform_SendGoal_Response__init(radar_msg__action__RadarBeamform_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    radar_msg__action__RadarBeamform_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
radar_msg__action__RadarBeamform_SendGoal_Response__fini(radar_msg__action__RadarBeamform_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
radar_msg__action__RadarBeamform_SendGoal_Response__are_equal(const radar_msg__action__RadarBeamform_SendGoal_Response * lhs, const radar_msg__action__RadarBeamform_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
radar_msg__action__RadarBeamform_SendGoal_Response__copy(
  const radar_msg__action__RadarBeamform_SendGoal_Response * input,
  radar_msg__action__RadarBeamform_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

radar_msg__action__RadarBeamform_SendGoal_Response *
radar_msg__action__RadarBeamform_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_msg__action__RadarBeamform_SendGoal_Response * msg = (radar_msg__action__RadarBeamform_SendGoal_Response *)allocator.allocate(sizeof(radar_msg__action__RadarBeamform_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(radar_msg__action__RadarBeamform_SendGoal_Response));
  bool success = radar_msg__action__RadarBeamform_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
radar_msg__action__RadarBeamform_SendGoal_Response__destroy(radar_msg__action__RadarBeamform_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    radar_msg__action__RadarBeamform_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
radar_msg__action__RadarBeamform_SendGoal_Response__Sequence__init(radar_msg__action__RadarBeamform_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_msg__action__RadarBeamform_SendGoal_Response * data = NULL;

  if (size) {
    data = (radar_msg__action__RadarBeamform_SendGoal_Response *)allocator.zero_allocate(size, sizeof(radar_msg__action__RadarBeamform_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = radar_msg__action__RadarBeamform_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        radar_msg__action__RadarBeamform_SendGoal_Response__fini(&data[i - 1]);
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
radar_msg__action__RadarBeamform_SendGoal_Response__Sequence__fini(radar_msg__action__RadarBeamform_SendGoal_Response__Sequence * array)
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
      radar_msg__action__RadarBeamform_SendGoal_Response__fini(&array->data[i]);
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

radar_msg__action__RadarBeamform_SendGoal_Response__Sequence *
radar_msg__action__RadarBeamform_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_msg__action__RadarBeamform_SendGoal_Response__Sequence * array = (radar_msg__action__RadarBeamform_SendGoal_Response__Sequence *)allocator.allocate(sizeof(radar_msg__action__RadarBeamform_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = radar_msg__action__RadarBeamform_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
radar_msg__action__RadarBeamform_SendGoal_Response__Sequence__destroy(radar_msg__action__RadarBeamform_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    radar_msg__action__RadarBeamform_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
radar_msg__action__RadarBeamform_SendGoal_Response__Sequence__are_equal(const radar_msg__action__RadarBeamform_SendGoal_Response__Sequence * lhs, const radar_msg__action__RadarBeamform_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!radar_msg__action__RadarBeamform_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
radar_msg__action__RadarBeamform_SendGoal_Response__Sequence__copy(
  const radar_msg__action__RadarBeamform_SendGoal_Response__Sequence * input,
  radar_msg__action__RadarBeamform_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(radar_msg__action__RadarBeamform_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    radar_msg__action__RadarBeamform_SendGoal_Response * data =
      (radar_msg__action__RadarBeamform_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!radar_msg__action__RadarBeamform_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          radar_msg__action__RadarBeamform_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!radar_msg__action__RadarBeamform_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
radar_msg__action__RadarBeamform_GetResult_Request__init(radar_msg__action__RadarBeamform_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    radar_msg__action__RadarBeamform_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
radar_msg__action__RadarBeamform_GetResult_Request__fini(radar_msg__action__RadarBeamform_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
radar_msg__action__RadarBeamform_GetResult_Request__are_equal(const radar_msg__action__RadarBeamform_GetResult_Request * lhs, const radar_msg__action__RadarBeamform_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
radar_msg__action__RadarBeamform_GetResult_Request__copy(
  const radar_msg__action__RadarBeamform_GetResult_Request * input,
  radar_msg__action__RadarBeamform_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

radar_msg__action__RadarBeamform_GetResult_Request *
radar_msg__action__RadarBeamform_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_msg__action__RadarBeamform_GetResult_Request * msg = (radar_msg__action__RadarBeamform_GetResult_Request *)allocator.allocate(sizeof(radar_msg__action__RadarBeamform_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(radar_msg__action__RadarBeamform_GetResult_Request));
  bool success = radar_msg__action__RadarBeamform_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
radar_msg__action__RadarBeamform_GetResult_Request__destroy(radar_msg__action__RadarBeamform_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    radar_msg__action__RadarBeamform_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
radar_msg__action__RadarBeamform_GetResult_Request__Sequence__init(radar_msg__action__RadarBeamform_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_msg__action__RadarBeamform_GetResult_Request * data = NULL;

  if (size) {
    data = (radar_msg__action__RadarBeamform_GetResult_Request *)allocator.zero_allocate(size, sizeof(radar_msg__action__RadarBeamform_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = radar_msg__action__RadarBeamform_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        radar_msg__action__RadarBeamform_GetResult_Request__fini(&data[i - 1]);
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
radar_msg__action__RadarBeamform_GetResult_Request__Sequence__fini(radar_msg__action__RadarBeamform_GetResult_Request__Sequence * array)
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
      radar_msg__action__RadarBeamform_GetResult_Request__fini(&array->data[i]);
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

radar_msg__action__RadarBeamform_GetResult_Request__Sequence *
radar_msg__action__RadarBeamform_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_msg__action__RadarBeamform_GetResult_Request__Sequence * array = (radar_msg__action__RadarBeamform_GetResult_Request__Sequence *)allocator.allocate(sizeof(radar_msg__action__RadarBeamform_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = radar_msg__action__RadarBeamform_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
radar_msg__action__RadarBeamform_GetResult_Request__Sequence__destroy(radar_msg__action__RadarBeamform_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    radar_msg__action__RadarBeamform_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
radar_msg__action__RadarBeamform_GetResult_Request__Sequence__are_equal(const radar_msg__action__RadarBeamform_GetResult_Request__Sequence * lhs, const radar_msg__action__RadarBeamform_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!radar_msg__action__RadarBeamform_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
radar_msg__action__RadarBeamform_GetResult_Request__Sequence__copy(
  const radar_msg__action__RadarBeamform_GetResult_Request__Sequence * input,
  radar_msg__action__RadarBeamform_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(radar_msg__action__RadarBeamform_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    radar_msg__action__RadarBeamform_GetResult_Request * data =
      (radar_msg__action__RadarBeamform_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!radar_msg__action__RadarBeamform_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          radar_msg__action__RadarBeamform_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!radar_msg__action__RadarBeamform_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "radar_msg/action/detail/radar_beamform__functions.h"

bool
radar_msg__action__RadarBeamform_GetResult_Response__init(radar_msg__action__RadarBeamform_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!radar_msg__action__RadarBeamform_Result__init(&msg->result)) {
    radar_msg__action__RadarBeamform_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
radar_msg__action__RadarBeamform_GetResult_Response__fini(radar_msg__action__RadarBeamform_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  radar_msg__action__RadarBeamform_Result__fini(&msg->result);
}

bool
radar_msg__action__RadarBeamform_GetResult_Response__are_equal(const radar_msg__action__RadarBeamform_GetResult_Response * lhs, const radar_msg__action__RadarBeamform_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!radar_msg__action__RadarBeamform_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
radar_msg__action__RadarBeamform_GetResult_Response__copy(
  const radar_msg__action__RadarBeamform_GetResult_Response * input,
  radar_msg__action__RadarBeamform_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!radar_msg__action__RadarBeamform_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

radar_msg__action__RadarBeamform_GetResult_Response *
radar_msg__action__RadarBeamform_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_msg__action__RadarBeamform_GetResult_Response * msg = (radar_msg__action__RadarBeamform_GetResult_Response *)allocator.allocate(sizeof(radar_msg__action__RadarBeamform_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(radar_msg__action__RadarBeamform_GetResult_Response));
  bool success = radar_msg__action__RadarBeamform_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
radar_msg__action__RadarBeamform_GetResult_Response__destroy(radar_msg__action__RadarBeamform_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    radar_msg__action__RadarBeamform_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
radar_msg__action__RadarBeamform_GetResult_Response__Sequence__init(radar_msg__action__RadarBeamform_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_msg__action__RadarBeamform_GetResult_Response * data = NULL;

  if (size) {
    data = (radar_msg__action__RadarBeamform_GetResult_Response *)allocator.zero_allocate(size, sizeof(radar_msg__action__RadarBeamform_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = radar_msg__action__RadarBeamform_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        radar_msg__action__RadarBeamform_GetResult_Response__fini(&data[i - 1]);
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
radar_msg__action__RadarBeamform_GetResult_Response__Sequence__fini(radar_msg__action__RadarBeamform_GetResult_Response__Sequence * array)
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
      radar_msg__action__RadarBeamform_GetResult_Response__fini(&array->data[i]);
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

radar_msg__action__RadarBeamform_GetResult_Response__Sequence *
radar_msg__action__RadarBeamform_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_msg__action__RadarBeamform_GetResult_Response__Sequence * array = (radar_msg__action__RadarBeamform_GetResult_Response__Sequence *)allocator.allocate(sizeof(radar_msg__action__RadarBeamform_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = radar_msg__action__RadarBeamform_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
radar_msg__action__RadarBeamform_GetResult_Response__Sequence__destroy(radar_msg__action__RadarBeamform_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    radar_msg__action__RadarBeamform_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
radar_msg__action__RadarBeamform_GetResult_Response__Sequence__are_equal(const radar_msg__action__RadarBeamform_GetResult_Response__Sequence * lhs, const radar_msg__action__RadarBeamform_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!radar_msg__action__RadarBeamform_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
radar_msg__action__RadarBeamform_GetResult_Response__Sequence__copy(
  const radar_msg__action__RadarBeamform_GetResult_Response__Sequence * input,
  radar_msg__action__RadarBeamform_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(radar_msg__action__RadarBeamform_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    radar_msg__action__RadarBeamform_GetResult_Response * data =
      (radar_msg__action__RadarBeamform_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!radar_msg__action__RadarBeamform_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          radar_msg__action__RadarBeamform_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!radar_msg__action__RadarBeamform_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "radar_msg/action/detail/radar_beamform__functions.h"

bool
radar_msg__action__RadarBeamform_FeedbackMessage__init(radar_msg__action__RadarBeamform_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    radar_msg__action__RadarBeamform_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!radar_msg__action__RadarBeamform_Feedback__init(&msg->feedback)) {
    radar_msg__action__RadarBeamform_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
radar_msg__action__RadarBeamform_FeedbackMessage__fini(radar_msg__action__RadarBeamform_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  radar_msg__action__RadarBeamform_Feedback__fini(&msg->feedback);
}

bool
radar_msg__action__RadarBeamform_FeedbackMessage__are_equal(const radar_msg__action__RadarBeamform_FeedbackMessage * lhs, const radar_msg__action__RadarBeamform_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!radar_msg__action__RadarBeamform_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
radar_msg__action__RadarBeamform_FeedbackMessage__copy(
  const radar_msg__action__RadarBeamform_FeedbackMessage * input,
  radar_msg__action__RadarBeamform_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!radar_msg__action__RadarBeamform_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

radar_msg__action__RadarBeamform_FeedbackMessage *
radar_msg__action__RadarBeamform_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_msg__action__RadarBeamform_FeedbackMessage * msg = (radar_msg__action__RadarBeamform_FeedbackMessage *)allocator.allocate(sizeof(radar_msg__action__RadarBeamform_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(radar_msg__action__RadarBeamform_FeedbackMessage));
  bool success = radar_msg__action__RadarBeamform_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
radar_msg__action__RadarBeamform_FeedbackMessage__destroy(radar_msg__action__RadarBeamform_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    radar_msg__action__RadarBeamform_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
radar_msg__action__RadarBeamform_FeedbackMessage__Sequence__init(radar_msg__action__RadarBeamform_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_msg__action__RadarBeamform_FeedbackMessage * data = NULL;

  if (size) {
    data = (radar_msg__action__RadarBeamform_FeedbackMessage *)allocator.zero_allocate(size, sizeof(radar_msg__action__RadarBeamform_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = radar_msg__action__RadarBeamform_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        radar_msg__action__RadarBeamform_FeedbackMessage__fini(&data[i - 1]);
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
radar_msg__action__RadarBeamform_FeedbackMessage__Sequence__fini(radar_msg__action__RadarBeamform_FeedbackMessage__Sequence * array)
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
      radar_msg__action__RadarBeamform_FeedbackMessage__fini(&array->data[i]);
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

radar_msg__action__RadarBeamform_FeedbackMessage__Sequence *
radar_msg__action__RadarBeamform_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_msg__action__RadarBeamform_FeedbackMessage__Sequence * array = (radar_msg__action__RadarBeamform_FeedbackMessage__Sequence *)allocator.allocate(sizeof(radar_msg__action__RadarBeamform_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = radar_msg__action__RadarBeamform_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
radar_msg__action__RadarBeamform_FeedbackMessage__Sequence__destroy(radar_msg__action__RadarBeamform_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    radar_msg__action__RadarBeamform_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
radar_msg__action__RadarBeamform_FeedbackMessage__Sequence__are_equal(const radar_msg__action__RadarBeamform_FeedbackMessage__Sequence * lhs, const radar_msg__action__RadarBeamform_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!radar_msg__action__RadarBeamform_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
radar_msg__action__RadarBeamform_FeedbackMessage__Sequence__copy(
  const radar_msg__action__RadarBeamform_FeedbackMessage__Sequence * input,
  radar_msg__action__RadarBeamform_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(radar_msg__action__RadarBeamform_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    radar_msg__action__RadarBeamform_FeedbackMessage * data =
      (radar_msg__action__RadarBeamform_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!radar_msg__action__RadarBeamform_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          radar_msg__action__RadarBeamform_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!radar_msg__action__RadarBeamform_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

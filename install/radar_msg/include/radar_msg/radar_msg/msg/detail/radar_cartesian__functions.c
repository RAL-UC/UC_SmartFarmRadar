// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from radar_msg:msg/RadarCartesian.idl
// generated code does not contain a copyright notice
#include "radar_msg/msg/detail/radar_cartesian__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `stamps`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `x`
// Member `y`
// Member `z`
// Member `gps_e`
// Member `gps_n`
// Member `gps_alt`
// Member `gps_qx`
// Member `gps_qy`
// Member `gps_qz`
// Member `gps_qw`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `gps_frame`
#include "rosidl_runtime_c/string_functions.h"

bool
radar_msg__msg__RadarCartesian__init(radar_msg__msg__RadarCartesian * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    radar_msg__msg__RadarCartesian__fini(msg);
    return false;
  }
  // stamps
  if (!builtin_interfaces__msg__Time__Sequence__init(&msg->stamps, 0)) {
    radar_msg__msg__RadarCartesian__fini(msg);
    return false;
  }
  // x
  if (!rosidl_runtime_c__float__Sequence__init(&msg->x, 0)) {
    radar_msg__msg__RadarCartesian__fini(msg);
    return false;
  }
  // y
  if (!rosidl_runtime_c__float__Sequence__init(&msg->y, 0)) {
    radar_msg__msg__RadarCartesian__fini(msg);
    return false;
  }
  // z
  if (!rosidl_runtime_c__float__Sequence__init(&msg->z, 0)) {
    radar_msg__msg__RadarCartesian__fini(msg);
    return false;
  }
  // gps_e
  if (!rosidl_runtime_c__float__Sequence__init(&msg->gps_e, 0)) {
    radar_msg__msg__RadarCartesian__fini(msg);
    return false;
  }
  // gps_n
  if (!rosidl_runtime_c__float__Sequence__init(&msg->gps_n, 0)) {
    radar_msg__msg__RadarCartesian__fini(msg);
    return false;
  }
  // gps_alt
  if (!rosidl_runtime_c__float__Sequence__init(&msg->gps_alt, 0)) {
    radar_msg__msg__RadarCartesian__fini(msg);
    return false;
  }
  // gps_qx
  if (!rosidl_runtime_c__float__Sequence__init(&msg->gps_qx, 0)) {
    radar_msg__msg__RadarCartesian__fini(msg);
    return false;
  }
  // gps_qy
  if (!rosidl_runtime_c__float__Sequence__init(&msg->gps_qy, 0)) {
    radar_msg__msg__RadarCartesian__fini(msg);
    return false;
  }
  // gps_qz
  if (!rosidl_runtime_c__float__Sequence__init(&msg->gps_qz, 0)) {
    radar_msg__msg__RadarCartesian__fini(msg);
    return false;
  }
  // gps_qw
  if (!rosidl_runtime_c__float__Sequence__init(&msg->gps_qw, 0)) {
    radar_msg__msg__RadarCartesian__fini(msg);
    return false;
  }
  // gps_frame
  if (!rosidl_runtime_c__String__init(&msg->gps_frame)) {
    radar_msg__msg__RadarCartesian__fini(msg);
    return false;
  }
  // robot_pose_id
  return true;
}

void
radar_msg__msg__RadarCartesian__fini(radar_msg__msg__RadarCartesian * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // stamps
  builtin_interfaces__msg__Time__Sequence__fini(&msg->stamps);
  // x
  rosidl_runtime_c__float__Sequence__fini(&msg->x);
  // y
  rosidl_runtime_c__float__Sequence__fini(&msg->y);
  // z
  rosidl_runtime_c__float__Sequence__fini(&msg->z);
  // gps_e
  rosidl_runtime_c__float__Sequence__fini(&msg->gps_e);
  // gps_n
  rosidl_runtime_c__float__Sequence__fini(&msg->gps_n);
  // gps_alt
  rosidl_runtime_c__float__Sequence__fini(&msg->gps_alt);
  // gps_qx
  rosidl_runtime_c__float__Sequence__fini(&msg->gps_qx);
  // gps_qy
  rosidl_runtime_c__float__Sequence__fini(&msg->gps_qy);
  // gps_qz
  rosidl_runtime_c__float__Sequence__fini(&msg->gps_qz);
  // gps_qw
  rosidl_runtime_c__float__Sequence__fini(&msg->gps_qw);
  // gps_frame
  rosidl_runtime_c__String__fini(&msg->gps_frame);
  // robot_pose_id
}

bool
radar_msg__msg__RadarCartesian__are_equal(const radar_msg__msg__RadarCartesian * lhs, const radar_msg__msg__RadarCartesian * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // stamps
  if (!builtin_interfaces__msg__Time__Sequence__are_equal(
      &(lhs->stamps), &(rhs->stamps)))
  {
    return false;
  }
  // x
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->x), &(rhs->x)))
  {
    return false;
  }
  // y
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->y), &(rhs->y)))
  {
    return false;
  }
  // z
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->z), &(rhs->z)))
  {
    return false;
  }
  // gps_e
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->gps_e), &(rhs->gps_e)))
  {
    return false;
  }
  // gps_n
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->gps_n), &(rhs->gps_n)))
  {
    return false;
  }
  // gps_alt
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->gps_alt), &(rhs->gps_alt)))
  {
    return false;
  }
  // gps_qx
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->gps_qx), &(rhs->gps_qx)))
  {
    return false;
  }
  // gps_qy
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->gps_qy), &(rhs->gps_qy)))
  {
    return false;
  }
  // gps_qz
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->gps_qz), &(rhs->gps_qz)))
  {
    return false;
  }
  // gps_qw
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->gps_qw), &(rhs->gps_qw)))
  {
    return false;
  }
  // gps_frame
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->gps_frame), &(rhs->gps_frame)))
  {
    return false;
  }
  // robot_pose_id
  if (lhs->robot_pose_id != rhs->robot_pose_id) {
    return false;
  }
  return true;
}

bool
radar_msg__msg__RadarCartesian__copy(
  const radar_msg__msg__RadarCartesian * input,
  radar_msg__msg__RadarCartesian * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // stamps
  if (!builtin_interfaces__msg__Time__Sequence__copy(
      &(input->stamps), &(output->stamps)))
  {
    return false;
  }
  // x
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->x), &(output->x)))
  {
    return false;
  }
  // y
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->y), &(output->y)))
  {
    return false;
  }
  // z
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->z), &(output->z)))
  {
    return false;
  }
  // gps_e
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->gps_e), &(output->gps_e)))
  {
    return false;
  }
  // gps_n
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->gps_n), &(output->gps_n)))
  {
    return false;
  }
  // gps_alt
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->gps_alt), &(output->gps_alt)))
  {
    return false;
  }
  // gps_qx
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->gps_qx), &(output->gps_qx)))
  {
    return false;
  }
  // gps_qy
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->gps_qy), &(output->gps_qy)))
  {
    return false;
  }
  // gps_qz
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->gps_qz), &(output->gps_qz)))
  {
    return false;
  }
  // gps_qw
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->gps_qw), &(output->gps_qw)))
  {
    return false;
  }
  // gps_frame
  if (!rosidl_runtime_c__String__copy(
      &(input->gps_frame), &(output->gps_frame)))
  {
    return false;
  }
  // robot_pose_id
  output->robot_pose_id = input->robot_pose_id;
  return true;
}

radar_msg__msg__RadarCartesian *
radar_msg__msg__RadarCartesian__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_msg__msg__RadarCartesian * msg = (radar_msg__msg__RadarCartesian *)allocator.allocate(sizeof(radar_msg__msg__RadarCartesian), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(radar_msg__msg__RadarCartesian));
  bool success = radar_msg__msg__RadarCartesian__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
radar_msg__msg__RadarCartesian__destroy(radar_msg__msg__RadarCartesian * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    radar_msg__msg__RadarCartesian__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
radar_msg__msg__RadarCartesian__Sequence__init(radar_msg__msg__RadarCartesian__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_msg__msg__RadarCartesian * data = NULL;

  if (size) {
    data = (radar_msg__msg__RadarCartesian *)allocator.zero_allocate(size, sizeof(radar_msg__msg__RadarCartesian), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = radar_msg__msg__RadarCartesian__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        radar_msg__msg__RadarCartesian__fini(&data[i - 1]);
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
radar_msg__msg__RadarCartesian__Sequence__fini(radar_msg__msg__RadarCartesian__Sequence * array)
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
      radar_msg__msg__RadarCartesian__fini(&array->data[i]);
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

radar_msg__msg__RadarCartesian__Sequence *
radar_msg__msg__RadarCartesian__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_msg__msg__RadarCartesian__Sequence * array = (radar_msg__msg__RadarCartesian__Sequence *)allocator.allocate(sizeof(radar_msg__msg__RadarCartesian__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = radar_msg__msg__RadarCartesian__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
radar_msg__msg__RadarCartesian__Sequence__destroy(radar_msg__msg__RadarCartesian__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    radar_msg__msg__RadarCartesian__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
radar_msg__msg__RadarCartesian__Sequence__are_equal(const radar_msg__msg__RadarCartesian__Sequence * lhs, const radar_msg__msg__RadarCartesian__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!radar_msg__msg__RadarCartesian__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
radar_msg__msg__RadarCartesian__Sequence__copy(
  const radar_msg__msg__RadarCartesian__Sequence * input,
  radar_msg__msg__RadarCartesian__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(radar_msg__msg__RadarCartesian);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    radar_msg__msg__RadarCartesian * data =
      (radar_msg__msg__RadarCartesian *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!radar_msg__msg__RadarCartesian__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          radar_msg__msg__RadarCartesian__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!radar_msg__msg__RadarCartesian__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

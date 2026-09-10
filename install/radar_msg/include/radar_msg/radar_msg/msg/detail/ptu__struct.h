// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from radar_msg:msg/Ptu.idl
// generated code does not contain a copyright notice

#ifndef RADAR_MSG__MSG__DETAIL__PTU__STRUCT_H_
#define RADAR_MSG__MSG__DETAIL__PTU__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/Ptu in the package radar_msg.
typedef struct radar_msg__msg__Ptu
{
  std_msgs__msg__Header header;
  int32_t pan_deg;
  int32_t tilt_deg;
} radar_msg__msg__Ptu;

// Struct for a sequence of radar_msg__msg__Ptu.
typedef struct radar_msg__msg__Ptu__Sequence
{
  radar_msg__msg__Ptu * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} radar_msg__msg__Ptu__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RADAR_MSG__MSG__DETAIL__PTU__STRUCT_H_

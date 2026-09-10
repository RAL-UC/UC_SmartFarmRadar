// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from radar_msg:msg/RadarCartesian.idl
// generated code does not contain a copyright notice

#ifndef RADAR_MSG__MSG__DETAIL__RADAR_CARTESIAN__STRUCT_H_
#define RADAR_MSG__MSG__DETAIL__RADAR_CARTESIAN__STRUCT_H_

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
// Member 'stamps'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'x'
// Member 'y'
// Member 'z'
// Member 'gps_e'
// Member 'gps_n'
// Member 'gps_alt'
// Member 'gps_qx'
// Member 'gps_qy'
// Member 'gps_qz'
// Member 'gps_qw'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'gps_frame'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/RadarCartesian in the package radar_msg.
typedef struct radar_msg__msg__RadarCartesian
{
  std_msgs__msg__Header header;
  /// marcas de tiempo
  builtin_interfaces__msg__Time__Sequence stamps;
  rosidl_runtime_c__float__Sequence x;
  rosidl_runtime_c__float__Sequence y;
  rosidl_runtime_c__float__Sequence z;
  /// datos de GPS
  rosidl_runtime_c__float__Sequence gps_e;
  rosidl_runtime_c__float__Sequence gps_n;
  rosidl_runtime_c__float__Sequence gps_alt;
  rosidl_runtime_c__float__Sequence gps_qx;
  rosidl_runtime_c__float__Sequence gps_qy;
  rosidl_runtime_c__float__Sequence gps_qz;
  rosidl_runtime_c__float__Sequence gps_qw;
  rosidl_runtime_c__String gps_frame;
  uint32_t robot_pose_id;
} radar_msg__msg__RadarCartesian;

// Struct for a sequence of radar_msg__msg__RadarCartesian.
typedef struct radar_msg__msg__RadarCartesian__Sequence
{
  radar_msg__msg__RadarCartesian * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} radar_msg__msg__RadarCartesian__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RADAR_MSG__MSG__DETAIL__RADAR_CARTESIAN__STRUCT_H_

// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from radar_msg:msg/RadarData.idl
// generated code does not contain a copyright notice

#ifndef RADAR_MSG__MSG__DETAIL__RADAR_DATA__STRUCT_H_
#define RADAR_MSG__MSG__DETAIL__RADAR_DATA__STRUCT_H_

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
// Member 'dtype'
#include "rosidl_runtime_c/string.h"
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/RadarData in the package radar_msg.
/**
  * Importar el tipo Header
 */
typedef struct radar_msg__msg__RadarData
{
  std_msgs__msg__Header header;
  /// RadarData.msg
  /// número de filas (por ejemplo 161)
  uint32_t rows;
  /// número de columnas (por ejemplo 4096)
  uint32_t cols;
  /// tipo de dato, p.ej. "float64"
  rosidl_runtime_c__String dtype;
  /// datos aplanados fila‑por‑fila
  rosidl_runtime_c__double__Sequence data;
} radar_msg__msg__RadarData;

// Struct for a sequence of radar_msg__msg__RadarData.
typedef struct radar_msg__msg__RadarData__Sequence
{
  radar_msg__msg__RadarData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} radar_msg__msg__RadarData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RADAR_MSG__MSG__DETAIL__RADAR_DATA__STRUCT_H_

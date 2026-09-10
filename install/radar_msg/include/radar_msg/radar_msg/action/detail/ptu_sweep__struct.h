// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from radar_msg:action/PtuSweep.idl
// generated code does not contain a copyright notice

#ifndef RADAR_MSG__ACTION__DETAIL__PTU_SWEEP__STRUCT_H_
#define RADAR_MSG__ACTION__DETAIL__PTU_SWEEP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'target_ptu'
#include "radar_msg/msg/detail/ptu__struct.h"

/// Struct defined in action/PtuSweep in the package radar_msg.
typedef struct radar_msg__action__PtuSweep_Goal
{
  /// si viene vacío, usa por defecto self.ptu_angles
  radar_msg__msg__Ptu target_ptu;
} radar_msg__action__PtuSweep_Goal;

// Struct for a sequence of radar_msg__action__PtuSweep_Goal.
typedef struct radar_msg__action__PtuSweep_Goal__Sequence
{
  radar_msg__action__PtuSweep_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} radar_msg__action__PtuSweep_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/PtuSweep in the package radar_msg.
typedef struct radar_msg__action__PtuSweep_Result
{
  bool success;
  rosidl_runtime_c__String message;
} radar_msg__action__PtuSweep_Result;

// Struct for a sequence of radar_msg__action__PtuSweep_Result.
typedef struct radar_msg__action__PtuSweep_Result__Sequence
{
  radar_msg__action__PtuSweep_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} radar_msg__action__PtuSweep_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'status'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/PtuSweep in the package radar_msg.
typedef struct radar_msg__action__PtuSweep_Feedback
{
  int32_t current_pan_deg;
  int32_t current_tilt_deg;
  rosidl_runtime_c__String status;
} radar_msg__action__PtuSweep_Feedback;

// Struct for a sequence of radar_msg__action__PtuSweep_Feedback.
typedef struct radar_msg__action__PtuSweep_Feedback__Sequence
{
  radar_msg__action__PtuSweep_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} radar_msg__action__PtuSweep_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "radar_msg/action/detail/ptu_sweep__struct.h"

/// Struct defined in action/PtuSweep in the package radar_msg.
typedef struct radar_msg__action__PtuSweep_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  radar_msg__action__PtuSweep_Goal goal;
} radar_msg__action__PtuSweep_SendGoal_Request;

// Struct for a sequence of radar_msg__action__PtuSweep_SendGoal_Request.
typedef struct radar_msg__action__PtuSweep_SendGoal_Request__Sequence
{
  radar_msg__action__PtuSweep_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} radar_msg__action__PtuSweep_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/PtuSweep in the package radar_msg.
typedef struct radar_msg__action__PtuSweep_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} radar_msg__action__PtuSweep_SendGoal_Response;

// Struct for a sequence of radar_msg__action__PtuSweep_SendGoal_Response.
typedef struct radar_msg__action__PtuSweep_SendGoal_Response__Sequence
{
  radar_msg__action__PtuSweep_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} radar_msg__action__PtuSweep_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/PtuSweep in the package radar_msg.
typedef struct radar_msg__action__PtuSweep_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} radar_msg__action__PtuSweep_GetResult_Request;

// Struct for a sequence of radar_msg__action__PtuSweep_GetResult_Request.
typedef struct radar_msg__action__PtuSweep_GetResult_Request__Sequence
{
  radar_msg__action__PtuSweep_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} radar_msg__action__PtuSweep_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "radar_msg/action/detail/ptu_sweep__struct.h"

/// Struct defined in action/PtuSweep in the package radar_msg.
typedef struct radar_msg__action__PtuSweep_GetResult_Response
{
  int8_t status;
  radar_msg__action__PtuSweep_Result result;
} radar_msg__action__PtuSweep_GetResult_Response;

// Struct for a sequence of radar_msg__action__PtuSweep_GetResult_Response.
typedef struct radar_msg__action__PtuSweep_GetResult_Response__Sequence
{
  radar_msg__action__PtuSweep_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} radar_msg__action__PtuSweep_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "radar_msg/action/detail/ptu_sweep__struct.h"

/// Struct defined in action/PtuSweep in the package radar_msg.
typedef struct radar_msg__action__PtuSweep_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  radar_msg__action__PtuSweep_Feedback feedback;
} radar_msg__action__PtuSweep_FeedbackMessage;

// Struct for a sequence of radar_msg__action__PtuSweep_FeedbackMessage.
typedef struct radar_msg__action__PtuSweep_FeedbackMessage__Sequence
{
  radar_msg__action__PtuSweep_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} radar_msg__action__PtuSweep_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RADAR_MSG__ACTION__DETAIL__PTU_SWEEP__STRUCT_H_

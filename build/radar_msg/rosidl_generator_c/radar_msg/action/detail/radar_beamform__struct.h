// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from radar_msg:action/RadarBeamform.idl
// generated code does not contain a copyright notice

#ifndef RADAR_MSG__ACTION__DETAIL__RADAR_BEAMFORM__STRUCT_H_
#define RADAR_MSG__ACTION__DETAIL__RADAR_BEAMFORM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in action/RadarBeamform in the package radar_msg.
typedef struct radar_msg__action__RadarBeamform_Goal
{
  uint8_t structure_needs_at_least_one_member;
} radar_msg__action__RadarBeamform_Goal;

// Struct for a sequence of radar_msg__action__RadarBeamform_Goal.
typedef struct radar_msg__action__RadarBeamform_Goal__Sequence
{
  radar_msg__action__RadarBeamform_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} radar_msg__action__RadarBeamform_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"
// Member 'radar_data'
#include "radar_msg/msg/detail/radar_data__struct.h"

/// Struct defined in action/RadarBeamform in the package radar_msg.
typedef struct radar_msg__action__RadarBeamform_Result
{
  bool success;
  rosidl_runtime_c__String message;
  radar_msg__msg__RadarData radar_data;
} radar_msg__action__RadarBeamform_Result;

// Struct for a sequence of radar_msg__action__RadarBeamform_Result.
typedef struct radar_msg__action__RadarBeamform_Result__Sequence
{
  radar_msg__action__RadarBeamform_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} radar_msg__action__RadarBeamform_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'status'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/RadarBeamform in the package radar_msg.
typedef struct radar_msg__action__RadarBeamform_Feedback
{
  rosidl_runtime_c__String status;
} radar_msg__action__RadarBeamform_Feedback;

// Struct for a sequence of radar_msg__action__RadarBeamform_Feedback.
typedef struct radar_msg__action__RadarBeamform_Feedback__Sequence
{
  radar_msg__action__RadarBeamform_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} radar_msg__action__RadarBeamform_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "radar_msg/action/detail/radar_beamform__struct.h"

/// Struct defined in action/RadarBeamform in the package radar_msg.
typedef struct radar_msg__action__RadarBeamform_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  radar_msg__action__RadarBeamform_Goal goal;
} radar_msg__action__RadarBeamform_SendGoal_Request;

// Struct for a sequence of radar_msg__action__RadarBeamform_SendGoal_Request.
typedef struct radar_msg__action__RadarBeamform_SendGoal_Request__Sequence
{
  radar_msg__action__RadarBeamform_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} radar_msg__action__RadarBeamform_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/RadarBeamform in the package radar_msg.
typedef struct radar_msg__action__RadarBeamform_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} radar_msg__action__RadarBeamform_SendGoal_Response;

// Struct for a sequence of radar_msg__action__RadarBeamform_SendGoal_Response.
typedef struct radar_msg__action__RadarBeamform_SendGoal_Response__Sequence
{
  radar_msg__action__RadarBeamform_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} radar_msg__action__RadarBeamform_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/RadarBeamform in the package radar_msg.
typedef struct radar_msg__action__RadarBeamform_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} radar_msg__action__RadarBeamform_GetResult_Request;

// Struct for a sequence of radar_msg__action__RadarBeamform_GetResult_Request.
typedef struct radar_msg__action__RadarBeamform_GetResult_Request__Sequence
{
  radar_msg__action__RadarBeamform_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} radar_msg__action__RadarBeamform_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "radar_msg/action/detail/radar_beamform__struct.h"

/// Struct defined in action/RadarBeamform in the package radar_msg.
typedef struct radar_msg__action__RadarBeamform_GetResult_Response
{
  int8_t status;
  radar_msg__action__RadarBeamform_Result result;
} radar_msg__action__RadarBeamform_GetResult_Response;

// Struct for a sequence of radar_msg__action__RadarBeamform_GetResult_Response.
typedef struct radar_msg__action__RadarBeamform_GetResult_Response__Sequence
{
  radar_msg__action__RadarBeamform_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} radar_msg__action__RadarBeamform_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "radar_msg/action/detail/radar_beamform__struct.h"

/// Struct defined in action/RadarBeamform in the package radar_msg.
typedef struct radar_msg__action__RadarBeamform_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  radar_msg__action__RadarBeamform_Feedback feedback;
} radar_msg__action__RadarBeamform_FeedbackMessage;

// Struct for a sequence of radar_msg__action__RadarBeamform_FeedbackMessage.
typedef struct radar_msg__action__RadarBeamform_FeedbackMessage__Sequence
{
  radar_msg__action__RadarBeamform_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} radar_msg__action__RadarBeamform_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RADAR_MSG__ACTION__DETAIL__RADAR_BEAMFORM__STRUCT_H_

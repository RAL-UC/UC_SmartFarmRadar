// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from radar_msg:action/PtuSweep.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "radar_msg/action/detail/ptu_sweep__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace radar_msg
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PtuSweep_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PtuSweep_Goal_type_support_ids_t;

static const _PtuSweep_Goal_type_support_ids_t _PtuSweep_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PtuSweep_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PtuSweep_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PtuSweep_Goal_type_support_symbol_names_t _PtuSweep_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, radar_msg, action, PtuSweep_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, radar_msg, action, PtuSweep_Goal)),
  }
};

typedef struct _PtuSweep_Goal_type_support_data_t
{
  void * data[2];
} _PtuSweep_Goal_type_support_data_t;

static _PtuSweep_Goal_type_support_data_t _PtuSweep_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PtuSweep_Goal_message_typesupport_map = {
  2,
  "radar_msg",
  &_PtuSweep_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_PtuSweep_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_PtuSweep_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PtuSweep_Goal_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PtuSweep_Goal_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace radar_msg

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<radar_msg::action::PtuSweep_Goal>()
{
  return &::radar_msg::action::rosidl_typesupport_cpp::PtuSweep_Goal_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, radar_msg, action, PtuSweep_Goal)() {
  return get_message_type_support_handle<radar_msg::action::PtuSweep_Goal>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "radar_msg/action/detail/ptu_sweep__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace radar_msg
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PtuSweep_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PtuSweep_Result_type_support_ids_t;

static const _PtuSweep_Result_type_support_ids_t _PtuSweep_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PtuSweep_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PtuSweep_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PtuSweep_Result_type_support_symbol_names_t _PtuSweep_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, radar_msg, action, PtuSweep_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, radar_msg, action, PtuSweep_Result)),
  }
};

typedef struct _PtuSweep_Result_type_support_data_t
{
  void * data[2];
} _PtuSweep_Result_type_support_data_t;

static _PtuSweep_Result_type_support_data_t _PtuSweep_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PtuSweep_Result_message_typesupport_map = {
  2,
  "radar_msg",
  &_PtuSweep_Result_message_typesupport_ids.typesupport_identifier[0],
  &_PtuSweep_Result_message_typesupport_symbol_names.symbol_name[0],
  &_PtuSweep_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PtuSweep_Result_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PtuSweep_Result_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace radar_msg

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<radar_msg::action::PtuSweep_Result>()
{
  return &::radar_msg::action::rosidl_typesupport_cpp::PtuSweep_Result_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, radar_msg, action, PtuSweep_Result)() {
  return get_message_type_support_handle<radar_msg::action::PtuSweep_Result>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "radar_msg/action/detail/ptu_sweep__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace radar_msg
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PtuSweep_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PtuSweep_Feedback_type_support_ids_t;

static const _PtuSweep_Feedback_type_support_ids_t _PtuSweep_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PtuSweep_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PtuSweep_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PtuSweep_Feedback_type_support_symbol_names_t _PtuSweep_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, radar_msg, action, PtuSweep_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, radar_msg, action, PtuSweep_Feedback)),
  }
};

typedef struct _PtuSweep_Feedback_type_support_data_t
{
  void * data[2];
} _PtuSweep_Feedback_type_support_data_t;

static _PtuSweep_Feedback_type_support_data_t _PtuSweep_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PtuSweep_Feedback_message_typesupport_map = {
  2,
  "radar_msg",
  &_PtuSweep_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_PtuSweep_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_PtuSweep_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PtuSweep_Feedback_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PtuSweep_Feedback_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace radar_msg

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<radar_msg::action::PtuSweep_Feedback>()
{
  return &::radar_msg::action::rosidl_typesupport_cpp::PtuSweep_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, radar_msg, action, PtuSweep_Feedback)() {
  return get_message_type_support_handle<radar_msg::action::PtuSweep_Feedback>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "radar_msg/action/detail/ptu_sweep__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace radar_msg
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PtuSweep_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PtuSweep_SendGoal_Request_type_support_ids_t;

static const _PtuSweep_SendGoal_Request_type_support_ids_t _PtuSweep_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PtuSweep_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PtuSweep_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PtuSweep_SendGoal_Request_type_support_symbol_names_t _PtuSweep_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, radar_msg, action, PtuSweep_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, radar_msg, action, PtuSweep_SendGoal_Request)),
  }
};

typedef struct _PtuSweep_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _PtuSweep_SendGoal_Request_type_support_data_t;

static _PtuSweep_SendGoal_Request_type_support_data_t _PtuSweep_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PtuSweep_SendGoal_Request_message_typesupport_map = {
  2,
  "radar_msg",
  &_PtuSweep_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_PtuSweep_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_PtuSweep_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PtuSweep_SendGoal_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PtuSweep_SendGoal_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace radar_msg

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<radar_msg::action::PtuSweep_SendGoal_Request>()
{
  return &::radar_msg::action::rosidl_typesupport_cpp::PtuSweep_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, radar_msg, action, PtuSweep_SendGoal_Request)() {
  return get_message_type_support_handle<radar_msg::action::PtuSweep_SendGoal_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "radar_msg/action/detail/ptu_sweep__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace radar_msg
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PtuSweep_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PtuSweep_SendGoal_Response_type_support_ids_t;

static const _PtuSweep_SendGoal_Response_type_support_ids_t _PtuSweep_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PtuSweep_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PtuSweep_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PtuSweep_SendGoal_Response_type_support_symbol_names_t _PtuSweep_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, radar_msg, action, PtuSweep_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, radar_msg, action, PtuSweep_SendGoal_Response)),
  }
};

typedef struct _PtuSweep_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _PtuSweep_SendGoal_Response_type_support_data_t;

static _PtuSweep_SendGoal_Response_type_support_data_t _PtuSweep_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PtuSweep_SendGoal_Response_message_typesupport_map = {
  2,
  "radar_msg",
  &_PtuSweep_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_PtuSweep_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_PtuSweep_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PtuSweep_SendGoal_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PtuSweep_SendGoal_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace radar_msg

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<radar_msg::action::PtuSweep_SendGoal_Response>()
{
  return &::radar_msg::action::rosidl_typesupport_cpp::PtuSweep_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, radar_msg, action, PtuSweep_SendGoal_Response)() {
  return get_message_type_support_handle<radar_msg::action::PtuSweep_SendGoal_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "radar_msg/action/detail/ptu_sweep__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace radar_msg
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PtuSweep_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PtuSweep_SendGoal_type_support_ids_t;

static const _PtuSweep_SendGoal_type_support_ids_t _PtuSweep_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PtuSweep_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PtuSweep_SendGoal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PtuSweep_SendGoal_type_support_symbol_names_t _PtuSweep_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, radar_msg, action, PtuSweep_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, radar_msg, action, PtuSweep_SendGoal)),
  }
};

typedef struct _PtuSweep_SendGoal_type_support_data_t
{
  void * data[2];
} _PtuSweep_SendGoal_type_support_data_t;

static _PtuSweep_SendGoal_type_support_data_t _PtuSweep_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PtuSweep_SendGoal_service_typesupport_map = {
  2,
  "radar_msg",
  &_PtuSweep_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_PtuSweep_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_PtuSweep_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t PtuSweep_SendGoal_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PtuSweep_SendGoal_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace radar_msg

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<radar_msg::action::PtuSweep_SendGoal>()
{
  return &::radar_msg::action::rosidl_typesupport_cpp::PtuSweep_SendGoal_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, radar_msg, action, PtuSweep_SendGoal)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<radar_msg::action::PtuSweep_SendGoal>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "radar_msg/action/detail/ptu_sweep__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace radar_msg
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PtuSweep_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PtuSweep_GetResult_Request_type_support_ids_t;

static const _PtuSweep_GetResult_Request_type_support_ids_t _PtuSweep_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PtuSweep_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PtuSweep_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PtuSweep_GetResult_Request_type_support_symbol_names_t _PtuSweep_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, radar_msg, action, PtuSweep_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, radar_msg, action, PtuSweep_GetResult_Request)),
  }
};

typedef struct _PtuSweep_GetResult_Request_type_support_data_t
{
  void * data[2];
} _PtuSweep_GetResult_Request_type_support_data_t;

static _PtuSweep_GetResult_Request_type_support_data_t _PtuSweep_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PtuSweep_GetResult_Request_message_typesupport_map = {
  2,
  "radar_msg",
  &_PtuSweep_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_PtuSweep_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_PtuSweep_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PtuSweep_GetResult_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PtuSweep_GetResult_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace radar_msg

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<radar_msg::action::PtuSweep_GetResult_Request>()
{
  return &::radar_msg::action::rosidl_typesupport_cpp::PtuSweep_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, radar_msg, action, PtuSweep_GetResult_Request)() {
  return get_message_type_support_handle<radar_msg::action::PtuSweep_GetResult_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "radar_msg/action/detail/ptu_sweep__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace radar_msg
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PtuSweep_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PtuSweep_GetResult_Response_type_support_ids_t;

static const _PtuSweep_GetResult_Response_type_support_ids_t _PtuSweep_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PtuSweep_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PtuSweep_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PtuSweep_GetResult_Response_type_support_symbol_names_t _PtuSweep_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, radar_msg, action, PtuSweep_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, radar_msg, action, PtuSweep_GetResult_Response)),
  }
};

typedef struct _PtuSweep_GetResult_Response_type_support_data_t
{
  void * data[2];
} _PtuSweep_GetResult_Response_type_support_data_t;

static _PtuSweep_GetResult_Response_type_support_data_t _PtuSweep_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PtuSweep_GetResult_Response_message_typesupport_map = {
  2,
  "radar_msg",
  &_PtuSweep_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_PtuSweep_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_PtuSweep_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PtuSweep_GetResult_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PtuSweep_GetResult_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace radar_msg

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<radar_msg::action::PtuSweep_GetResult_Response>()
{
  return &::radar_msg::action::rosidl_typesupport_cpp::PtuSweep_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, radar_msg, action, PtuSweep_GetResult_Response)() {
  return get_message_type_support_handle<radar_msg::action::PtuSweep_GetResult_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "radar_msg/action/detail/ptu_sweep__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace radar_msg
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PtuSweep_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PtuSweep_GetResult_type_support_ids_t;

static const _PtuSweep_GetResult_type_support_ids_t _PtuSweep_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PtuSweep_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PtuSweep_GetResult_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PtuSweep_GetResult_type_support_symbol_names_t _PtuSweep_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, radar_msg, action, PtuSweep_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, radar_msg, action, PtuSweep_GetResult)),
  }
};

typedef struct _PtuSweep_GetResult_type_support_data_t
{
  void * data[2];
} _PtuSweep_GetResult_type_support_data_t;

static _PtuSweep_GetResult_type_support_data_t _PtuSweep_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PtuSweep_GetResult_service_typesupport_map = {
  2,
  "radar_msg",
  &_PtuSweep_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_PtuSweep_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_PtuSweep_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t PtuSweep_GetResult_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PtuSweep_GetResult_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace radar_msg

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<radar_msg::action::PtuSweep_GetResult>()
{
  return &::radar_msg::action::rosidl_typesupport_cpp::PtuSweep_GetResult_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, radar_msg, action, PtuSweep_GetResult)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<radar_msg::action::PtuSweep_GetResult>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "radar_msg/action/detail/ptu_sweep__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace radar_msg
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PtuSweep_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PtuSweep_FeedbackMessage_type_support_ids_t;

static const _PtuSweep_FeedbackMessage_type_support_ids_t _PtuSweep_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PtuSweep_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PtuSweep_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PtuSweep_FeedbackMessage_type_support_symbol_names_t _PtuSweep_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, radar_msg, action, PtuSweep_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, radar_msg, action, PtuSweep_FeedbackMessage)),
  }
};

typedef struct _PtuSweep_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _PtuSweep_FeedbackMessage_type_support_data_t;

static _PtuSweep_FeedbackMessage_type_support_data_t _PtuSweep_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PtuSweep_FeedbackMessage_message_typesupport_map = {
  2,
  "radar_msg",
  &_PtuSweep_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_PtuSweep_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_PtuSweep_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PtuSweep_FeedbackMessage_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PtuSweep_FeedbackMessage_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace radar_msg

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<radar_msg::action::PtuSweep_FeedbackMessage>()
{
  return &::radar_msg::action::rosidl_typesupport_cpp::PtuSweep_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, radar_msg, action, PtuSweep_FeedbackMessage)() {
  return get_message_type_support_handle<radar_msg::action::PtuSweep_FeedbackMessage>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
// already included above
// #include "radar_msg/action/detail/ptu_sweep__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_typesupport_cpp/action_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"

namespace radar_msg
{

namespace action
{

namespace rosidl_typesupport_cpp
{

static rosidl_action_type_support_t PtuSweep_action_type_support_handle = {
  NULL, NULL, NULL, NULL, NULL};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace radar_msg

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
get_action_type_support_handle<radar_msg::action::PtuSweep>()
{
  using ::radar_msg::action::rosidl_typesupport_cpp::PtuSweep_action_type_support_handle;
  // Thread-safe by always writing the same values to the static struct
  PtuSweep_action_type_support_handle.goal_service_type_support = get_service_type_support_handle<::radar_msg::action::PtuSweep::Impl::SendGoalService>();
  PtuSweep_action_type_support_handle.result_service_type_support = get_service_type_support_handle<::radar_msg::action::PtuSweep::Impl::GetResultService>();
  PtuSweep_action_type_support_handle.cancel_service_type_support = get_service_type_support_handle<::radar_msg::action::PtuSweep::Impl::CancelGoalService>();
  PtuSweep_action_type_support_handle.feedback_message_type_support = get_message_type_support_handle<::radar_msg::action::PtuSweep::Impl::FeedbackMessage>();
  PtuSweep_action_type_support_handle.status_message_type_support = get_message_type_support_handle<::radar_msg::action::PtuSweep::Impl::GoalStatusMessage>();
  return &PtuSweep_action_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(rosidl_typesupport_cpp, radar_msg, action, PtuSweep)() {
  return ::rosidl_typesupport_cpp::get_action_type_support_handle<radar_msg::action::PtuSweep>();
}

#ifdef __cplusplus
}
#endif

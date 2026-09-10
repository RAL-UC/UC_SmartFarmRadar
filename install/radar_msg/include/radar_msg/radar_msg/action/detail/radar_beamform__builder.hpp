// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from radar_msg:action/RadarBeamform.idl
// generated code does not contain a copyright notice

#ifndef RADAR_MSG__ACTION__DETAIL__RADAR_BEAMFORM__BUILDER_HPP_
#define RADAR_MSG__ACTION__DETAIL__RADAR_BEAMFORM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "radar_msg/action/detail/radar_beamform__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace radar_msg
{

namespace action
{


}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::radar_msg::action::RadarBeamform_Goal>()
{
  return ::radar_msg::action::RadarBeamform_Goal(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace radar_msg


namespace radar_msg
{

namespace action
{

namespace builder
{

class Init_RadarBeamform_Result_radar_data
{
public:
  explicit Init_RadarBeamform_Result_radar_data(::radar_msg::action::RadarBeamform_Result & msg)
  : msg_(msg)
  {}
  ::radar_msg::action::RadarBeamform_Result radar_data(::radar_msg::action::RadarBeamform_Result::_radar_data_type arg)
  {
    msg_.radar_data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::radar_msg::action::RadarBeamform_Result msg_;
};

class Init_RadarBeamform_Result_message
{
public:
  explicit Init_RadarBeamform_Result_message(::radar_msg::action::RadarBeamform_Result & msg)
  : msg_(msg)
  {}
  Init_RadarBeamform_Result_radar_data message(::radar_msg::action::RadarBeamform_Result::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_RadarBeamform_Result_radar_data(msg_);
  }

private:
  ::radar_msg::action::RadarBeamform_Result msg_;
};

class Init_RadarBeamform_Result_success
{
public:
  Init_RadarBeamform_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RadarBeamform_Result_message success(::radar_msg::action::RadarBeamform_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_RadarBeamform_Result_message(msg_);
  }

private:
  ::radar_msg::action::RadarBeamform_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::radar_msg::action::RadarBeamform_Result>()
{
  return radar_msg::action::builder::Init_RadarBeamform_Result_success();
}

}  // namespace radar_msg


namespace radar_msg
{

namespace action
{

namespace builder
{

class Init_RadarBeamform_Feedback_status
{
public:
  Init_RadarBeamform_Feedback_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::radar_msg::action::RadarBeamform_Feedback status(::radar_msg::action::RadarBeamform_Feedback::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::radar_msg::action::RadarBeamform_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::radar_msg::action::RadarBeamform_Feedback>()
{
  return radar_msg::action::builder::Init_RadarBeamform_Feedback_status();
}

}  // namespace radar_msg


namespace radar_msg
{

namespace action
{

namespace builder
{

class Init_RadarBeamform_SendGoal_Request_goal
{
public:
  explicit Init_RadarBeamform_SendGoal_Request_goal(::radar_msg::action::RadarBeamform_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::radar_msg::action::RadarBeamform_SendGoal_Request goal(::radar_msg::action::RadarBeamform_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::radar_msg::action::RadarBeamform_SendGoal_Request msg_;
};

class Init_RadarBeamform_SendGoal_Request_goal_id
{
public:
  Init_RadarBeamform_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RadarBeamform_SendGoal_Request_goal goal_id(::radar_msg::action::RadarBeamform_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_RadarBeamform_SendGoal_Request_goal(msg_);
  }

private:
  ::radar_msg::action::RadarBeamform_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::radar_msg::action::RadarBeamform_SendGoal_Request>()
{
  return radar_msg::action::builder::Init_RadarBeamform_SendGoal_Request_goal_id();
}

}  // namespace radar_msg


namespace radar_msg
{

namespace action
{

namespace builder
{

class Init_RadarBeamform_SendGoal_Response_stamp
{
public:
  explicit Init_RadarBeamform_SendGoal_Response_stamp(::radar_msg::action::RadarBeamform_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::radar_msg::action::RadarBeamform_SendGoal_Response stamp(::radar_msg::action::RadarBeamform_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::radar_msg::action::RadarBeamform_SendGoal_Response msg_;
};

class Init_RadarBeamform_SendGoal_Response_accepted
{
public:
  Init_RadarBeamform_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RadarBeamform_SendGoal_Response_stamp accepted(::radar_msg::action::RadarBeamform_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_RadarBeamform_SendGoal_Response_stamp(msg_);
  }

private:
  ::radar_msg::action::RadarBeamform_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::radar_msg::action::RadarBeamform_SendGoal_Response>()
{
  return radar_msg::action::builder::Init_RadarBeamform_SendGoal_Response_accepted();
}

}  // namespace radar_msg


namespace radar_msg
{

namespace action
{

namespace builder
{

class Init_RadarBeamform_GetResult_Request_goal_id
{
public:
  Init_RadarBeamform_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::radar_msg::action::RadarBeamform_GetResult_Request goal_id(::radar_msg::action::RadarBeamform_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::radar_msg::action::RadarBeamform_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::radar_msg::action::RadarBeamform_GetResult_Request>()
{
  return radar_msg::action::builder::Init_RadarBeamform_GetResult_Request_goal_id();
}

}  // namespace radar_msg


namespace radar_msg
{

namespace action
{

namespace builder
{

class Init_RadarBeamform_GetResult_Response_result
{
public:
  explicit Init_RadarBeamform_GetResult_Response_result(::radar_msg::action::RadarBeamform_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::radar_msg::action::RadarBeamform_GetResult_Response result(::radar_msg::action::RadarBeamform_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::radar_msg::action::RadarBeamform_GetResult_Response msg_;
};

class Init_RadarBeamform_GetResult_Response_status
{
public:
  Init_RadarBeamform_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RadarBeamform_GetResult_Response_result status(::radar_msg::action::RadarBeamform_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_RadarBeamform_GetResult_Response_result(msg_);
  }

private:
  ::radar_msg::action::RadarBeamform_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::radar_msg::action::RadarBeamform_GetResult_Response>()
{
  return radar_msg::action::builder::Init_RadarBeamform_GetResult_Response_status();
}

}  // namespace radar_msg


namespace radar_msg
{

namespace action
{

namespace builder
{

class Init_RadarBeamform_FeedbackMessage_feedback
{
public:
  explicit Init_RadarBeamform_FeedbackMessage_feedback(::radar_msg::action::RadarBeamform_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::radar_msg::action::RadarBeamform_FeedbackMessage feedback(::radar_msg::action::RadarBeamform_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::radar_msg::action::RadarBeamform_FeedbackMessage msg_;
};

class Init_RadarBeamform_FeedbackMessage_goal_id
{
public:
  Init_RadarBeamform_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RadarBeamform_FeedbackMessage_feedback goal_id(::radar_msg::action::RadarBeamform_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_RadarBeamform_FeedbackMessage_feedback(msg_);
  }

private:
  ::radar_msg::action::RadarBeamform_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::radar_msg::action::RadarBeamform_FeedbackMessage>()
{
  return radar_msg::action::builder::Init_RadarBeamform_FeedbackMessage_goal_id();
}

}  // namespace radar_msg

#endif  // RADAR_MSG__ACTION__DETAIL__RADAR_BEAMFORM__BUILDER_HPP_

// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from radar_msg:action/PtuSweep.idl
// generated code does not contain a copyright notice

#ifndef RADAR_MSG__ACTION__DETAIL__PTU_SWEEP__BUILDER_HPP_
#define RADAR_MSG__ACTION__DETAIL__PTU_SWEEP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "radar_msg/action/detail/ptu_sweep__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace radar_msg
{

namespace action
{

namespace builder
{

class Init_PtuSweep_Goal_target_ptu
{
public:
  Init_PtuSweep_Goal_target_ptu()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::radar_msg::action::PtuSweep_Goal target_ptu(::radar_msg::action::PtuSweep_Goal::_target_ptu_type arg)
  {
    msg_.target_ptu = std::move(arg);
    return std::move(msg_);
  }

private:
  ::radar_msg::action::PtuSweep_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::radar_msg::action::PtuSweep_Goal>()
{
  return radar_msg::action::builder::Init_PtuSweep_Goal_target_ptu();
}

}  // namespace radar_msg


namespace radar_msg
{

namespace action
{

namespace builder
{

class Init_PtuSweep_Result_message
{
public:
  explicit Init_PtuSweep_Result_message(::radar_msg::action::PtuSweep_Result & msg)
  : msg_(msg)
  {}
  ::radar_msg::action::PtuSweep_Result message(::radar_msg::action::PtuSweep_Result::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::radar_msg::action::PtuSweep_Result msg_;
};

class Init_PtuSweep_Result_success
{
public:
  Init_PtuSweep_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PtuSweep_Result_message success(::radar_msg::action::PtuSweep_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_PtuSweep_Result_message(msg_);
  }

private:
  ::radar_msg::action::PtuSweep_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::radar_msg::action::PtuSweep_Result>()
{
  return radar_msg::action::builder::Init_PtuSweep_Result_success();
}

}  // namespace radar_msg


namespace radar_msg
{

namespace action
{

namespace builder
{

class Init_PtuSweep_Feedback_status
{
public:
  explicit Init_PtuSweep_Feedback_status(::radar_msg::action::PtuSweep_Feedback & msg)
  : msg_(msg)
  {}
  ::radar_msg::action::PtuSweep_Feedback status(::radar_msg::action::PtuSweep_Feedback::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::radar_msg::action::PtuSweep_Feedback msg_;
};

class Init_PtuSweep_Feedback_current_tilt_deg
{
public:
  explicit Init_PtuSweep_Feedback_current_tilt_deg(::radar_msg::action::PtuSweep_Feedback & msg)
  : msg_(msg)
  {}
  Init_PtuSweep_Feedback_status current_tilt_deg(::radar_msg::action::PtuSweep_Feedback::_current_tilt_deg_type arg)
  {
    msg_.current_tilt_deg = std::move(arg);
    return Init_PtuSweep_Feedback_status(msg_);
  }

private:
  ::radar_msg::action::PtuSweep_Feedback msg_;
};

class Init_PtuSweep_Feedback_current_pan_deg
{
public:
  Init_PtuSweep_Feedback_current_pan_deg()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PtuSweep_Feedback_current_tilt_deg current_pan_deg(::radar_msg::action::PtuSweep_Feedback::_current_pan_deg_type arg)
  {
    msg_.current_pan_deg = std::move(arg);
    return Init_PtuSweep_Feedback_current_tilt_deg(msg_);
  }

private:
  ::radar_msg::action::PtuSweep_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::radar_msg::action::PtuSweep_Feedback>()
{
  return radar_msg::action::builder::Init_PtuSweep_Feedback_current_pan_deg();
}

}  // namespace radar_msg


namespace radar_msg
{

namespace action
{

namespace builder
{

class Init_PtuSweep_SendGoal_Request_goal
{
public:
  explicit Init_PtuSweep_SendGoal_Request_goal(::radar_msg::action::PtuSweep_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::radar_msg::action::PtuSweep_SendGoal_Request goal(::radar_msg::action::PtuSweep_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::radar_msg::action::PtuSweep_SendGoal_Request msg_;
};

class Init_PtuSweep_SendGoal_Request_goal_id
{
public:
  Init_PtuSweep_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PtuSweep_SendGoal_Request_goal goal_id(::radar_msg::action::PtuSweep_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_PtuSweep_SendGoal_Request_goal(msg_);
  }

private:
  ::radar_msg::action::PtuSweep_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::radar_msg::action::PtuSweep_SendGoal_Request>()
{
  return radar_msg::action::builder::Init_PtuSweep_SendGoal_Request_goal_id();
}

}  // namespace radar_msg


namespace radar_msg
{

namespace action
{

namespace builder
{

class Init_PtuSweep_SendGoal_Response_stamp
{
public:
  explicit Init_PtuSweep_SendGoal_Response_stamp(::radar_msg::action::PtuSweep_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::radar_msg::action::PtuSweep_SendGoal_Response stamp(::radar_msg::action::PtuSweep_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::radar_msg::action::PtuSweep_SendGoal_Response msg_;
};

class Init_PtuSweep_SendGoal_Response_accepted
{
public:
  Init_PtuSweep_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PtuSweep_SendGoal_Response_stamp accepted(::radar_msg::action::PtuSweep_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_PtuSweep_SendGoal_Response_stamp(msg_);
  }

private:
  ::radar_msg::action::PtuSweep_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::radar_msg::action::PtuSweep_SendGoal_Response>()
{
  return radar_msg::action::builder::Init_PtuSweep_SendGoal_Response_accepted();
}

}  // namespace radar_msg


namespace radar_msg
{

namespace action
{

namespace builder
{

class Init_PtuSweep_GetResult_Request_goal_id
{
public:
  Init_PtuSweep_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::radar_msg::action::PtuSweep_GetResult_Request goal_id(::radar_msg::action::PtuSweep_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::radar_msg::action::PtuSweep_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::radar_msg::action::PtuSweep_GetResult_Request>()
{
  return radar_msg::action::builder::Init_PtuSweep_GetResult_Request_goal_id();
}

}  // namespace radar_msg


namespace radar_msg
{

namespace action
{

namespace builder
{

class Init_PtuSweep_GetResult_Response_result
{
public:
  explicit Init_PtuSweep_GetResult_Response_result(::radar_msg::action::PtuSweep_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::radar_msg::action::PtuSweep_GetResult_Response result(::radar_msg::action::PtuSweep_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::radar_msg::action::PtuSweep_GetResult_Response msg_;
};

class Init_PtuSweep_GetResult_Response_status
{
public:
  Init_PtuSweep_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PtuSweep_GetResult_Response_result status(::radar_msg::action::PtuSweep_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_PtuSweep_GetResult_Response_result(msg_);
  }

private:
  ::radar_msg::action::PtuSweep_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::radar_msg::action::PtuSweep_GetResult_Response>()
{
  return radar_msg::action::builder::Init_PtuSweep_GetResult_Response_status();
}

}  // namespace radar_msg


namespace radar_msg
{

namespace action
{

namespace builder
{

class Init_PtuSweep_FeedbackMessage_feedback
{
public:
  explicit Init_PtuSweep_FeedbackMessage_feedback(::radar_msg::action::PtuSweep_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::radar_msg::action::PtuSweep_FeedbackMessage feedback(::radar_msg::action::PtuSweep_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::radar_msg::action::PtuSweep_FeedbackMessage msg_;
};

class Init_PtuSweep_FeedbackMessage_goal_id
{
public:
  Init_PtuSweep_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PtuSweep_FeedbackMessage_feedback goal_id(::radar_msg::action::PtuSweep_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_PtuSweep_FeedbackMessage_feedback(msg_);
  }

private:
  ::radar_msg::action::PtuSweep_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::radar_msg::action::PtuSweep_FeedbackMessage>()
{
  return radar_msg::action::builder::Init_PtuSweep_FeedbackMessage_goal_id();
}

}  // namespace radar_msg

#endif  // RADAR_MSG__ACTION__DETAIL__PTU_SWEEP__BUILDER_HPP_

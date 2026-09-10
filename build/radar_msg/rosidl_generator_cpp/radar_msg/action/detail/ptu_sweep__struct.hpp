// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from radar_msg:action/PtuSweep.idl
// generated code does not contain a copyright notice

#ifndef RADAR_MSG__ACTION__DETAIL__PTU_SWEEP__STRUCT_HPP_
#define RADAR_MSG__ACTION__DETAIL__PTU_SWEEP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'target_ptu'
#include "radar_msg/msg/detail/ptu__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__radar_msg__action__PtuSweep_Goal __attribute__((deprecated))
#else
# define DEPRECATED__radar_msg__action__PtuSweep_Goal __declspec(deprecated)
#endif

namespace radar_msg
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PtuSweep_Goal_
{
  using Type = PtuSweep_Goal_<ContainerAllocator>;

  explicit PtuSweep_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target_ptu(_init)
  {
    (void)_init;
  }

  explicit PtuSweep_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target_ptu(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _target_ptu_type =
    radar_msg::msg::Ptu_<ContainerAllocator>;
  _target_ptu_type target_ptu;

  // setters for named parameter idiom
  Type & set__target_ptu(
    const radar_msg::msg::Ptu_<ContainerAllocator> & _arg)
  {
    this->target_ptu = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    radar_msg::action::PtuSweep_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const radar_msg::action::PtuSweep_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<radar_msg::action::PtuSweep_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<radar_msg::action::PtuSweep_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      radar_msg::action::PtuSweep_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<radar_msg::action::PtuSweep_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      radar_msg::action::PtuSweep_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<radar_msg::action::PtuSweep_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<radar_msg::action::PtuSweep_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<radar_msg::action::PtuSweep_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__radar_msg__action__PtuSweep_Goal
    std::shared_ptr<radar_msg::action::PtuSweep_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__radar_msg__action__PtuSweep_Goal
    std::shared_ptr<radar_msg::action::PtuSweep_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PtuSweep_Goal_ & other) const
  {
    if (this->target_ptu != other.target_ptu) {
      return false;
    }
    return true;
  }
  bool operator!=(const PtuSweep_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PtuSweep_Goal_

// alias to use template instance with default allocator
using PtuSweep_Goal =
  radar_msg::action::PtuSweep_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace radar_msg


#ifndef _WIN32
# define DEPRECATED__radar_msg__action__PtuSweep_Result __attribute__((deprecated))
#else
# define DEPRECATED__radar_msg__action__PtuSweep_Result __declspec(deprecated)
#endif

namespace radar_msg
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PtuSweep_Result_
{
  using Type = PtuSweep_Result_<ContainerAllocator>;

  explicit PtuSweep_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit PtuSweep_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    radar_msg::action::PtuSweep_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const radar_msg::action::PtuSweep_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<radar_msg::action::PtuSweep_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<radar_msg::action::PtuSweep_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      radar_msg::action::PtuSweep_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<radar_msg::action::PtuSweep_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      radar_msg::action::PtuSweep_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<radar_msg::action::PtuSweep_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<radar_msg::action::PtuSweep_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<radar_msg::action::PtuSweep_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__radar_msg__action__PtuSweep_Result
    std::shared_ptr<radar_msg::action::PtuSweep_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__radar_msg__action__PtuSweep_Result
    std::shared_ptr<radar_msg::action::PtuSweep_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PtuSweep_Result_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const PtuSweep_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PtuSweep_Result_

// alias to use template instance with default allocator
using PtuSweep_Result =
  radar_msg::action::PtuSweep_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace radar_msg


#ifndef _WIN32
# define DEPRECATED__radar_msg__action__PtuSweep_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__radar_msg__action__PtuSweep_Feedback __declspec(deprecated)
#endif

namespace radar_msg
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PtuSweep_Feedback_
{
  using Type = PtuSweep_Feedback_<ContainerAllocator>;

  explicit PtuSweep_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_pan_deg = 0l;
      this->current_tilt_deg = 0l;
      this->status = "";
    }
  }

  explicit PtuSweep_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : status(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_pan_deg = 0l;
      this->current_tilt_deg = 0l;
      this->status = "";
    }
  }

  // field types and members
  using _current_pan_deg_type =
    int32_t;
  _current_pan_deg_type current_pan_deg;
  using _current_tilt_deg_type =
    int32_t;
  _current_tilt_deg_type current_tilt_deg;
  using _status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _status_type status;

  // setters for named parameter idiom
  Type & set__current_pan_deg(
    const int32_t & _arg)
  {
    this->current_pan_deg = _arg;
    return *this;
  }
  Type & set__current_tilt_deg(
    const int32_t & _arg)
  {
    this->current_tilt_deg = _arg;
    return *this;
  }
  Type & set__status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    radar_msg::action::PtuSweep_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const radar_msg::action::PtuSweep_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<radar_msg::action::PtuSweep_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<radar_msg::action::PtuSweep_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      radar_msg::action::PtuSweep_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<radar_msg::action::PtuSweep_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      radar_msg::action::PtuSweep_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<radar_msg::action::PtuSweep_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<radar_msg::action::PtuSweep_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<radar_msg::action::PtuSweep_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__radar_msg__action__PtuSweep_Feedback
    std::shared_ptr<radar_msg::action::PtuSweep_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__radar_msg__action__PtuSweep_Feedback
    std::shared_ptr<radar_msg::action::PtuSweep_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PtuSweep_Feedback_ & other) const
  {
    if (this->current_pan_deg != other.current_pan_deg) {
      return false;
    }
    if (this->current_tilt_deg != other.current_tilt_deg) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const PtuSweep_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PtuSweep_Feedback_

// alias to use template instance with default allocator
using PtuSweep_Feedback =
  radar_msg::action::PtuSweep_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace radar_msg


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "radar_msg/action/detail/ptu_sweep__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__radar_msg__action__PtuSweep_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__radar_msg__action__PtuSweep_SendGoal_Request __declspec(deprecated)
#endif

namespace radar_msg
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PtuSweep_SendGoal_Request_
{
  using Type = PtuSweep_SendGoal_Request_<ContainerAllocator>;

  explicit PtuSweep_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit PtuSweep_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    radar_msg::action::PtuSweep_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const radar_msg::action::PtuSweep_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    radar_msg::action::PtuSweep_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const radar_msg::action::PtuSweep_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<radar_msg::action::PtuSweep_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<radar_msg::action::PtuSweep_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      radar_msg::action::PtuSweep_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<radar_msg::action::PtuSweep_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      radar_msg::action::PtuSweep_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<radar_msg::action::PtuSweep_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<radar_msg::action::PtuSweep_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<radar_msg::action::PtuSweep_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__radar_msg__action__PtuSweep_SendGoal_Request
    std::shared_ptr<radar_msg::action::PtuSweep_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__radar_msg__action__PtuSweep_SendGoal_Request
    std::shared_ptr<radar_msg::action::PtuSweep_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PtuSweep_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const PtuSweep_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PtuSweep_SendGoal_Request_

// alias to use template instance with default allocator
using PtuSweep_SendGoal_Request =
  radar_msg::action::PtuSweep_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace radar_msg


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__radar_msg__action__PtuSweep_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__radar_msg__action__PtuSweep_SendGoal_Response __declspec(deprecated)
#endif

namespace radar_msg
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PtuSweep_SendGoal_Response_
{
  using Type = PtuSweep_SendGoal_Response_<ContainerAllocator>;

  explicit PtuSweep_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit PtuSweep_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    radar_msg::action::PtuSweep_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const radar_msg::action::PtuSweep_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<radar_msg::action::PtuSweep_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<radar_msg::action::PtuSweep_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      radar_msg::action::PtuSweep_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<radar_msg::action::PtuSweep_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      radar_msg::action::PtuSweep_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<radar_msg::action::PtuSweep_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<radar_msg::action::PtuSweep_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<radar_msg::action::PtuSweep_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__radar_msg__action__PtuSweep_SendGoal_Response
    std::shared_ptr<radar_msg::action::PtuSweep_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__radar_msg__action__PtuSweep_SendGoal_Response
    std::shared_ptr<radar_msg::action::PtuSweep_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PtuSweep_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const PtuSweep_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PtuSweep_SendGoal_Response_

// alias to use template instance with default allocator
using PtuSweep_SendGoal_Response =
  radar_msg::action::PtuSweep_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace radar_msg

namespace radar_msg
{

namespace action
{

struct PtuSweep_SendGoal
{
  using Request = radar_msg::action::PtuSweep_SendGoal_Request;
  using Response = radar_msg::action::PtuSweep_SendGoal_Response;
};

}  // namespace action

}  // namespace radar_msg


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__radar_msg__action__PtuSweep_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__radar_msg__action__PtuSweep_GetResult_Request __declspec(deprecated)
#endif

namespace radar_msg
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PtuSweep_GetResult_Request_
{
  using Type = PtuSweep_GetResult_Request_<ContainerAllocator>;

  explicit PtuSweep_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit PtuSweep_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    radar_msg::action::PtuSweep_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const radar_msg::action::PtuSweep_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<radar_msg::action::PtuSweep_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<radar_msg::action::PtuSweep_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      radar_msg::action::PtuSweep_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<radar_msg::action::PtuSweep_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      radar_msg::action::PtuSweep_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<radar_msg::action::PtuSweep_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<radar_msg::action::PtuSweep_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<radar_msg::action::PtuSweep_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__radar_msg__action__PtuSweep_GetResult_Request
    std::shared_ptr<radar_msg::action::PtuSweep_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__radar_msg__action__PtuSweep_GetResult_Request
    std::shared_ptr<radar_msg::action::PtuSweep_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PtuSweep_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const PtuSweep_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PtuSweep_GetResult_Request_

// alias to use template instance with default allocator
using PtuSweep_GetResult_Request =
  radar_msg::action::PtuSweep_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace radar_msg


// Include directives for member types
// Member 'result'
// already included above
// #include "radar_msg/action/detail/ptu_sweep__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__radar_msg__action__PtuSweep_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__radar_msg__action__PtuSweep_GetResult_Response __declspec(deprecated)
#endif

namespace radar_msg
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PtuSweep_GetResult_Response_
{
  using Type = PtuSweep_GetResult_Response_<ContainerAllocator>;

  explicit PtuSweep_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit PtuSweep_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    radar_msg::action::PtuSweep_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const radar_msg::action::PtuSweep_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    radar_msg::action::PtuSweep_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const radar_msg::action::PtuSweep_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<radar_msg::action::PtuSweep_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<radar_msg::action::PtuSweep_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      radar_msg::action::PtuSweep_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<radar_msg::action::PtuSweep_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      radar_msg::action::PtuSweep_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<radar_msg::action::PtuSweep_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<radar_msg::action::PtuSweep_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<radar_msg::action::PtuSweep_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__radar_msg__action__PtuSweep_GetResult_Response
    std::shared_ptr<radar_msg::action::PtuSweep_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__radar_msg__action__PtuSweep_GetResult_Response
    std::shared_ptr<radar_msg::action::PtuSweep_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PtuSweep_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const PtuSweep_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PtuSweep_GetResult_Response_

// alias to use template instance with default allocator
using PtuSweep_GetResult_Response =
  radar_msg::action::PtuSweep_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace radar_msg

namespace radar_msg
{

namespace action
{

struct PtuSweep_GetResult
{
  using Request = radar_msg::action::PtuSweep_GetResult_Request;
  using Response = radar_msg::action::PtuSweep_GetResult_Response;
};

}  // namespace action

}  // namespace radar_msg


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "radar_msg/action/detail/ptu_sweep__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__radar_msg__action__PtuSweep_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__radar_msg__action__PtuSweep_FeedbackMessage __declspec(deprecated)
#endif

namespace radar_msg
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PtuSweep_FeedbackMessage_
{
  using Type = PtuSweep_FeedbackMessage_<ContainerAllocator>;

  explicit PtuSweep_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit PtuSweep_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    radar_msg::action::PtuSweep_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const radar_msg::action::PtuSweep_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    radar_msg::action::PtuSweep_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const radar_msg::action::PtuSweep_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<radar_msg::action::PtuSweep_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<radar_msg::action::PtuSweep_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      radar_msg::action::PtuSweep_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<radar_msg::action::PtuSweep_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      radar_msg::action::PtuSweep_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<radar_msg::action::PtuSweep_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<radar_msg::action::PtuSweep_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<radar_msg::action::PtuSweep_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__radar_msg__action__PtuSweep_FeedbackMessage
    std::shared_ptr<radar_msg::action::PtuSweep_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__radar_msg__action__PtuSweep_FeedbackMessage
    std::shared_ptr<radar_msg::action::PtuSweep_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PtuSweep_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const PtuSweep_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PtuSweep_FeedbackMessage_

// alias to use template instance with default allocator
using PtuSweep_FeedbackMessage =
  radar_msg::action::PtuSweep_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace radar_msg

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace radar_msg
{

namespace action
{

struct PtuSweep
{
  /// The goal message defined in the action definition.
  using Goal = radar_msg::action::PtuSweep_Goal;
  /// The result message defined in the action definition.
  using Result = radar_msg::action::PtuSweep_Result;
  /// The feedback message defined in the action definition.
  using Feedback = radar_msg::action::PtuSweep_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = radar_msg::action::PtuSweep_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = radar_msg::action::PtuSweep_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = radar_msg::action::PtuSweep_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct PtuSweep PtuSweep;

}  // namespace action

}  // namespace radar_msg

#endif  // RADAR_MSG__ACTION__DETAIL__PTU_SWEEP__STRUCT_HPP_

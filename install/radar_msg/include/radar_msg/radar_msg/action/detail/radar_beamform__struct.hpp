// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from radar_msg:action/RadarBeamform.idl
// generated code does not contain a copyright notice

#ifndef RADAR_MSG__ACTION__DETAIL__RADAR_BEAMFORM__STRUCT_HPP_
#define RADAR_MSG__ACTION__DETAIL__RADAR_BEAMFORM__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__radar_msg__action__RadarBeamform_Goal __attribute__((deprecated))
#else
# define DEPRECATED__radar_msg__action__RadarBeamform_Goal __declspec(deprecated)
#endif

namespace radar_msg
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct RadarBeamform_Goal_
{
  using Type = RadarBeamform_Goal_<ContainerAllocator>;

  explicit RadarBeamform_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit RadarBeamform_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    radar_msg::action::RadarBeamform_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const radar_msg::action::RadarBeamform_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<radar_msg::action::RadarBeamform_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<radar_msg::action::RadarBeamform_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      radar_msg::action::RadarBeamform_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<radar_msg::action::RadarBeamform_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      radar_msg::action::RadarBeamform_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<radar_msg::action::RadarBeamform_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<radar_msg::action::RadarBeamform_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<radar_msg::action::RadarBeamform_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__radar_msg__action__RadarBeamform_Goal
    std::shared_ptr<radar_msg::action::RadarBeamform_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__radar_msg__action__RadarBeamform_Goal
    std::shared_ptr<radar_msg::action::RadarBeamform_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RadarBeamform_Goal_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const RadarBeamform_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RadarBeamform_Goal_

// alias to use template instance with default allocator
using RadarBeamform_Goal =
  radar_msg::action::RadarBeamform_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace radar_msg


// Include directives for member types
// Member 'radar_data'
#include "radar_msg/msg/detail/radar_data__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__radar_msg__action__RadarBeamform_Result __attribute__((deprecated))
#else
# define DEPRECATED__radar_msg__action__RadarBeamform_Result __declspec(deprecated)
#endif

namespace radar_msg
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct RadarBeamform_Result_
{
  using Type = RadarBeamform_Result_<ContainerAllocator>;

  explicit RadarBeamform_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : radar_data(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit RadarBeamform_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc),
    radar_data(_alloc, _init)
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
  using _radar_data_type =
    radar_msg::msg::RadarData_<ContainerAllocator>;
  _radar_data_type radar_data;

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
  Type & set__radar_data(
    const radar_msg::msg::RadarData_<ContainerAllocator> & _arg)
  {
    this->radar_data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    radar_msg::action::RadarBeamform_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const radar_msg::action::RadarBeamform_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<radar_msg::action::RadarBeamform_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<radar_msg::action::RadarBeamform_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      radar_msg::action::RadarBeamform_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<radar_msg::action::RadarBeamform_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      radar_msg::action::RadarBeamform_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<radar_msg::action::RadarBeamform_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<radar_msg::action::RadarBeamform_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<radar_msg::action::RadarBeamform_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__radar_msg__action__RadarBeamform_Result
    std::shared_ptr<radar_msg::action::RadarBeamform_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__radar_msg__action__RadarBeamform_Result
    std::shared_ptr<radar_msg::action::RadarBeamform_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RadarBeamform_Result_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->radar_data != other.radar_data) {
      return false;
    }
    return true;
  }
  bool operator!=(const RadarBeamform_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RadarBeamform_Result_

// alias to use template instance with default allocator
using RadarBeamform_Result =
  radar_msg::action::RadarBeamform_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace radar_msg


#ifndef _WIN32
# define DEPRECATED__radar_msg__action__RadarBeamform_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__radar_msg__action__RadarBeamform_Feedback __declspec(deprecated)
#endif

namespace radar_msg
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct RadarBeamform_Feedback_
{
  using Type = RadarBeamform_Feedback_<ContainerAllocator>;

  explicit RadarBeamform_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = "";
    }
  }

  explicit RadarBeamform_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : status(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = "";
    }
  }

  // field types and members
  using _status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _status_type status;

  // setters for named parameter idiom
  Type & set__status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    radar_msg::action::RadarBeamform_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const radar_msg::action::RadarBeamform_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<radar_msg::action::RadarBeamform_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<radar_msg::action::RadarBeamform_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      radar_msg::action::RadarBeamform_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<radar_msg::action::RadarBeamform_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      radar_msg::action::RadarBeamform_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<radar_msg::action::RadarBeamform_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<radar_msg::action::RadarBeamform_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<radar_msg::action::RadarBeamform_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__radar_msg__action__RadarBeamform_Feedback
    std::shared_ptr<radar_msg::action::RadarBeamform_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__radar_msg__action__RadarBeamform_Feedback
    std::shared_ptr<radar_msg::action::RadarBeamform_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RadarBeamform_Feedback_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const RadarBeamform_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RadarBeamform_Feedback_

// alias to use template instance with default allocator
using RadarBeamform_Feedback =
  radar_msg::action::RadarBeamform_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace radar_msg


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "radar_msg/action/detail/radar_beamform__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__radar_msg__action__RadarBeamform_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__radar_msg__action__RadarBeamform_SendGoal_Request __declspec(deprecated)
#endif

namespace radar_msg
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct RadarBeamform_SendGoal_Request_
{
  using Type = RadarBeamform_SendGoal_Request_<ContainerAllocator>;

  explicit RadarBeamform_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit RadarBeamform_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    radar_msg::action::RadarBeamform_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const radar_msg::action::RadarBeamform_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    radar_msg::action::RadarBeamform_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const radar_msg::action::RadarBeamform_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<radar_msg::action::RadarBeamform_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<radar_msg::action::RadarBeamform_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      radar_msg::action::RadarBeamform_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<radar_msg::action::RadarBeamform_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      radar_msg::action::RadarBeamform_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<radar_msg::action::RadarBeamform_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<radar_msg::action::RadarBeamform_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<radar_msg::action::RadarBeamform_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__radar_msg__action__RadarBeamform_SendGoal_Request
    std::shared_ptr<radar_msg::action::RadarBeamform_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__radar_msg__action__RadarBeamform_SendGoal_Request
    std::shared_ptr<radar_msg::action::RadarBeamform_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RadarBeamform_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const RadarBeamform_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RadarBeamform_SendGoal_Request_

// alias to use template instance with default allocator
using RadarBeamform_SendGoal_Request =
  radar_msg::action::RadarBeamform_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace radar_msg


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__radar_msg__action__RadarBeamform_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__radar_msg__action__RadarBeamform_SendGoal_Response __declspec(deprecated)
#endif

namespace radar_msg
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct RadarBeamform_SendGoal_Response_
{
  using Type = RadarBeamform_SendGoal_Response_<ContainerAllocator>;

  explicit RadarBeamform_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit RadarBeamform_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    radar_msg::action::RadarBeamform_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const radar_msg::action::RadarBeamform_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<radar_msg::action::RadarBeamform_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<radar_msg::action::RadarBeamform_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      radar_msg::action::RadarBeamform_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<radar_msg::action::RadarBeamform_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      radar_msg::action::RadarBeamform_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<radar_msg::action::RadarBeamform_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<radar_msg::action::RadarBeamform_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<radar_msg::action::RadarBeamform_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__radar_msg__action__RadarBeamform_SendGoal_Response
    std::shared_ptr<radar_msg::action::RadarBeamform_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__radar_msg__action__RadarBeamform_SendGoal_Response
    std::shared_ptr<radar_msg::action::RadarBeamform_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RadarBeamform_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const RadarBeamform_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RadarBeamform_SendGoal_Response_

// alias to use template instance with default allocator
using RadarBeamform_SendGoal_Response =
  radar_msg::action::RadarBeamform_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace radar_msg

namespace radar_msg
{

namespace action
{

struct RadarBeamform_SendGoal
{
  using Request = radar_msg::action::RadarBeamform_SendGoal_Request;
  using Response = radar_msg::action::RadarBeamform_SendGoal_Response;
};

}  // namespace action

}  // namespace radar_msg


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__radar_msg__action__RadarBeamform_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__radar_msg__action__RadarBeamform_GetResult_Request __declspec(deprecated)
#endif

namespace radar_msg
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct RadarBeamform_GetResult_Request_
{
  using Type = RadarBeamform_GetResult_Request_<ContainerAllocator>;

  explicit RadarBeamform_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit RadarBeamform_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    radar_msg::action::RadarBeamform_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const radar_msg::action::RadarBeamform_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<radar_msg::action::RadarBeamform_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<radar_msg::action::RadarBeamform_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      radar_msg::action::RadarBeamform_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<radar_msg::action::RadarBeamform_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      radar_msg::action::RadarBeamform_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<radar_msg::action::RadarBeamform_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<radar_msg::action::RadarBeamform_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<radar_msg::action::RadarBeamform_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__radar_msg__action__RadarBeamform_GetResult_Request
    std::shared_ptr<radar_msg::action::RadarBeamform_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__radar_msg__action__RadarBeamform_GetResult_Request
    std::shared_ptr<radar_msg::action::RadarBeamform_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RadarBeamform_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const RadarBeamform_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RadarBeamform_GetResult_Request_

// alias to use template instance with default allocator
using RadarBeamform_GetResult_Request =
  radar_msg::action::RadarBeamform_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace radar_msg


// Include directives for member types
// Member 'result'
// already included above
// #include "radar_msg/action/detail/radar_beamform__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__radar_msg__action__RadarBeamform_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__radar_msg__action__RadarBeamform_GetResult_Response __declspec(deprecated)
#endif

namespace radar_msg
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct RadarBeamform_GetResult_Response_
{
  using Type = RadarBeamform_GetResult_Response_<ContainerAllocator>;

  explicit RadarBeamform_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit RadarBeamform_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    radar_msg::action::RadarBeamform_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const radar_msg::action::RadarBeamform_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    radar_msg::action::RadarBeamform_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const radar_msg::action::RadarBeamform_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<radar_msg::action::RadarBeamform_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<radar_msg::action::RadarBeamform_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      radar_msg::action::RadarBeamform_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<radar_msg::action::RadarBeamform_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      radar_msg::action::RadarBeamform_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<radar_msg::action::RadarBeamform_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<radar_msg::action::RadarBeamform_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<radar_msg::action::RadarBeamform_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__radar_msg__action__RadarBeamform_GetResult_Response
    std::shared_ptr<radar_msg::action::RadarBeamform_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__radar_msg__action__RadarBeamform_GetResult_Response
    std::shared_ptr<radar_msg::action::RadarBeamform_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RadarBeamform_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const RadarBeamform_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RadarBeamform_GetResult_Response_

// alias to use template instance with default allocator
using RadarBeamform_GetResult_Response =
  radar_msg::action::RadarBeamform_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace radar_msg

namespace radar_msg
{

namespace action
{

struct RadarBeamform_GetResult
{
  using Request = radar_msg::action::RadarBeamform_GetResult_Request;
  using Response = radar_msg::action::RadarBeamform_GetResult_Response;
};

}  // namespace action

}  // namespace radar_msg


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "radar_msg/action/detail/radar_beamform__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__radar_msg__action__RadarBeamform_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__radar_msg__action__RadarBeamform_FeedbackMessage __declspec(deprecated)
#endif

namespace radar_msg
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct RadarBeamform_FeedbackMessage_
{
  using Type = RadarBeamform_FeedbackMessage_<ContainerAllocator>;

  explicit RadarBeamform_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit RadarBeamform_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    radar_msg::action::RadarBeamform_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const radar_msg::action::RadarBeamform_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    radar_msg::action::RadarBeamform_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const radar_msg::action::RadarBeamform_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<radar_msg::action::RadarBeamform_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<radar_msg::action::RadarBeamform_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      radar_msg::action::RadarBeamform_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<radar_msg::action::RadarBeamform_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      radar_msg::action::RadarBeamform_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<radar_msg::action::RadarBeamform_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<radar_msg::action::RadarBeamform_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<radar_msg::action::RadarBeamform_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__radar_msg__action__RadarBeamform_FeedbackMessage
    std::shared_ptr<radar_msg::action::RadarBeamform_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__radar_msg__action__RadarBeamform_FeedbackMessage
    std::shared_ptr<radar_msg::action::RadarBeamform_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RadarBeamform_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const RadarBeamform_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RadarBeamform_FeedbackMessage_

// alias to use template instance with default allocator
using RadarBeamform_FeedbackMessage =
  radar_msg::action::RadarBeamform_FeedbackMessage_<std::allocator<void>>;

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

struct RadarBeamform
{
  /// The goal message defined in the action definition.
  using Goal = radar_msg::action::RadarBeamform_Goal;
  /// The result message defined in the action definition.
  using Result = radar_msg::action::RadarBeamform_Result;
  /// The feedback message defined in the action definition.
  using Feedback = radar_msg::action::RadarBeamform_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = radar_msg::action::RadarBeamform_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = radar_msg::action::RadarBeamform_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = radar_msg::action::RadarBeamform_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct RadarBeamform RadarBeamform;

}  // namespace action

}  // namespace radar_msg

#endif  // RADAR_MSG__ACTION__DETAIL__RADAR_BEAMFORM__STRUCT_HPP_

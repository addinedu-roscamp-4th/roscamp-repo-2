// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mycobot_interfaces:srv/GripperStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mycobot_interfaces/srv/gripper_status.hpp"


#ifndef MYCOBOT_INTERFACES__SRV__DETAIL__GRIPPER_STATUS__BUILDER_HPP_
#define MYCOBOT_INTERFACES__SRV__DETAIL__GRIPPER_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mycobot_interfaces/srv/detail/gripper_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mycobot_interfaces
{

namespace srv
{

namespace builder
{

class Init_GripperStatus_Request_status
{
public:
  Init_GripperStatus_Request_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mycobot_interfaces::srv::GripperStatus_Request status(::mycobot_interfaces::srv::GripperStatus_Request::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mycobot_interfaces::srv::GripperStatus_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mycobot_interfaces::srv::GripperStatus_Request>()
{
  return mycobot_interfaces::srv::builder::Init_GripperStatus_Request_status();
}

}  // namespace mycobot_interfaces


namespace mycobot_interfaces
{

namespace srv
{

namespace builder
{

class Init_GripperStatus_Response_flag
{
public:
  Init_GripperStatus_Response_flag()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mycobot_interfaces::srv::GripperStatus_Response flag(::mycobot_interfaces::srv::GripperStatus_Response::_flag_type arg)
  {
    msg_.flag = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mycobot_interfaces::srv::GripperStatus_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mycobot_interfaces::srv::GripperStatus_Response>()
{
  return mycobot_interfaces::srv::builder::Init_GripperStatus_Response_flag();
}

}  // namespace mycobot_interfaces


namespace mycobot_interfaces
{

namespace srv
{

namespace builder
{

class Init_GripperStatus_Event_response
{
public:
  explicit Init_GripperStatus_Event_response(::mycobot_interfaces::srv::GripperStatus_Event & msg)
  : msg_(msg)
  {}
  ::mycobot_interfaces::srv::GripperStatus_Event response(::mycobot_interfaces::srv::GripperStatus_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mycobot_interfaces::srv::GripperStatus_Event msg_;
};

class Init_GripperStatus_Event_request
{
public:
  explicit Init_GripperStatus_Event_request(::mycobot_interfaces::srv::GripperStatus_Event & msg)
  : msg_(msg)
  {}
  Init_GripperStatus_Event_response request(::mycobot_interfaces::srv::GripperStatus_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GripperStatus_Event_response(msg_);
  }

private:
  ::mycobot_interfaces::srv::GripperStatus_Event msg_;
};

class Init_GripperStatus_Event_info
{
public:
  Init_GripperStatus_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GripperStatus_Event_request info(::mycobot_interfaces::srv::GripperStatus_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GripperStatus_Event_request(msg_);
  }

private:
  ::mycobot_interfaces::srv::GripperStatus_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mycobot_interfaces::srv::GripperStatus_Event>()
{
  return mycobot_interfaces::srv::builder::Init_GripperStatus_Event_info();
}

}  // namespace mycobot_interfaces

#endif  // MYCOBOT_INTERFACES__SRV__DETAIL__GRIPPER_STATUS__BUILDER_HPP_

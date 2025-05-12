// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mycobot_interfaces:srv/PumpStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mycobot_interfaces/srv/pump_status.hpp"


#ifndef MYCOBOT_INTERFACES__SRV__DETAIL__PUMP_STATUS__BUILDER_HPP_
#define MYCOBOT_INTERFACES__SRV__DETAIL__PUMP_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mycobot_interfaces/srv/detail/pump_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mycobot_interfaces
{

namespace srv
{

namespace builder
{

class Init_PumpStatus_Request_pin2
{
public:
  explicit Init_PumpStatus_Request_pin2(::mycobot_interfaces::srv::PumpStatus_Request & msg)
  : msg_(msg)
  {}
  ::mycobot_interfaces::srv::PumpStatus_Request pin2(::mycobot_interfaces::srv::PumpStatus_Request::_pin2_type arg)
  {
    msg_.pin2 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mycobot_interfaces::srv::PumpStatus_Request msg_;
};

class Init_PumpStatus_Request_pin1
{
public:
  explicit Init_PumpStatus_Request_pin1(::mycobot_interfaces::srv::PumpStatus_Request & msg)
  : msg_(msg)
  {}
  Init_PumpStatus_Request_pin2 pin1(::mycobot_interfaces::srv::PumpStatus_Request::_pin1_type arg)
  {
    msg_.pin1 = std::move(arg);
    return Init_PumpStatus_Request_pin2(msg_);
  }

private:
  ::mycobot_interfaces::srv::PumpStatus_Request msg_;
};

class Init_PumpStatus_Request_status
{
public:
  Init_PumpStatus_Request_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PumpStatus_Request_pin1 status(::mycobot_interfaces::srv::PumpStatus_Request::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_PumpStatus_Request_pin1(msg_);
  }

private:
  ::mycobot_interfaces::srv::PumpStatus_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mycobot_interfaces::srv::PumpStatus_Request>()
{
  return mycobot_interfaces::srv::builder::Init_PumpStatus_Request_status();
}

}  // namespace mycobot_interfaces


namespace mycobot_interfaces
{

namespace srv
{

namespace builder
{

class Init_PumpStatus_Response_flag
{
public:
  Init_PumpStatus_Response_flag()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mycobot_interfaces::srv::PumpStatus_Response flag(::mycobot_interfaces::srv::PumpStatus_Response::_flag_type arg)
  {
    msg_.flag = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mycobot_interfaces::srv::PumpStatus_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mycobot_interfaces::srv::PumpStatus_Response>()
{
  return mycobot_interfaces::srv::builder::Init_PumpStatus_Response_flag();
}

}  // namespace mycobot_interfaces


namespace mycobot_interfaces
{

namespace srv
{

namespace builder
{

class Init_PumpStatus_Event_response
{
public:
  explicit Init_PumpStatus_Event_response(::mycobot_interfaces::srv::PumpStatus_Event & msg)
  : msg_(msg)
  {}
  ::mycobot_interfaces::srv::PumpStatus_Event response(::mycobot_interfaces::srv::PumpStatus_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mycobot_interfaces::srv::PumpStatus_Event msg_;
};

class Init_PumpStatus_Event_request
{
public:
  explicit Init_PumpStatus_Event_request(::mycobot_interfaces::srv::PumpStatus_Event & msg)
  : msg_(msg)
  {}
  Init_PumpStatus_Event_response request(::mycobot_interfaces::srv::PumpStatus_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_PumpStatus_Event_response(msg_);
  }

private:
  ::mycobot_interfaces::srv::PumpStatus_Event msg_;
};

class Init_PumpStatus_Event_info
{
public:
  Init_PumpStatus_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PumpStatus_Event_request info(::mycobot_interfaces::srv::PumpStatus_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_PumpStatus_Event_request(msg_);
  }

private:
  ::mycobot_interfaces::srv::PumpStatus_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mycobot_interfaces::srv::PumpStatus_Event>()
{
  return mycobot_interfaces::srv::builder::Init_PumpStatus_Event_info();
}

}  // namespace mycobot_interfaces

#endif  // MYCOBOT_INTERFACES__SRV__DETAIL__PUMP_STATUS__BUILDER_HPP_

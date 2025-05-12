// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mycobot_interfaces:srv/SetAngles.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mycobot_interfaces/srv/set_angles.hpp"


#ifndef MYCOBOT_INTERFACES__SRV__DETAIL__SET_ANGLES__BUILDER_HPP_
#define MYCOBOT_INTERFACES__SRV__DETAIL__SET_ANGLES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mycobot_interfaces/srv/detail/set_angles__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mycobot_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetAngles_Request_speed
{
public:
  explicit Init_SetAngles_Request_speed(::mycobot_interfaces::srv::SetAngles_Request & msg)
  : msg_(msg)
  {}
  ::mycobot_interfaces::srv::SetAngles_Request speed(::mycobot_interfaces::srv::SetAngles_Request::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mycobot_interfaces::srv::SetAngles_Request msg_;
};

class Init_SetAngles_Request_joint_6
{
public:
  explicit Init_SetAngles_Request_joint_6(::mycobot_interfaces::srv::SetAngles_Request & msg)
  : msg_(msg)
  {}
  Init_SetAngles_Request_speed joint_6(::mycobot_interfaces::srv::SetAngles_Request::_joint_6_type arg)
  {
    msg_.joint_6 = std::move(arg);
    return Init_SetAngles_Request_speed(msg_);
  }

private:
  ::mycobot_interfaces::srv::SetAngles_Request msg_;
};

class Init_SetAngles_Request_joint_5
{
public:
  explicit Init_SetAngles_Request_joint_5(::mycobot_interfaces::srv::SetAngles_Request & msg)
  : msg_(msg)
  {}
  Init_SetAngles_Request_joint_6 joint_5(::mycobot_interfaces::srv::SetAngles_Request::_joint_5_type arg)
  {
    msg_.joint_5 = std::move(arg);
    return Init_SetAngles_Request_joint_6(msg_);
  }

private:
  ::mycobot_interfaces::srv::SetAngles_Request msg_;
};

class Init_SetAngles_Request_joint_4
{
public:
  explicit Init_SetAngles_Request_joint_4(::mycobot_interfaces::srv::SetAngles_Request & msg)
  : msg_(msg)
  {}
  Init_SetAngles_Request_joint_5 joint_4(::mycobot_interfaces::srv::SetAngles_Request::_joint_4_type arg)
  {
    msg_.joint_4 = std::move(arg);
    return Init_SetAngles_Request_joint_5(msg_);
  }

private:
  ::mycobot_interfaces::srv::SetAngles_Request msg_;
};

class Init_SetAngles_Request_joint_3
{
public:
  explicit Init_SetAngles_Request_joint_3(::mycobot_interfaces::srv::SetAngles_Request & msg)
  : msg_(msg)
  {}
  Init_SetAngles_Request_joint_4 joint_3(::mycobot_interfaces::srv::SetAngles_Request::_joint_3_type arg)
  {
    msg_.joint_3 = std::move(arg);
    return Init_SetAngles_Request_joint_4(msg_);
  }

private:
  ::mycobot_interfaces::srv::SetAngles_Request msg_;
};

class Init_SetAngles_Request_joint_2
{
public:
  explicit Init_SetAngles_Request_joint_2(::mycobot_interfaces::srv::SetAngles_Request & msg)
  : msg_(msg)
  {}
  Init_SetAngles_Request_joint_3 joint_2(::mycobot_interfaces::srv::SetAngles_Request::_joint_2_type arg)
  {
    msg_.joint_2 = std::move(arg);
    return Init_SetAngles_Request_joint_3(msg_);
  }

private:
  ::mycobot_interfaces::srv::SetAngles_Request msg_;
};

class Init_SetAngles_Request_joint_1
{
public:
  Init_SetAngles_Request_joint_1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetAngles_Request_joint_2 joint_1(::mycobot_interfaces::srv::SetAngles_Request::_joint_1_type arg)
  {
    msg_.joint_1 = std::move(arg);
    return Init_SetAngles_Request_joint_2(msg_);
  }

private:
  ::mycobot_interfaces::srv::SetAngles_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mycobot_interfaces::srv::SetAngles_Request>()
{
  return mycobot_interfaces::srv::builder::Init_SetAngles_Request_joint_1();
}

}  // namespace mycobot_interfaces


namespace mycobot_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetAngles_Response_flag
{
public:
  Init_SetAngles_Response_flag()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mycobot_interfaces::srv::SetAngles_Response flag(::mycobot_interfaces::srv::SetAngles_Response::_flag_type arg)
  {
    msg_.flag = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mycobot_interfaces::srv::SetAngles_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mycobot_interfaces::srv::SetAngles_Response>()
{
  return mycobot_interfaces::srv::builder::Init_SetAngles_Response_flag();
}

}  // namespace mycobot_interfaces


namespace mycobot_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetAngles_Event_response
{
public:
  explicit Init_SetAngles_Event_response(::mycobot_interfaces::srv::SetAngles_Event & msg)
  : msg_(msg)
  {}
  ::mycobot_interfaces::srv::SetAngles_Event response(::mycobot_interfaces::srv::SetAngles_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mycobot_interfaces::srv::SetAngles_Event msg_;
};

class Init_SetAngles_Event_request
{
public:
  explicit Init_SetAngles_Event_request(::mycobot_interfaces::srv::SetAngles_Event & msg)
  : msg_(msg)
  {}
  Init_SetAngles_Event_response request(::mycobot_interfaces::srv::SetAngles_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SetAngles_Event_response(msg_);
  }

private:
  ::mycobot_interfaces::srv::SetAngles_Event msg_;
};

class Init_SetAngles_Event_info
{
public:
  Init_SetAngles_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetAngles_Event_request info(::mycobot_interfaces::srv::SetAngles_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SetAngles_Event_request(msg_);
  }

private:
  ::mycobot_interfaces::srv::SetAngles_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mycobot_interfaces::srv::SetAngles_Event>()
{
  return mycobot_interfaces::srv::builder::Init_SetAngles_Event_info();
}

}  // namespace mycobot_interfaces

#endif  // MYCOBOT_INTERFACES__SRV__DETAIL__SET_ANGLES__BUILDER_HPP_

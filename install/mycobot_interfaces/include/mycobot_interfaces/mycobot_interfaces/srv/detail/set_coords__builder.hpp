// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mycobot_interfaces:srv/SetCoords.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mycobot_interfaces/srv/set_coords.hpp"


#ifndef MYCOBOT_INTERFACES__SRV__DETAIL__SET_COORDS__BUILDER_HPP_
#define MYCOBOT_INTERFACES__SRV__DETAIL__SET_COORDS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mycobot_interfaces/srv/detail/set_coords__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mycobot_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetCoords_Request_model
{
public:
  explicit Init_SetCoords_Request_model(::mycobot_interfaces::srv::SetCoords_Request & msg)
  : msg_(msg)
  {}
  ::mycobot_interfaces::srv::SetCoords_Request model(::mycobot_interfaces::srv::SetCoords_Request::_model_type arg)
  {
    msg_.model = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mycobot_interfaces::srv::SetCoords_Request msg_;
};

class Init_SetCoords_Request_speed
{
public:
  explicit Init_SetCoords_Request_speed(::mycobot_interfaces::srv::SetCoords_Request & msg)
  : msg_(msg)
  {}
  Init_SetCoords_Request_model speed(::mycobot_interfaces::srv::SetCoords_Request::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_SetCoords_Request_model(msg_);
  }

private:
  ::mycobot_interfaces::srv::SetCoords_Request msg_;
};

class Init_SetCoords_Request_rz
{
public:
  explicit Init_SetCoords_Request_rz(::mycobot_interfaces::srv::SetCoords_Request & msg)
  : msg_(msg)
  {}
  Init_SetCoords_Request_speed rz(::mycobot_interfaces::srv::SetCoords_Request::_rz_type arg)
  {
    msg_.rz = std::move(arg);
    return Init_SetCoords_Request_speed(msg_);
  }

private:
  ::mycobot_interfaces::srv::SetCoords_Request msg_;
};

class Init_SetCoords_Request_ry
{
public:
  explicit Init_SetCoords_Request_ry(::mycobot_interfaces::srv::SetCoords_Request & msg)
  : msg_(msg)
  {}
  Init_SetCoords_Request_rz ry(::mycobot_interfaces::srv::SetCoords_Request::_ry_type arg)
  {
    msg_.ry = std::move(arg);
    return Init_SetCoords_Request_rz(msg_);
  }

private:
  ::mycobot_interfaces::srv::SetCoords_Request msg_;
};

class Init_SetCoords_Request_rx
{
public:
  explicit Init_SetCoords_Request_rx(::mycobot_interfaces::srv::SetCoords_Request & msg)
  : msg_(msg)
  {}
  Init_SetCoords_Request_ry rx(::mycobot_interfaces::srv::SetCoords_Request::_rx_type arg)
  {
    msg_.rx = std::move(arg);
    return Init_SetCoords_Request_ry(msg_);
  }

private:
  ::mycobot_interfaces::srv::SetCoords_Request msg_;
};

class Init_SetCoords_Request_z
{
public:
  explicit Init_SetCoords_Request_z(::mycobot_interfaces::srv::SetCoords_Request & msg)
  : msg_(msg)
  {}
  Init_SetCoords_Request_rx z(::mycobot_interfaces::srv::SetCoords_Request::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_SetCoords_Request_rx(msg_);
  }

private:
  ::mycobot_interfaces::srv::SetCoords_Request msg_;
};

class Init_SetCoords_Request_y
{
public:
  explicit Init_SetCoords_Request_y(::mycobot_interfaces::srv::SetCoords_Request & msg)
  : msg_(msg)
  {}
  Init_SetCoords_Request_z y(::mycobot_interfaces::srv::SetCoords_Request::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_SetCoords_Request_z(msg_);
  }

private:
  ::mycobot_interfaces::srv::SetCoords_Request msg_;
};

class Init_SetCoords_Request_x
{
public:
  Init_SetCoords_Request_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetCoords_Request_y x(::mycobot_interfaces::srv::SetCoords_Request::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_SetCoords_Request_y(msg_);
  }

private:
  ::mycobot_interfaces::srv::SetCoords_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mycobot_interfaces::srv::SetCoords_Request>()
{
  return mycobot_interfaces::srv::builder::Init_SetCoords_Request_x();
}

}  // namespace mycobot_interfaces


namespace mycobot_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetCoords_Response_flag
{
public:
  Init_SetCoords_Response_flag()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mycobot_interfaces::srv::SetCoords_Response flag(::mycobot_interfaces::srv::SetCoords_Response::_flag_type arg)
  {
    msg_.flag = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mycobot_interfaces::srv::SetCoords_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mycobot_interfaces::srv::SetCoords_Response>()
{
  return mycobot_interfaces::srv::builder::Init_SetCoords_Response_flag();
}

}  // namespace mycobot_interfaces


namespace mycobot_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetCoords_Event_response
{
public:
  explicit Init_SetCoords_Event_response(::mycobot_interfaces::srv::SetCoords_Event & msg)
  : msg_(msg)
  {}
  ::mycobot_interfaces::srv::SetCoords_Event response(::mycobot_interfaces::srv::SetCoords_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mycobot_interfaces::srv::SetCoords_Event msg_;
};

class Init_SetCoords_Event_request
{
public:
  explicit Init_SetCoords_Event_request(::mycobot_interfaces::srv::SetCoords_Event & msg)
  : msg_(msg)
  {}
  Init_SetCoords_Event_response request(::mycobot_interfaces::srv::SetCoords_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SetCoords_Event_response(msg_);
  }

private:
  ::mycobot_interfaces::srv::SetCoords_Event msg_;
};

class Init_SetCoords_Event_info
{
public:
  Init_SetCoords_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetCoords_Event_request info(::mycobot_interfaces::srv::SetCoords_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SetCoords_Event_request(msg_);
  }

private:
  ::mycobot_interfaces::srv::SetCoords_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mycobot_interfaces::srv::SetCoords_Event>()
{
  return mycobot_interfaces::srv::builder::Init_SetCoords_Event_info();
}

}  // namespace mycobot_interfaces

#endif  // MYCOBOT_INTERFACES__SRV__DETAIL__SET_COORDS__BUILDER_HPP_

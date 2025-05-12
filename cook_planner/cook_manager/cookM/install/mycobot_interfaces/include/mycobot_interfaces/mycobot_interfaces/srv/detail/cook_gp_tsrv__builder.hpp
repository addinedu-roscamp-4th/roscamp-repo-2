// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mycobot_interfaces:srv/CookGPTsrv.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mycobot_interfaces/srv/cook_gp_tsrv.hpp"


#ifndef MYCOBOT_INTERFACES__SRV__DETAIL__COOK_GP_TSRV__BUILDER_HPP_
#define MYCOBOT_INTERFACES__SRV__DETAIL__COOK_GP_TSRV__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mycobot_interfaces/srv/detail/cook_gp_tsrv__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mycobot_interfaces
{

namespace srv
{

namespace builder
{

class Init_CookGPTsrv_Request_robot_id
{
public:
  explicit Init_CookGPTsrv_Request_robot_id(::mycobot_interfaces::srv::CookGPTsrv_Request & msg)
  : msg_(msg)
  {}
  ::mycobot_interfaces::srv::CookGPTsrv_Request robot_id(::mycobot_interfaces::srv::CookGPTsrv_Request::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mycobot_interfaces::srv::CookGPTsrv_Request msg_;
};

class Init_CookGPTsrv_Request_command
{
public:
  Init_CookGPTsrv_Request_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CookGPTsrv_Request_robot_id command(::mycobot_interfaces::srv::CookGPTsrv_Request::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_CookGPTsrv_Request_robot_id(msg_);
  }

private:
  ::mycobot_interfaces::srv::CookGPTsrv_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mycobot_interfaces::srv::CookGPTsrv_Request>()
{
  return mycobot_interfaces::srv::builder::Init_CookGPTsrv_Request_command();
}

}  // namespace mycobot_interfaces


namespace mycobot_interfaces
{

namespace srv
{

namespace builder
{

class Init_CookGPTsrv_Response_stain
{
public:
  explicit Init_CookGPTsrv_Response_stain(::mycobot_interfaces::srv::CookGPTsrv_Response & msg)
  : msg_(msg)
  {}
  ::mycobot_interfaces::srv::CookGPTsrv_Response stain(::mycobot_interfaces::srv::CookGPTsrv_Response::_stain_type arg)
  {
    msg_.stain = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mycobot_interfaces::srv::CookGPTsrv_Response msg_;
};

class Init_CookGPTsrv_Response_sauce
{
public:
  explicit Init_CookGPTsrv_Response_sauce(::mycobot_interfaces::srv::CookGPTsrv_Response & msg)
  : msg_(msg)
  {}
  Init_CookGPTsrv_Response_stain sauce(::mycobot_interfaces::srv::CookGPTsrv_Response::_sauce_type arg)
  {
    msg_.sauce = std::move(arg);
    return Init_CookGPTsrv_Response_stain(msg_);
  }

private:
  ::mycobot_interfaces::srv::CookGPTsrv_Response msg_;
};

class Init_CookGPTsrv_Response_dish
{
public:
  explicit Init_CookGPTsrv_Response_dish(::mycobot_interfaces::srv::CookGPTsrv_Response & msg)
  : msg_(msg)
  {}
  Init_CookGPTsrv_Response_sauce dish(::mycobot_interfaces::srv::CookGPTsrv_Response::_dish_type arg)
  {
    msg_.dish = std::move(arg);
    return Init_CookGPTsrv_Response_sauce(msg_);
  }

private:
  ::mycobot_interfaces::srv::CookGPTsrv_Response msg_;
};

class Init_CookGPTsrv_Response_rz
{
public:
  explicit Init_CookGPTsrv_Response_rz(::mycobot_interfaces::srv::CookGPTsrv_Response & msg)
  : msg_(msg)
  {}
  Init_CookGPTsrv_Response_dish rz(::mycobot_interfaces::srv::CookGPTsrv_Response::_rz_type arg)
  {
    msg_.rz = std::move(arg);
    return Init_CookGPTsrv_Response_dish(msg_);
  }

private:
  ::mycobot_interfaces::srv::CookGPTsrv_Response msg_;
};

class Init_CookGPTsrv_Response_ry
{
public:
  explicit Init_CookGPTsrv_Response_ry(::mycobot_interfaces::srv::CookGPTsrv_Response & msg)
  : msg_(msg)
  {}
  Init_CookGPTsrv_Response_rz ry(::mycobot_interfaces::srv::CookGPTsrv_Response::_ry_type arg)
  {
    msg_.ry = std::move(arg);
    return Init_CookGPTsrv_Response_rz(msg_);
  }

private:
  ::mycobot_interfaces::srv::CookGPTsrv_Response msg_;
};

class Init_CookGPTsrv_Response_rx
{
public:
  explicit Init_CookGPTsrv_Response_rx(::mycobot_interfaces::srv::CookGPTsrv_Response & msg)
  : msg_(msg)
  {}
  Init_CookGPTsrv_Response_ry rx(::mycobot_interfaces::srv::CookGPTsrv_Response::_rx_type arg)
  {
    msg_.rx = std::move(arg);
    return Init_CookGPTsrv_Response_ry(msg_);
  }

private:
  ::mycobot_interfaces::srv::CookGPTsrv_Response msg_;
};

class Init_CookGPTsrv_Response_z
{
public:
  explicit Init_CookGPTsrv_Response_z(::mycobot_interfaces::srv::CookGPTsrv_Response & msg)
  : msg_(msg)
  {}
  Init_CookGPTsrv_Response_rx z(::mycobot_interfaces::srv::CookGPTsrv_Response::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_CookGPTsrv_Response_rx(msg_);
  }

private:
  ::mycobot_interfaces::srv::CookGPTsrv_Response msg_;
};

class Init_CookGPTsrv_Response_y
{
public:
  explicit Init_CookGPTsrv_Response_y(::mycobot_interfaces::srv::CookGPTsrv_Response & msg)
  : msg_(msg)
  {}
  Init_CookGPTsrv_Response_z y(::mycobot_interfaces::srv::CookGPTsrv_Response::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_CookGPTsrv_Response_z(msg_);
  }

private:
  ::mycobot_interfaces::srv::CookGPTsrv_Response msg_;
};

class Init_CookGPTsrv_Response_x
{
public:
  Init_CookGPTsrv_Response_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CookGPTsrv_Response_y x(::mycobot_interfaces::srv::CookGPTsrv_Response::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_CookGPTsrv_Response_y(msg_);
  }

private:
  ::mycobot_interfaces::srv::CookGPTsrv_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mycobot_interfaces::srv::CookGPTsrv_Response>()
{
  return mycobot_interfaces::srv::builder::Init_CookGPTsrv_Response_x();
}

}  // namespace mycobot_interfaces


namespace mycobot_interfaces
{

namespace srv
{

namespace builder
{

class Init_CookGPTsrv_Event_response
{
public:
  explicit Init_CookGPTsrv_Event_response(::mycobot_interfaces::srv::CookGPTsrv_Event & msg)
  : msg_(msg)
  {}
  ::mycobot_interfaces::srv::CookGPTsrv_Event response(::mycobot_interfaces::srv::CookGPTsrv_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mycobot_interfaces::srv::CookGPTsrv_Event msg_;
};

class Init_CookGPTsrv_Event_request
{
public:
  explicit Init_CookGPTsrv_Event_request(::mycobot_interfaces::srv::CookGPTsrv_Event & msg)
  : msg_(msg)
  {}
  Init_CookGPTsrv_Event_response request(::mycobot_interfaces::srv::CookGPTsrv_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_CookGPTsrv_Event_response(msg_);
  }

private:
  ::mycobot_interfaces::srv::CookGPTsrv_Event msg_;
};

class Init_CookGPTsrv_Event_info
{
public:
  Init_CookGPTsrv_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CookGPTsrv_Event_request info(::mycobot_interfaces::srv::CookGPTsrv_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_CookGPTsrv_Event_request(msg_);
  }

private:
  ::mycobot_interfaces::srv::CookGPTsrv_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mycobot_interfaces::srv::CookGPTsrv_Event>()
{
  return mycobot_interfaces::srv::builder::Init_CookGPTsrv_Event_info();
}

}  // namespace mycobot_interfaces

#endif  // MYCOBOT_INTERFACES__SRV__DETAIL__COOK_GP_TSRV__BUILDER_HPP_

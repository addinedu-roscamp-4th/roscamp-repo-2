// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mycobot_interfaces:msg/MycobotSetCoords.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mycobot_interfaces/msg/mycobot_set_coords.hpp"


#ifndef MYCOBOT_INTERFACES__MSG__DETAIL__MYCOBOT_SET_COORDS__BUILDER_HPP_
#define MYCOBOT_INTERFACES__MSG__DETAIL__MYCOBOT_SET_COORDS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mycobot_interfaces/msg/detail/mycobot_set_coords__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mycobot_interfaces
{

namespace msg
{

namespace builder
{

class Init_MycobotSetCoords_model
{
public:
  explicit Init_MycobotSetCoords_model(::mycobot_interfaces::msg::MycobotSetCoords & msg)
  : msg_(msg)
  {}
  ::mycobot_interfaces::msg::MycobotSetCoords model(::mycobot_interfaces::msg::MycobotSetCoords::_model_type arg)
  {
    msg_.model = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mycobot_interfaces::msg::MycobotSetCoords msg_;
};

class Init_MycobotSetCoords_speed
{
public:
  explicit Init_MycobotSetCoords_speed(::mycobot_interfaces::msg::MycobotSetCoords & msg)
  : msg_(msg)
  {}
  Init_MycobotSetCoords_model speed(::mycobot_interfaces::msg::MycobotSetCoords::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_MycobotSetCoords_model(msg_);
  }

private:
  ::mycobot_interfaces::msg::MycobotSetCoords msg_;
};

class Init_MycobotSetCoords_rz
{
public:
  explicit Init_MycobotSetCoords_rz(::mycobot_interfaces::msg::MycobotSetCoords & msg)
  : msg_(msg)
  {}
  Init_MycobotSetCoords_speed rz(::mycobot_interfaces::msg::MycobotSetCoords::_rz_type arg)
  {
    msg_.rz = std::move(arg);
    return Init_MycobotSetCoords_speed(msg_);
  }

private:
  ::mycobot_interfaces::msg::MycobotSetCoords msg_;
};

class Init_MycobotSetCoords_ry
{
public:
  explicit Init_MycobotSetCoords_ry(::mycobot_interfaces::msg::MycobotSetCoords & msg)
  : msg_(msg)
  {}
  Init_MycobotSetCoords_rz ry(::mycobot_interfaces::msg::MycobotSetCoords::_ry_type arg)
  {
    msg_.ry = std::move(arg);
    return Init_MycobotSetCoords_rz(msg_);
  }

private:
  ::mycobot_interfaces::msg::MycobotSetCoords msg_;
};

class Init_MycobotSetCoords_rx
{
public:
  explicit Init_MycobotSetCoords_rx(::mycobot_interfaces::msg::MycobotSetCoords & msg)
  : msg_(msg)
  {}
  Init_MycobotSetCoords_ry rx(::mycobot_interfaces::msg::MycobotSetCoords::_rx_type arg)
  {
    msg_.rx = std::move(arg);
    return Init_MycobotSetCoords_ry(msg_);
  }

private:
  ::mycobot_interfaces::msg::MycobotSetCoords msg_;
};

class Init_MycobotSetCoords_z
{
public:
  explicit Init_MycobotSetCoords_z(::mycobot_interfaces::msg::MycobotSetCoords & msg)
  : msg_(msg)
  {}
  Init_MycobotSetCoords_rx z(::mycobot_interfaces::msg::MycobotSetCoords::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_MycobotSetCoords_rx(msg_);
  }

private:
  ::mycobot_interfaces::msg::MycobotSetCoords msg_;
};

class Init_MycobotSetCoords_y
{
public:
  explicit Init_MycobotSetCoords_y(::mycobot_interfaces::msg::MycobotSetCoords & msg)
  : msg_(msg)
  {}
  Init_MycobotSetCoords_z y(::mycobot_interfaces::msg::MycobotSetCoords::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_MycobotSetCoords_z(msg_);
  }

private:
  ::mycobot_interfaces::msg::MycobotSetCoords msg_;
};

class Init_MycobotSetCoords_x
{
public:
  Init_MycobotSetCoords_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MycobotSetCoords_y x(::mycobot_interfaces::msg::MycobotSetCoords::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_MycobotSetCoords_y(msg_);
  }

private:
  ::mycobot_interfaces::msg::MycobotSetCoords msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mycobot_interfaces::msg::MycobotSetCoords>()
{
  return mycobot_interfaces::msg::builder::Init_MycobotSetCoords_x();
}

}  // namespace mycobot_interfaces

#endif  // MYCOBOT_INTERFACES__MSG__DETAIL__MYCOBOT_SET_COORDS__BUILDER_HPP_
